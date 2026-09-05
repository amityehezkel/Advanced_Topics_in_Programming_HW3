#include <Simulator/PluginLoader.h>

#include <algorithm>
#include <set>
#include <system_error>
#include <utility>

namespace simulator
{
namespace
{

[[nodiscard]] std::filesystem::path canonicalLibraryPath(
    const std::filesystem::path& library_file)
{
    std::error_code error;
    if (!std::filesystem::is_regular_file(library_file, error) || error) {
        throw PluginLoadError("Plugin is not a readable regular file: '" +
                              library_file.string() + "'.");
    }
    if (library_file.extension() != ".so") {
        throw PluginLoadError("Plugin must have the .so extension: '" +
                              library_file.string() + "'.");
    }
    const auto canonical = std::filesystem::weakly_canonical(library_file, error);
    if (error) {
        throw PluginLoadError("Cannot resolve plugin path '" +
                              library_file.string() + "': " + error.message());
    }
    return canonical;
}

[[nodiscard]] std::vector<std::filesystem::path> librariesIn(
    const std::filesystem::path& folder)
{
    std::error_code error;
    if (!std::filesystem::is_directory(folder, error) || error) {
        throw PluginLoadError("Plugin folder is not readable: '" +
                              folder.string() + "'.");
    }
    std::vector<std::filesystem::path> libraries;
    std::set<std::filesystem::path> seen;
    for (std::filesystem::directory_iterator iterator{folder, error}, end;
         !error && iterator != end; iterator.increment(error)) {
        std::error_code file_error;
        if (iterator->is_regular_file(file_error) && !file_error &&
            iterator->path().extension() == ".so") {
            const auto canonical =
                std::filesystem::weakly_canonical(iterator->path(), file_error);
            if (file_error) {
                throw PluginLoadError("Cannot resolve plugin path '" +
                                      iterator->path().string() + "': " +
                                      file_error.message());
            }
            if (seen.insert(canonical).second) {
                libraries.push_back(canonical);
            }
        }
    }
    if (error) {
        throw PluginLoadError("Cannot enumerate plugin folder '" +
                              folder.string() + "': " + error.message());
    }
    std::sort(libraries.begin(), libraries.end(),
              [](const auto& lhs, const auto& rhs) {
                  return lhs.filename().string() < rhs.filename().string();
              });
    if (libraries.empty()) {
        throw PluginLoadError("Plugin folder contains no .so files: '" +
                              folder.string() + "'.");
    }
    return libraries;
}

} // namespace

MappingAlgorithmPlugin::MappingAlgorithmPlugin(
    std::filesystem::path library_file,
    SharedLibrary library,
    common::MappingAlgorithmFactory factory)
    : library_file_(std::move(library_file)), library_(std::move(library)),
      factory_(std::move(factory))
{
}

const std::filesystem::path& MappingAlgorithmPlugin::path() const noexcept
{
    return library_file_;
}

const common::MappingAlgorithmFactory& MappingAlgorithmPlugin::factory() const noexcept
{
    return factory_;
}

MissionControlPlugin::MissionControlPlugin(
    std::filesystem::path library_file,
    SharedLibrary library,
    common::MissionControlFactory factory)
    : library_file_(std::move(library_file)), library_(std::move(library)),
      factory_(std::move(factory))
{
}

const std::filesystem::path& MissionControlPlugin::path() const noexcept
{
    return library_file_;
}

const common::MissionControlFactory& MissionControlPlugin::factory() const noexcept
{
    return factory_;
}

PluginLoader::PluginLoader(FactoryRegistry& registry) noexcept : registry_(registry)
{
}

MappingAlgorithmPlugin PluginLoader::loadMappingAlgorithm(
    const std::filesystem::path& library_file) const
{
    const auto path = canonicalLibraryPath(library_file);
    const std::size_t algorithm_checkpoint = registry_.mappingAlgorithmCount();
    const std::size_t control_checkpoint = registry_.missionControlCount();
    try {
        SharedLibrary library{path};
        auto algorithms =
            registry_.extractMappingAlgorithmsFrom(algorithm_checkpoint);
        auto mission_controls =
            registry_.extractMissionControlsFrom(control_checkpoint);
        if (algorithms.size() != 1 || !mission_controls.empty()) {
            throw PluginLoadError(
                "Algorithm plugin must register exactly one MappingAlgorithm "
                "and no MissionControl: '" + path.string() + "'.");
        }
        return MappingAlgorithmPlugin{
            path, std::move(library), std::move(algorithms.front())};
    } catch (const PluginLoadError&) {
        throw;
    } catch (const std::exception& error) {
        // Remove any partial registrations left by a failed dlopen.
        static_cast<void>(
            registry_.extractMappingAlgorithmsFrom(algorithm_checkpoint));
        static_cast<void>(
            registry_.extractMissionControlsFrom(control_checkpoint));
        throw PluginLoadError("Failed to load Algorithm plugin '" +
                              path.string() + "': " + error.what());
    }
}

MissionControlPlugin PluginLoader::loadMissionControl(
    const std::filesystem::path& library_file) const
{
    const auto path = canonicalLibraryPath(library_file);
    const std::size_t algorithm_checkpoint = registry_.mappingAlgorithmCount();
    const std::size_t control_checkpoint = registry_.missionControlCount();
    try {
        SharedLibrary library{path};
        auto algorithms =
            registry_.extractMappingAlgorithmsFrom(algorithm_checkpoint);
        auto mission_controls =
            registry_.extractMissionControlsFrom(control_checkpoint);
        if (!algorithms.empty() || mission_controls.size() != 1) {
            throw PluginLoadError(
                "MissionControl plugin must register exactly one MissionControl "
                "and no MappingAlgorithm: '" + path.string() + "'.");
        }
        return MissionControlPlugin{
            path, std::move(library), std::move(mission_controls.front())};
    } catch (const PluginLoadError&) {
        throw;
    } catch (const std::exception& error) {
        static_cast<void>(
            registry_.extractMappingAlgorithmsFrom(algorithm_checkpoint));
        static_cast<void>(
            registry_.extractMissionControlsFrom(control_checkpoint));
        throw PluginLoadError("Failed to load MissionControl plugin '" +
                              path.string() + "': " + error.what());
    }
}

std::vector<MappingAlgorithmPlugin> PluginLoader::loadMappingAlgorithms(
    const std::filesystem::path& folder) const
{
    const auto libraries = librariesIn(folder);
    std::vector<MappingAlgorithmPlugin> result;
    result.reserve(libraries.size());
    for (const auto& library : libraries) {
        result.push_back(loadMappingAlgorithm(library));
    }
    return result;
}

std::vector<MissionControlPlugin> PluginLoader::loadMissionControls(
    const std::filesystem::path& folder) const
{
    const auto libraries = librariesIn(folder);
    std::vector<MissionControlPlugin> result;
    result.reserve(libraries.size());
    for (const auto& library : libraries) {
        result.push_back(loadMissionControl(library));
    }
    return result;
}

} // namespace simulator
