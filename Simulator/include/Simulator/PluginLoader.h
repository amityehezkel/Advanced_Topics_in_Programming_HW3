/**
 * @file PluginLoader.h
 * @brief Declares discovery and validated loading of Algorithm and MissionControl plugins.
 *
 * Each descriptor owns both its factory and the .so handle that contains the
 * factory's code. Folder discovery is deterministic and each library is loaded
 * at most once per Simulator invocation.
 */
#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/FactoryRegistry.h>
#include <Simulator/SharedLibrary.h>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace simulator
{

/** @brief Reports shared-library loading or registration validation failures. */
class PluginLoadError final : public std::runtime_error
{
public:
    /** @brief Inherits constructors accepting an explanatory error string. */
    using std::runtime_error::runtime_error;
};

/** @brief Owns one loaded MappingAlgorithm factory and its library lifetime. */
class MappingAlgorithmPlugin final
{
public:
    /**
     * @brief Constructs an owning Algorithm plugin descriptor.
     * @param library_file Canonical .so path used for reporting.
     * @param library Loaded library whose code implements the factory.
     * @param factory Exactly one factory extracted for this library.
     */
    MappingAlgorithmPlugin(std::filesystem::path library_file,
                           SharedLibrary library,
                           common::MappingAlgorithmFactory factory);

    /** @brief Transfers plugin ownership without unloading it. */
    MappingAlgorithmPlugin(MappingAlgorithmPlugin&&) noexcept = default;

    /** @brief Move assignment is disabled to preserve factory-before-library teardown. */
    MappingAlgorithmPlugin& operator=(MappingAlgorithmPlugin&&) = delete;

    /** @brief Plugin descriptors cannot share ownership through copying. */
    MappingAlgorithmPlugin(const MappingAlgorithmPlugin&) = delete;

    /** @brief Plugin descriptors cannot be copy-assigned. */
    MappingAlgorithmPlugin& operator=(const MappingAlgorithmPlugin&) = delete;

    /**
     * @brief Returns the plugin file used for reports and output names.
     * @return Canonical Algorithm .so path.
     */
    [[nodiscard]] const std::filesystem::path& path() const noexcept;

    /**
     * @brief Returns the registered construction function.
     * @return Factory valid while this descriptor remains alive.
     */
    [[nodiscard]] const common::MappingAlgorithmFactory& factory() const noexcept;

private:
    std::filesystem::path library_file_; ///< Stable plugin identity.
    SharedLibrary library_; ///< Declared before factory so it is destroyed after it.
    common::MappingAlgorithmFactory factory_; ///< Factory code resident in library_.
};

/** @brief Owns one loaded MissionControl factory and its library lifetime. */
class MissionControlPlugin final
{
public:
    /**
     * @brief Constructs an owning MissionControl plugin descriptor.
     * @param library_file Canonical .so path used for reporting.
     * @param library Loaded library whose code implements the factory.
     * @param factory Exactly one factory extracted for this library.
     */
    MissionControlPlugin(std::filesystem::path library_file,
                         SharedLibrary library,
                         common::MissionControlFactory factory);

    /** @brief Transfers plugin ownership without unloading it. */
    MissionControlPlugin(MissionControlPlugin&&) noexcept = default;

    /** @brief Move assignment is disabled to preserve factory-before-library teardown. */
    MissionControlPlugin& operator=(MissionControlPlugin&&) = delete;

    /** @brief Plugin descriptors cannot share ownership through copying. */
    MissionControlPlugin(const MissionControlPlugin&) = delete;

    /** @brief Plugin descriptors cannot be copy-assigned. */
    MissionControlPlugin& operator=(const MissionControlPlugin&) = delete;

    /**
     * @brief Returns the plugin file used for reports and output names.
     * @return Canonical MissionControl .so path.
     */
    [[nodiscard]] const std::filesystem::path& path() const noexcept;

    /**
     * @brief Returns the registered construction function.
     * @return Factory valid while this descriptor remains alive.
     */
    [[nodiscard]] const common::MissionControlFactory& factory() const noexcept;

private:
    std::filesystem::path library_file_; ///< Stable plugin identity.
    SharedLibrary library_; ///< Declared before factory so it is destroyed after it.
    common::MissionControlFactory factory_; ///< Factory code resident in library_.
};

/** @brief Loads individual plugins and sorted plugin folders through FactoryRegistry. */
class PluginLoader final
{
public:
    /**
     * @brief Constructs a loader over the process registration target.
     * @param registry Non-owning registry used to observe static registration.
     */
    explicit PluginLoader(FactoryRegistry& registry = FactoryRegistry::instance()) noexcept;

    /**
     * @brief Loads one Algorithm library and requires exactly one registration.
     * @param library_file Readable .so file.
     * @return Owning plugin descriptor.
     * @throws PluginLoadError If loading or registration validation fails.
     */
    [[nodiscard]] MappingAlgorithmPlugin loadMappingAlgorithm(
        const std::filesystem::path& library_file) const;

    /**
     * @brief Loads one MissionControl library and requires exactly one registration.
     * @param library_file Readable .so file.
     * @return Owning plugin descriptor.
     * @throws PluginLoadError If loading or registration validation fails.
     */
    [[nodiscard]] MissionControlPlugin loadMissionControl(
        const std::filesystem::path& library_file) const;

    /**
     * @brief Loads every Algorithm .so in deterministic filename order.
     * @param folder Readable directory containing at least one .so file.
     * @return Successfully loaded Algorithm descriptors.
     */
    [[nodiscard]] std::vector<MappingAlgorithmPlugin> loadMappingAlgorithms(
        const std::filesystem::path& folder) const;

    /**
     * @brief Loads every MissionControl .so in deterministic filename order.
     * @param folder Readable directory containing at least one .so file.
     * @return Successfully loaded MissionControl descriptors.
     */
    [[nodiscard]] std::vector<MissionControlPlugin> loadMissionControls(
        const std::filesystem::path& folder) const;

private:
    FactoryRegistry& registry_; ///< Process-wide registration destination.
};

} // namespace simulator
