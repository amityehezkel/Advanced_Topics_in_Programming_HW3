#include <Simulator/SimulatorApplication.h>

#include <Simulator/ConfigurationLoader.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/OutputManager.h>
#include <Simulator/PluginLoader.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationReportWriter.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace simulator
{
namespace
{

[[nodiscard]] std::string generatedAtUtc()
{
    const std::time_t now = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    std::ostringstream output;
    output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return output.str();
}

[[nodiscard]] std::vector<std::filesystem::path> pluginFiles(
    const std::filesystem::path& folder)
{
    std::vector<std::filesystem::path> result;
    std::set<std::filesystem::path> seen;
    std::error_code error;
    for (std::filesystem::directory_iterator iterator{folder, error}, end;
         !error && iterator != end; iterator.increment(error)) {
        std::error_code entry_error;
        if (iterator->is_regular_file(entry_error) && !entry_error &&
            iterator->path().extension() == ".so") {
            const auto canonical =
                std::filesystem::weakly_canonical(iterator->path(), entry_error);
            if (entry_error) {
                throw std::runtime_error("Cannot resolve plugin path '" +
                                         iterator->path().string() + "': " +
                                         entry_error.message());
            }
            if (seen.insert(canonical).second) {
                result.push_back(canonical);
            }
        }
    }
    if (error) {
        throw std::runtime_error("Cannot enumerate plugin folder '" +
                                 folder.string() + "': " + error.message());
    }
    std::sort(result.begin(), result.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.filename().string() < rhs.filename().string();
    });
    return result;
}

[[nodiscard]] PluginResultSummary summarize(
    std::string plugin,
    const types::SimulationManagerReport& report,
    const std::vector<RejectedSimulationRun>& rejected)
{
    PluginResultSummary result;
    result.plugin_file = std::move(plugin);
    for (const auto& run : report.runs) {
        result.total_score += run.mission_score;
        for (const auto& mission : run.mission_results) {
            result.total_steps += mission.steps;
        }
    }
    for (const auto& rejected_run : rejected) {
        result.total_score += rejected_run.score;
    }
    return result;
}

void recordPluginError(const ErrorLogger& logger,
                       std::vector<std::string>& errors,
                       const std::filesystem::path& plugin,
                       std::string_view message)
{
    const std::string plugin_name = plugin.filename().string();
    const std::string description = plugin_name + ": " + std::string{message};
    errors.push_back(plugin_name);
    logger.scenarioFailure(ErrorCode::PluginLoadFailed, description);
}

} // namespace

SimulatorApplication::SimulatorApplication(std::ostream& output,
                                           std::ostream& error) noexcept
    : output_(output), error_(error)
{
}

int SimulatorApplication::run(const CommandLineOptions& options) const
{
    try {
        return options.mode == RunMode::Comparative
                   ? runComparative(options)
                   : runCompetition(options);
    } catch (const std::exception& exception) {
        error_ << "Simulator failed: " << exception.what() << '\n';
        return 1;
    } catch (...) {
        error_ << "Simulator failed with an unknown error.\n";
        return 1;
    }
}

int SimulatorApplication::runComparative(
    const CommandLineOptions& options) const
{
    if (!options.mission_control_folder || !options.algorithm_file) {
        throw std::invalid_argument("Incomplete comparative options.");
    }
    const auto results_directory = OutputManager::createResultsDirectory(
        *options.mission_control_folder, "comparative_results");
    ErrorLogger logger{results_directory / "errors.log"};
    ConfigurationLoader configuration_loader{logger};
    const auto composition = configuration_loader.loadSimulationComposition(
        options.composition_file);
    PluginLoader plugin_loader;

    MappingAlgorithmPlugin algorithm = [&] {
        try {
            return plugin_loader.loadMappingAlgorithm(*options.algorithm_file);
        } catch (const std::exception& exception) {
            logger.fatal(ErrorCode::PluginLoadFailed, exception.what());
            throw;
        }
    }();

    ComparativeReportData aggregate{
        options.composition_file, *options.mission_control_folder,
        generatedAtUtc(), {}, {}};
    const auto plugins = pluginFiles(*options.mission_control_folder);
    std::size_t plugin_index = 0;
    for (const auto& plugin_file : plugins) {
        try {
            MissionControlPlugin mission_control =
                plugin_loader.loadMissionControl(plugin_file);
            const auto plugin_directory = OutputManager::createRunDirectory(
                results_directory, plugin_index, plugin_file.stem().string());
            auto factory = std::make_unique<SimulationRunFactoryImpl>(
                algorithm.factory(), mission_control.factory(), options.verbose,
                &logger);
            SimulationManager manager{std::move(factory),
                                      options.simulation_threads, &logger};
            const auto report =
                manager.run(composition.valid_composition, plugin_directory);
            SimulationReportWriter::writeDetailedReport(
                report, composition.valid_run_config_paths,
                composition.rejected_runs, options.composition_file,
                results_directory /
                    ("simulation_results_" +
                     OutputManager::sanitizeFilenameComponent(
                     plugin_file.stem().string()) +
                     ".yaml"));
            const bool produced_result = std::any_of(
                report.runs.begin(), report.runs.end(),
                [](const auto& run) { return run.mission_score >= 0.0; });
            if (produced_result) {
                aggregate.successful_results.push_back(summarize(
                    plugin_file.filename().string(), report,
                    composition.rejected_runs));
            } else {
                recordPluginError(logger, aggregate.errors, plugin_file,
                                  "did not complete any simulation run");
            }
        } catch (const std::exception& exception) {
            recordPluginError(logger, aggregate.errors, plugin_file,
                              exception.what());
        } catch (...) {
            recordPluginError(logger, aggregate.errors, plugin_file,
                              "unknown plugin exception");
        }
        ++plugin_index;
    }
    if (plugins.empty()) {
        aggregate.errors.push_back("No MissionControl .so files were found.");
        logger.scenarioFailure(ErrorCode::PluginLoadFailed,
                               aggregate.errors.back());
    }
    SimulationReportWriter::writeComparativeReport(
        aggregate, results_directory / "comparative_results.yaml");
    output_ << "Comparative results written to " << results_directory << '\n';
    return aggregate.successful_results.empty() ? 1 : 0;
}

int SimulatorApplication::runCompetition(
    const CommandLineOptions& options) const
{
    if (!options.mission_control_file || !options.algorithms_folder) {
        throw std::invalid_argument("Incomplete competition options.");
    }
    const auto results_directory = OutputManager::createResultsDirectory(
        *options.algorithms_folder, "competition");
    ErrorLogger logger{results_directory / "errors.log"};
    ConfigurationLoader configuration_loader{logger};
    const auto composition = configuration_loader.loadSimulationComposition(
        options.composition_file);
    PluginLoader plugin_loader;

    MissionControlPlugin mission_control = [&] {
        try {
            return plugin_loader.loadMissionControl(*options.mission_control_file);
        } catch (const std::exception& exception) {
            logger.fatal(ErrorCode::PluginLoadFailed, exception.what());
            throw;
        }
    }();

    CompetitionReportData aggregate{
        options.composition_file, options.mission_control_file->filename(),
        generatedAtUtc(), {}, {}};
    const auto plugins = pluginFiles(*options.algorithms_folder);
    std::size_t plugin_index = 0;
    for (const auto& plugin_file : plugins) {
        try {
            MappingAlgorithmPlugin algorithm =
                plugin_loader.loadMappingAlgorithm(plugin_file);
            const auto plugin_directory = OutputManager::createRunDirectory(
                results_directory, plugin_index, plugin_file.stem().string());
            auto factory = std::make_unique<SimulationRunFactoryImpl>(
                algorithm.factory(), mission_control.factory(), options.verbose,
                &logger);
            SimulationManager manager{std::move(factory),
                                      options.simulation_threads, &logger};
            const auto report =
                manager.run(composition.valid_composition, plugin_directory);
            SimulationReportWriter::writeDetailedReport(
                report, composition.valid_run_config_paths,
                composition.rejected_runs, options.composition_file,
                results_directory /
                    ("simulation_results_" +
                     OutputManager::sanitizeFilenameComponent(
                     plugin_file.stem().string()) +
                     ".yaml"));
            const bool produced_result = std::any_of(
                report.runs.begin(), report.runs.end(),
                [](const auto& run) { return run.mission_score >= 0.0; });
            if (produced_result) {
                aggregate.successful_results.push_back(summarize(
                    plugin_file.filename().string(), report,
                    composition.rejected_runs));
            } else {
                recordPluginError(logger, aggregate.errors, plugin_file,
                                  "did not complete any simulation run");
            }
        } catch (const std::exception& exception) {
            recordPluginError(logger, aggregate.errors, plugin_file,
                              exception.what());
        } catch (...) {
            recordPluginError(logger, aggregate.errors, plugin_file,
                              "unknown plugin exception");
        }
        ++plugin_index;
    }
    if (plugins.empty()) {
        aggregate.errors.push_back("No Algorithm .so files were found.");
        logger.scenarioFailure(ErrorCode::PluginLoadFailed,
                               aggregate.errors.back());
    }
    SimulationReportWriter::writeCompetitionReport(
        aggregate, results_directory / "competition_results.yaml");
    output_ << "Competition results written to " << results_directory << '\n';
    return aggregate.successful_results.empty() ? 1 : 0;
}

} // namespace simulator
