#include <Simulator/SimulationManager.h>

#include <Simulator/OutputManager.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

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

[[nodiscard]] types::SimulationResult failedResult(
    const types::SimulationConfigData& simulation,
    const common::types::MissionConfigData& mission,
    std::string message)
{
    types::SimulationResult result;
    result.simulation_config = simulation;
    result.mission_config = mission;
    result.mission_results.push_back(common::types::MissionRunResult{
        common::types::MissionRunStatus::Error, 0,
        {{std::string{errorCodeName(ErrorCode::SimulationRunFailed)},
          std::move(message)}}});
    result.mission_score = -1.0;
    return result;
}

void logRunFailure(const ErrorLogger* logger,
                   const types::SimulationConfigData& simulation,
                   std::string_view message) noexcept
{
    if (logger == nullptr) return;
    try {
        logger->scenarioFailure(ErrorCode::SimulationRunFailed,
                                "Map '" + simulation.map_filename.string() +
                                    "' failed: " + std::string{message});
    } catch (...) {
        // A secondary logging failure must not terminate a worker thread.
    }
}

} // namespace

SimulationManager::SimulationManager(
    std::unique_ptr<ISimulationRunFactory> run_factory,
    std::size_t simulation_threads,
    const ErrorLogger* error_logger)
    : run_factory_(std::move(run_factory)),
      simulation_threads_(simulation_threads), error_logger_(error_logger)
{
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
    if (simulation_threads_ == 0) {
        throw std::invalid_argument("Simulation thread count must be positive.");
    }
}

types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path)
{
    const std::size_t total = [&composition] {
        std::size_t count = 0;
        for (const auto& [unused, missions] :
             composition.simulation_mission_groups) {
            static_cast<void>(unused);
            count += missions.size() * composition.drone_configs.size() *
                     composition.lidar_configs.size();
        }
        return count;
    }();

    std::vector<types::SimulationResult> results(total);
    std::vector<PreparedRun> prepared;
    prepared.reserve(total);
    std::size_t result_index = 0;
    for (const auto& [simulation, missions] :
         composition.simulation_mission_groups) {
        for (const auto& mission : missions) {
            for (const auto& drone : composition.drone_configs) {
                for (const auto& lidar : composition.lidar_configs) {
                    results[result_index].simulation_config = simulation;
                    results[result_index].mission_config = mission;
                    results[result_index].mission_score = -1.0;
                    try {
                        const auto run_directory = OutputManager::createRunDirectory(
                            output_path, result_index,
                            simulation.map_filename.stem().string());
                        auto run = run_factory_->create(
                            simulation, mission, drone, lidar, run_directory);
                        if (!run) {
                            throw std::runtime_error(
                                "Simulation run factory returned null.");
                        }
                        prepared.push_back(
                            PreparedRun{result_index, std::move(run)});
                    } catch (const std::exception& error) {
                        logRunFailure(error_logger_, simulation, error.what());
                        results[result_index] =
                            failedResult(simulation, mission, error.what());
                    } catch (...) {
                        constexpr std::string_view message =
                            "Unknown exception while preparing simulation run.";
                        logRunFailure(error_logger_, simulation, message);
                        results[result_index] =
                            failedResult(simulation, mission, std::string{message});
                    }
                    ++result_index;
                }
            }
        }
    }

    executePreparedRuns(prepared, results);
    return {composition.composition_file, generatedAtUtc(),
            "output_map_accuracy", {0.0, 100.0}, -1, std::move(results)};
}

void SimulationManager::executePreparedRuns(
    std::vector<PreparedRun>& prepared_runs,
    std::vector<types::SimulationResult>& results) const
{
    const auto execute = [this, &prepared_runs, &results](std::size_t index) {
        PreparedRun& prepared = prepared_runs[index];
        try {
            types::SimulationResult result = prepared.run->run();
            if (result.mission_score < 0.0) {
                bool described = false;
                for (const auto& mission : result.mission_results) {
                    for (const auto& error : mission.errors) {
                        described = true;
                        logRunFailure(error_logger_, result.simulation_config,
                                      error.code + ": " + error.message);
                    }
                }
                if (!described) {
                    logRunFailure(error_logger_, result.simulation_config,
                                  "Mission returned an error result.");
                }
            }
            results[prepared.result_index] = std::move(result);
        } catch (const std::exception& error) {
            const auto& prior = results[prepared.result_index];
            logRunFailure(error_logger_, prior.simulation_config, error.what());
            results[prepared.result_index] = failedResult(
                prior.simulation_config, prior.mission_config, error.what());
        } catch (...) {
            constexpr std::string_view message =
                "Unknown exception while executing simulation run.";
            const auto& prior = results[prepared.result_index];
            logRunFailure(error_logger_, prior.simulation_config, message);
            results[prepared.result_index] = failedResult(
                prior.simulation_config, prior.mission_config,
                std::string{message});
        }
    };

    if (simulation_threads_ == 1 || prepared_runs.size() <= 1) {
        for (std::size_t index = 0; index < prepared_runs.size(); ++index) {
            execute(index);
        }
        return;
    }

    std::atomic_size_t next{0};
    const std::size_t worker_count =
        std::min(simulation_threads_, prepared_runs.size());
    std::vector<std::jthread> workers;
    workers.reserve(worker_count);
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        static_cast<void>(worker);
        workers.emplace_back([&execute, &next, count = prepared_runs.size()] {
            while (true) {
                const std::size_t index = next.fetch_add(1);
                if (index >= count) return;
                execute(index);
            }
        });
    }
}

} // namespace simulator
