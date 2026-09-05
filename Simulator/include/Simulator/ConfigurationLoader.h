/**
 * @file ConfigurationLoader.h
 * @brief Declares conversion of composition YAML into typed simulation data.
 *
 * Parsing is isolated in the Simulator and completes before worker threads are
 * started. Valid configurations remain executable while invalid nested inputs
 * are retained as score--1 rejected runs.
 */
#pragma once

#include <Common/Types.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <stdexcept>
#include <vector>

namespace simulator
{

/**
 * @brief Reports a top-level composition problem that prevents job enumeration.
 */
class ConfigurationError final : public std::runtime_error
{
public:
    /** @brief Inherits constructors accepting an explanatory error string. */
    using std::runtime_error::runtime_error;
};

/** @brief Source YAML paths for one successfully parsed requested run. */
struct RunConfigurationPaths
{
    std::filesystem::path simulation_config_file; ///< Simulation YAML path.
    std::filesystem::path mission_config_file; ///< Mission YAML path.
    std::filesystem::path drone_config_file; ///< Drone YAML path.
    std::filesystem::path lidar_config_file; ///< LiDAR YAML path.
};

/**
 * @brief Describes one requested run rejected during configuration loading.
 */
struct RejectedSimulationRun
{
    std::filesystem::path simulation_config_file; ///< Requested simulation YAML.
    std::filesystem::path mission_config_file; ///< Requested mission YAML.
    std::filesystem::path drone_config_file; ///< Requested drone YAML.
    std::filesystem::path lidar_config_file; ///< Requested LiDAR YAML.
    std::vector<common::types::ErrorRef> errors; ///< All rejection reasons.
    double score = -1.0; ///< Required score for a rejected run.
};

/** @brief Contains executable configurations and rejected requested runs. */
struct CompositionLoadResult
{
    simulator::types::SimulationCompositionData valid_composition; ///< Parsed executable data.
    std::vector<RunConfigurationPaths> valid_run_config_paths; ///< Paths aligned with valid runs.
    std::vector<RejectedSimulationRun> rejected_runs; ///< Invalid combinations retained for reporting.
};

/**
 * @brief Loads composition and nested YAML files into course data structures.
 *
 * The loader resolves relative paths, applies documented EX2 fallbacks, logs
 * every recovery or rejection, and never starts simulation threads.
 */
class ConfigurationLoader final
{
public:
    /**
     * @brief Constructs a loader using an existing error sink.
     * @param error_logger Non-owning logger that must outlive this loader.
     */
    explicit ConfigurationLoader(const ErrorLogger& error_logger);

    /**
     * @brief Loads a simulation composition and every referenced YAML file.
     * @param composition_file Readable top-level composition path.
     * @return Valid typed inputs, source paths, and rejected combinations.
     * @throws ConfigurationError If the top-level composition is unusable.
     * @throws std::runtime_error If immediate error logging fails.
     */
    [[nodiscard]] CompositionLoadResult loadSimulationComposition(
        const std::filesystem::path& composition_file) const;

private:
    const ErrorLogger& error_logger_; ///< Non-owning immediate diagnostic sink.
};

} // namespace simulator
