/**
 * @file SimulationRunFactoryImpl.h
 * @brief Declares construction of one independent simulation object graph.
 *
 * The factory preserves EX2 map and hardware creation while replacing direct
 * construction of concrete plugins with dynamically registered factories.
 */
#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/ISimulationRunFactory.h>

#include <filesystem>

namespace simulator
{

/**
 * @brief Creates SimulationRunImpl instances for one selected plugin pairing.
 *
 * The factory keeps no mutable run counter. Callers provide a unique per-run
 * output directory, making concurrent execution deterministic.
 */
class SimulationRunFactoryImpl final : public ISimulationRunFactory
{
public:
    /**
     * @brief Stores factories and invocation-wide options.
     * @param algorithm_factory Factory from one loaded Algorithm plugin.
     * @param mission_control_factory Factory from one loaded MissionControl plugin.
     * @param verbose Whether newly created MissionControl instances log verbosely.
     * @param error_logger Optional non-owning Simulator error sink.
     */
    SimulationRunFactoryImpl(
        common::MappingAlgorithmFactory algorithm_factory,
        common::MissionControlFactory mission_control_factory,
        bool verbose,
        const ErrorLogger* error_logger = nullptr);

    /**
     * @brief Creates one independent run for a Cartesian-product configuration.
     * @param simulation_config Map, pose, and simulation geometry.
     * @param mission_config Mission bounds, resolution request, and step limit.
     * @param drone_config Drone radius and per-command limits.
     * @param lidar_config LiDAR range and field-of-view configuration.
     * @param output_path Unique directory reserved for this run.
     * @return Fully owned run ready to execute on any worker.
     * @throws std::runtime_error If map loading, validation, or wiring fails.
     */
    [[nodiscard]] std::unique_ptr<ISimulationRun> create(
        const types::SimulationConfigData& simulation_config,
        const common::types::MissionConfigData& mission_config,
        const common::types::DroneConfigData& drone_config,
        const common::types::LidarConfigData& lidar_config,
        const std::filesystem::path& output_path) override;

private:
    common::MappingAlgorithmFactory algorithm_factory_; ///< Fresh Algorithm creator.
    common::MissionControlFactory mission_control_factory_; ///< Fresh MissionControl creator.
    bool verbose_ = false; ///< Forwarded to MissionControlDependencies.
    const ErrorLogger* error_logger_ = nullptr; ///< Optional non-owning diagnostic sink.
};

} // namespace simulator
