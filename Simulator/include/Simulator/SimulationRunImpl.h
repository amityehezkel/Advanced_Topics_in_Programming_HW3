/**
 * @file SimulationRunImpl.h
 * @brief Declares ownership and execution of one fully wired simulation run.
 *
 * A run owns its maps, simulated hardware, Algorithm, and MissionControl. The
 * declaration order guarantees that plugin-created objects are destroyed before
 * the dependencies they reference.
 */
#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Simulator/ISimulationRun.h>

#include <filesystem>
#include <memory>

namespace simulator
{

/** @brief Owns and executes the complete object graph for one configuration node. */
class SimulationRunImpl final : public ISimulationRun
{
public:
    /**
     * @brief Constructs one independent simulation run.
     * @param hidden_map Ground-truth map used by hardware and scoring.
     * @param output_map Empty mutable map exposed to plugins.
     * @param gps Simulated GPS referenced by hardware and MissionControl.
     * @param movement Simulated movement driver.
     * @param lidar Simulated LiDAR.
     * @param mapping_algorithm Fresh Algorithm instance for this run.
     * @param mission_control Fresh MissionControl instance for this run.
     * @param simulation_config Typed simulation configuration returned in results.
     * @param mission_config Typed mission configuration returned in results.
     * @param output_map_file Unique output-map destination.
     * @param resolution_status Simulator decision for requested output resolution.
     */
    SimulationRunImpl(
        std::unique_ptr<const common::IMap3D> hidden_map,
        std::unique_ptr<common::IMutableMap3D> output_map,
        std::unique_ptr<common::IGPS> gps,
        std::unique_ptr<common::IDroneMovement> movement,
        std::unique_ptr<common::ILidar> lidar,
        std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
        std::unique_ptr<common::IMissionControl> mission_control,
        types::SimulationConfigData simulation_config,
        common::types::MissionConfigData mission_config,
        std::filesystem::path output_map_file,
        types::ResolutionRequestStatus resolution_status);

    /**
     * @brief Runs MissionControl and scores every non-error terminal output map.
     * @return Complete course SimulationResult for this configuration node.
     */
    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<const common::IMap3D> hidden_map_; ///< Ground truth; destroyed last.
    std::unique_ptr<common::IMutableMap3D> output_map_; ///< Generated per-run map.
    std::unique_ptr<common::IGPS> gps_; ///< Per-run pose source.
    std::unique_ptr<common::IDroneMovement> movement_; ///< Per-run movement hardware.
    std::unique_ptr<common::ILidar> lidar_; ///< Per-run sensor hardware.
    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm_; ///< Plugin strategy.
    std::unique_ptr<common::IMissionControl> mission_control_; ///< Destroyed before dependencies.
    types::SimulationConfigData simulation_config_; ///< Original simulation input.
    common::types::MissionConfigData mission_config_; ///< Original mission input.
    std::filesystem::path output_map_file_; ///< Unique generated map path.
    types::ResolutionRequestStatus resolution_status_ =
        types::ResolutionRequestStatus::Ignored; ///< Output-resolution decision.
};

} // namespace simulator
