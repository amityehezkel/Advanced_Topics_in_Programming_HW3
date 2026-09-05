/**
 * @file DroneControlImpl.h
 * @brief Declares the adapter between a mapping algorithm and drone hardware.
 *
 * The adapter validates one mapping command at a time, delegates movement and
 * scanning to injected course interfaces, and updates the mutable output map.
 */
#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <MissionControl/IDroneControl.h>

#include <cstddef>
#include <optional>

namespace mission_control_212200943
{

/**
 * @brief Executes mapping-algorithm commands against injected drone hardware.
 *
 * Each instance belongs to exactly one MissionControl instance and therefore
 * keeps only per-mission state. All referenced dependencies must outlive it.
 */
class DroneControlImpl final : public mission_control::IDroneControl
{
public:
    /**
     * @brief Constructs a ready-to-run drone controller.
     * @param drone Drone movement limits, copied into the controller.
     * @param mission Mission limits and mapping bounds, copied into the controller.
     * @param lidar Non-owning reference to the simulation LiDAR.
     * @param gps Non-owning reference to the simulation GPS.
     * @param movement Non-owning reference to the movement driver.
     * @param output_map Non-owning reference to the generated mutable map.
     * @param mapping_algorithm Non-owning reference to the selected algorithm.
     */
    DroneControlImpl(common::types::DroneConfigData drone,
                     common::types::MissionConfigData mission,
                     common::ILidar& lidar,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);

    /**
     * @brief Executes one algorithm, movement, and optional scan cycle.
     * @return Continue, completed, or error status with an explanatory message.
     */
    [[nodiscard]] common::types::DroneStepResult step() override;

    /**
     * @brief Reads the current GPS-observed drone state.
     * @return Position, heading, and zero-based executed-step index.
     */
    [[nodiscard]] common::types::DroneState state() const override;

private:
    common::types::DroneConfigData drone_; ///< Copied movement constraints.
    common::types::MissionConfigData mission_; ///< Copied mission constraints.
    common::ILidar& lidar_; ///< LiDAR supplied by the Simulator.
    common::IGPS& gps_; ///< GPS supplied by the Simulator.
    common::IDroneMovement& movement_; ///< Movement driver supplied by the Simulator.
    common::IMutableMap3D& output_map_; ///< Per-run output map.
    common::IMappingAlgorithm& mapping_algorithm_; ///< Per-run mapping strategy.
    std::size_t step_index_ = 0; ///< Number of completed controller cycles.
    std::optional<common::types::LidarScanResult> latest_scan_; ///< Last scan passed to the algorithm.
};

} // namespace mission_control_212200943
