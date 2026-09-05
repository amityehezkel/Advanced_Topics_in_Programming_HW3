/**
 * @file ScanResultToVoxels.h
 * @brief Declares conversion of LiDAR observations into occupancy-map updates.
 *
 * This stateless MissionControl helper translates relative beam measurements
 * to world coordinates and records free, occupied, or uncertain evidence.
 */
#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control_212200943
{

/** @brief Applies one LiDAR scan to a mutable output map. */
class ScanResultToVoxels final
{
public:
    /**
     * @brief Converts every scan beam to world-space voxel observations.
     * @param output_map Map receiving Empty, Occupied, and PotentiallyOccupied values.
     * @param scan_origin World-space sensor position at scan time.
     * @param drone_heading World-space drone orientation at scan time.
     * @param scan LiDAR hits whose beam angles are relative to the requested scan.
     * @param lidar_config Sensor range and field-of-view configuration.
     */
    static void applyToMap(common::IMutableMap3D& output_map,
                           const common::Position3D& scan_origin,
                           const common::Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control_212200943
