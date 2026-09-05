/**
 * @file MockLidar.h
 * @brief Declares the hidden-map-backed LiDAR simulation.
 *
 * The mock ray-marches through the real simulation map while returning only
 * measurements allowed by the course ILidar interface.
 */
#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>

namespace simulator
{

/** @brief Simulated LiDAR using sub-voxel ray marching. */
class MockLidar final : public common::ILidar
{
public:
    /**
     * @brief Binds immutable sensor configuration, hidden map, and GPS pose.
     * @param config LiDAR range and field-of-view configuration.
     * @param map Hidden ground-truth occupancy map; must outlive this object.
     * @param gps Drone pose source; must outlive this object.
     */
    MockLidar(common::types::LidarConfigData config,
              const common::IMap3D& map,
              const common::IGPS& gps);

    /**
     * @brief Simulates the requested scan and every configured FOV beam.
     * @param scan_orientation Orientation relative to the drone heading.
     * @return Beam distances and relative angles.
     */
    [[nodiscard]] common::types::LidarScanResult scan(
        common::Orientation scan_orientation) const override;

    /**
     * @brief Returns the immutable sensor configuration.
     * @return Copy of the LiDAR configuration.
     */
    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    /**
     * @brief Ray-marches one absolute beam through the hidden map.
     * @param beam Absolute world-space beam orientation.
     * @return Measured distance using course miss and near-hit conventions.
     */
    [[nodiscard]] common::PhysicalLength traceBeam(
        const common::Orientation& beam) const;

    common::types::LidarConfigData config_; ///< Immutable sensor characteristics.
    const common::IMap3D& map_; ///< Hidden ground-truth map.
    const common::IGPS& gps_; ///< Current drone pose.
};

} // namespace simulator
