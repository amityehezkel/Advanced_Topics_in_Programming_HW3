/**
 * @file MockMovement.h
 * @brief Declares collision-aware simulated drone movement.
 *
 * The mock validates swept movement against mission bounds and the hidden map,
 * then commits successful pose changes through MockGPS.
 */
#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Simulator/MockGPS.h>

namespace simulator
{

/** @brief Simulated movement driver that updates a MockGPS pose. */
class MockMovement final : public common::IDroneMovement
{
public:
    /**
     * @brief Constructs movement without collision validation for isolated tests.
     * @param gps Mutable simulated pose; must outlive this object.
     */
    explicit MockMovement(MockGPS& gps);

    /**
     * @brief Constructs collision-aware simulation movement.
     * @param gps Mutable simulated pose; must outlive this object.
     * @param hidden_map Ground-truth map; must outlive this object.
     * @param mission_bounds Allowed drone-center mission volume.
     * @param drone_radius Spherical collision radius.
     */
    MockMovement(MockGPS& gps,
                 const common::IMap3D& hidden_map,
                 common::types::MappingBounds mission_bounds,
                 common::PhysicalLength drone_radius);

    /**
     * @brief Rotates the simulated drone.
     * @param direction Left or right rotation direction.
     * @param angle Non-negative requested angle.
     * @return Success or a diagnostic failure.
     */
    common::types::MovementResult rotate(
        common::types::RotationDirection direction,
        common::HorizontalAngle angle) override;

    /**
     * @brief Moves along the current horizontal heading.
     * @param distance Signed travel distance.
     * @return Success or a collision/bounds failure.
     */
    common::types::MovementResult advance(common::PhysicalLength distance) override;

    /**
     * @brief Moves vertically without changing heading.
     * @param distance Signed elevation distance.
     * @return Success or a collision/bounds failure.
     */
    common::types::MovementResult elevate(common::PhysicalLength distance) override;

private:
    /**
     * @brief Validates and commits movement to a destination.
     * @param destination Candidate exact world-space center.
     * @return Success or the first validation failure.
     */
    [[nodiscard]] common::types::MovementResult moveTo(
        const common::Position3D& destination);

    /**
     * @brief Checks the drone's spherical body against the hidden map.
     * @param center Candidate drone center.
     * @return true when every sampled body point is collision-free.
     */
    [[nodiscard]] bool sphereIsClear(const common::Position3D& center) const;

    /**
     * @brief Checks whether a drone center lies inside mission bounds.
     * @param center Candidate drone center.
     * @return true when the position respects all configured boundaries.
     */
    [[nodiscard]] bool centerIsInsideMission(
        const common::Position3D& center) const;

    MockGPS& gps_; ///< Mutable exact pose.
    const common::IMap3D* hidden_map_ = nullptr; ///< Optional collision map.
    common::types::MappingBounds mission_bounds_{}; ///< Allowed center volume.
    common::PhysicalLength drone_radius_{}; ///< Collision radius.
};

} // namespace simulator
