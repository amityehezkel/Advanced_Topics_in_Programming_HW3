#include <Simulator/MockMovement.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>

namespace simulator
{
namespace
{

constexpr double kMovementEpsilon = 1e-9;

[[nodiscard]] double zeroIfTiny(double value)
{
    return std::abs(value) < kMovementEpsilon ? 0.0 : value;
}

} // namespace

MockMovement::MockMovement(MockGPS& gps) : gps_(gps)
{
}

MockMovement::MockMovement(MockGPS& gps,
                           const common::IMap3D& hidden_map,
                           common::types::MappingBounds mission_bounds,
                           common::PhysicalLength drone_radius)
    : gps_(gps), hidden_map_(&hidden_map), mission_bounds_(mission_bounds),
      drone_radius_(drone_radius)
{
}

common::types::MovementResult MockMovement::rotate(
    common::types::RotationDirection direction,
    common::HorizontalAngle angle)
{
    const common::Orientation current = gps_.heading();
    const common::HorizontalAngle signed_angle =
        direction == common::types::RotationDirection::Right ? angle : -angle;
    gps_.setHeading(common::Orientation{
        current.horizontal + signed_angle, current.altitude});
    return {true, {}};
}

common::types::MovementResult MockMovement::advance(
    common::PhysicalLength distance)
{
    const common::Position3D position = gps_.actualPosition();
    const common::Orientation heading = gps_.heading();
    const double distance_cm =
        distance.force_numerical_value_in(common::cm);
    const double dx = zeroIfTiny(
        common::si::cos(heading.horizontal)
            .force_numerical_value_in(common::mp::one));
    const double dy = zeroIfTiny(
        common::si::sin(heading.horizontal)
            .force_numerical_value_in(common::mp::one));
    return moveTo(common::Position3D{
        position.x + dx * distance_cm * common::x_extent[common::cm],
        position.y + dy * distance_cm * common::y_extent[common::cm],
        position.z,
    });
}

common::types::MovementResult MockMovement::elevate(
    common::PhysicalLength distance)
{
    const common::Position3D position = gps_.actualPosition();
    const double distance_cm =
        distance.force_numerical_value_in(common::cm);
    return moveTo(common::Position3D{
        position.x,
        position.y,
        position.z + distance_cm * common::z_extent[common::cm],
    });
}

common::types::MovementResult MockMovement::moveTo(
    const common::Position3D& destination)
{
    const common::Position3D start = gps_.actualPosition();
    if (hidden_map_ == nullptr) {
        gps_.setPosition(destination);
        return {true, {}};
    }

    const double dx =
        (destination.x - start.x).force_numerical_value_in(common::cm);
    const double dy =
        (destination.y - start.y).force_numerical_value_in(common::cm);
    const double dz =
        (destination.z - start.z).force_numerical_value_in(common::cm);
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double map_step = hidden_map_->getMapConfig()
                                .resolution.force_numerical_value_in(common::cm);
    const double sample_step = std::max(map_step / 2.0, 0.01);
    const std::size_t samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(distance / sample_step)));

    for (std::size_t index = 1; index <= samples; ++index) {
        const double ratio =
            static_cast<double>(index) / static_cast<double>(samples);
        const common::Position3D sample{
            start.x + ratio * dx * common::x_extent[common::cm],
            start.y + ratio * dy * common::y_extent[common::cm],
            start.z + ratio * dz * common::z_extent[common::cm],
        };
        if (!centerIsInsideMission(sample)) {
            return {false, "Movement would leave the mission boundaries."};
        }
        if (!sphereIsClear(sample)) {
            return {false, "Movement would collide with an occupied voxel."};
        }
    }

    gps_.setPosition(destination);
    return {true, {}};
}

bool MockMovement::centerIsInsideMission(
    const common::Position3D& center) const
{
    const double radius =
        drone_radius_.force_numerical_value_in(common::cm);
    return center.x - radius * common::x_extent[common::cm] >=
               mission_bounds_.min_x &&
           center.x + radius * common::x_extent[common::cm] <=
               mission_bounds_.max_x &&
           center.y - radius * common::y_extent[common::cm] >=
               mission_bounds_.min_y &&
           center.y + radius * common::y_extent[common::cm] <=
               mission_bounds_.max_y &&
           center.z - radius * common::z_extent[common::cm] >=
               mission_bounds_.min_height &&
           center.z + radius * common::z_extent[common::cm] <=
               mission_bounds_.max_height;
}

bool MockMovement::sphereIsClear(const common::Position3D& center) const
{
    if (hidden_map_ == nullptr) {
        return true;
    }
    const double radius =
        drone_radius_.force_numerical_value_in(common::cm);
    const double resolution = hidden_map_->getMapConfig()
                                  .resolution.force_numerical_value_in(common::cm);
    const auto center_occupancy = hidden_map_->atVoxel(center);
    if (center_occupancy == common::types::VoxelOccupancy::Occupied ||
        center_occupancy == common::types::VoxelOccupancy::OutOfBounds) {
        return false;
    }
    if (radius <= 0.0) {
        return true;
    }

    const double step = std::max(resolution / 2.0, 0.01);
    for (double z = -radius; z <= radius + kMovementEpsilon; z += step) {
        for (double y = -radius; y <= radius + kMovementEpsilon; y += step) {
            for (double x = -radius; x <= radius + kMovementEpsilon; x += step) {
                if (x * x + y * y + z * z >
                    radius * radius + kMovementEpsilon) {
                    continue;
                }
                const common::Position3D point{
                    center.x + x * common::x_extent[common::cm],
                    center.y + y * common::y_extent[common::cm],
                    center.z + z * common::z_extent[common::cm],
                };
                const auto occupancy = hidden_map_->atVoxel(point);
                if (occupancy == common::types::VoxelOccupancy::Occupied ||
                    occupancy == common::types::VoxelOccupancy::OutOfBounds) {
                    return false;
                }
            }
        }
    }
    return true;
}

} // namespace simulator
