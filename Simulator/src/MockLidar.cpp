#include <Simulator/MockLidar.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace simulator
{
namespace
{

[[nodiscard]] std::size_t beamsOnCircle(std::size_t circle_index)
{
    std::size_t count = 1;
    for (std::size_t index = 0; index < circle_index; ++index) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] common::HorizontalAngle horizontalDelta(
    common::PhysicalLength offset,
    common::PhysicalLength distance)
{
    return common::HorizontalAngle{common::si::atan2(offset, distance)};
}

[[nodiscard]] common::AltitudeAngle altitudeDelta(
    common::PhysicalLength offset,
    common::PhysicalLength distance)
{
    return common::AltitudeAngle{common::si::atan2(offset, distance)};
}

} // namespace

MockLidar::MockLidar(common::types::LidarConfigData config,
                     const common::IMap3D& map,
                     const common::IGPS& gps)
    : config_(config), map_(map), gps_(gps)
{
}

common::types::LidarConfigData MockLidar::config() const
{
    return config_;
}

common::types::LidarScanResult MockLidar::scan(
    common::Orientation scan_orientation) const
{
    common::types::LidarScanResult results;
    if (config_.fov_circles == 0) {
        return results;
    }

    const common::Orientation sensor_heading = gps_.heading();
    const common::Orientation center_beam_abs{
        scan_orientation.horizontal + sensor_heading.horizontal,
        scan_orientation.altitude + sensor_heading.altitude,
    };

    const common::PhysicalLength center_distance = traceBeam(center_beam_abs);
    results.push_back(
        common::types::LidarHit{center_distance, scan_orientation});

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beamsOnCircle(circle);
        const common::PhysicalLength radius =
            static_cast<double>(circle) * config_.d;

        for (std::size_t index = 0; index < beam_count; ++index) {
            const auto theta =
                (360.0 * static_cast<double>(index) /
                 static_cast<double>(beam_count)) *
                common::deg;
            const common::PhysicalLength horizontal_offset =
                radius * common::si::cos(theta);
            const common::PhysicalLength altitude_offset =
                radius * common::si::sin(theta);

            const common::Orientation offset{
                horizontalDelta(horizontal_offset, config_.z_min),
                altitudeDelta(altitude_offset, config_.z_min),
            };
            const common::Orientation relative_beam{
                scan_orientation.horizontal + offset.horizontal,
                scan_orientation.altitude + offset.altitude,
            };
            const common::Orientation absolute_beam{
                relative_beam.horizontal + sensor_heading.horizontal,
                relative_beam.altitude + sensor_heading.altitude,
            };
            results.push_back(common::types::LidarHit{
                traceBeam(absolute_beam), relative_beam});
        }
    }

    return results;
}

common::PhysicalLength MockLidar::traceBeam(
    const common::Orientation& beam) const
{
    const common::Position3D origin = gps_.position();
    const auto cos_altitude = common::si::cos(beam.altitude);
    const auto dx = cos_altitude * common::si::cos(beam.horizontal);
    const auto dy = cos_altitude * common::si::sin(beam.horizontal);
    const auto dz = common::si::sin(beam.altitude);

    const common::PhysicalLength step =
        0.1 * map_.getMapConfig().resolution;
    const double step_cm = step.force_numerical_value_in(common::cm);
    const double maximum_cm =
        config_.z_max.force_numerical_value_in(common::cm);
    if (step_cm <= 0.0 || maximum_cm < 0.0) {
        return std::numeric_limits<double>::max() * common::cm;
    }

    const auto trace_at =
        [&](common::PhysicalLength distance)
            -> std::optional<common::PhysicalLength> {
        const double distance_cm =
            distance.force_numerical_value_in(common::cm);
        const double dir_x = dx.force_numerical_value_in(common::mp::one);
        const double dir_y = dy.force_numerical_value_in(common::mp::one);
        const double dir_z = dz.force_numerical_value_in(common::mp::one);

        const common::Position3D sample{
            origin.x + dir_x * distance_cm * common::x_extent[common::cm],
            origin.y + dir_y * distance_cm * common::y_extent[common::cm],
            origin.z + dir_z * distance_cm * common::z_extent[common::cm],
        };
        if (map_.atVoxel(sample) ==
            common::types::VoxelOccupancy::Occupied) {
            if (distance < config_.z_min) {
                return 0.0 * common::cm;
            }
            return distance;
        }
        return std::nullopt;
    };

    const double full_steps_value = std::floor(maximum_cm / step_cm);
    if (!std::isfinite(full_steps_value) ||
        full_steps_value >
            static_cast<double>(std::numeric_limits<std::size_t>::max())) {
        throw std::overflow_error("LiDAR ray requires too many samples.");
    }
    const auto full_steps = static_cast<std::size_t>(full_steps_value);
    for (std::size_t index = 0; index <= full_steps; ++index) {
        const common::PhysicalLength distance =
            (static_cast<double>(index) * step_cm) * common::cm;
        if (const auto hit = trace_at(distance)) {
            return *hit;
        }
    }
    const double last_sample_cm = static_cast<double>(full_steps) * step_cm;
    if (last_sample_cm < maximum_cm) {
        if (const auto hit = trace_at(config_.z_max)) {
            return *hit;
        }
    }
    return std::numeric_limits<double>::max() * common::cm;
}

} // namespace simulator
