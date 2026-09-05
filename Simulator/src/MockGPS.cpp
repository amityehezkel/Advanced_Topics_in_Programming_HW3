#include <Simulator/MockGPS.h>

#include <cmath>

namespace simulator
{
namespace
{

[[nodiscard]] double normalizeDegrees(double degrees)
{
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

template<typename Quantity, typename Unit>
[[nodiscard]] Quantity quantized(Quantity value,
                                 common::PhysicalLength resolution,
                                 Unit unit)
{
    const double step = resolution.force_numerical_value_in(common::cm);
    if (step <= 0.0) {
        return value;
    }
    const double raw = value.force_numerical_value_in(common::cm);
    return std::round(raw / step) * step * unit;
}

} // namespace

MockGPS::MockGPS(common::Position3D position,
                 common::Orientation heading,
                 common::PhysicalLength resolution)
    : actual_position_(position),
      heading_(common::Orientation{
          normalizeDegrees(
              heading.horizontal.force_numerical_value_in(common::deg)) *
              common::horizontal_angle[common::deg],
          normalizeDegrees(
              heading.altitude.force_numerical_value_in(common::deg)) *
              common::altitude_angle[common::deg]}),
      resolution_(resolution)
{
}

common::Position3D MockGPS::position() const
{
    return common::Position3D{
        quantized(actual_position_.x, resolution_,
                  common::x_extent[common::cm]),
        quantized(actual_position_.y, resolution_,
                  common::y_extent[common::cm]),
        quantized(actual_position_.z, resolution_,
                  common::z_extent[common::cm]),
    };
}

common::Orientation MockGPS::heading() const
{
    return heading_;
}

common::Position3D MockGPS::actualPosition() const
{
    return actual_position_;
}

void MockGPS::setPosition(common::Position3D position)
{
    actual_position_ = position;
}

void MockGPS::setHeading(common::Orientation heading)
{
    heading_ = common::Orientation{
        normalizeDegrees(
            heading.horizontal.force_numerical_value_in(common::deg)) *
            common::horizontal_angle[common::deg],
        normalizeDegrees(
            heading.altitude.force_numerical_value_in(common::deg)) *
            common::altitude_angle[common::deg],
    };
}

} // namespace simulator
