#include <Simulator/MapsComparison.h>

#include <cmath>
#include <stdexcept>

namespace simulator
{
namespace
{

constexpr double kTolerance = 1e-9;

[[nodiscard]] double centimeters(common::PhysicalLength value)
{
    return value.force_numerical_value_in(common::cm);
}

[[nodiscard]] bool sameResolution(const common::types::MapConfig& lhs,
                                  const common::types::MapConfig& rhs)
{
    return std::abs(centimeters(lhs.resolution) -
                    centimeters(rhs.resolution)) <= kTolerance;
}

} // namespace

std::vector<double> MapsComparison::compare(
    const common::IMap3D& origin,
    const std::vector<const common::IMap3D*>& targets)
{
    std::vector<double> scores;
    scores.reserve(targets.size());
    const common::types::MapConfig original_config = origin.getMapConfig();

    for (const common::IMap3D* target : targets) {
        if (target == nullptr) {
            throw std::invalid_argument("Map comparison target cannot be null.");
        }
        const common::types::MapConfig target_config = target->getMapConfig();
        if (!sameResolution(original_config, target_config)) {
            throw std::invalid_argument(
                "Cannot compare maps with different resolutions.");
        }
        const double resolution = centimeters(target_config.resolution);
        if (resolution <= 0.0) {
            throw std::invalid_argument(
                "Map comparison resolution must be positive.");
        }

        std::size_t compared = 0;
        std::size_t matching = 0;
        for (double x = target_config.boundaries.min_x
                            .force_numerical_value_in(common::cm);
             x < target_config.boundaries.max_x
                         .force_numerical_value_in(common::cm) -
                     kTolerance;
             x += resolution) {
            for (double y = target_config.boundaries.min_y
                                .force_numerical_value_in(common::cm);
                 y < target_config.boundaries.max_y
                             .force_numerical_value_in(common::cm) -
                         kTolerance;
                 y += resolution) {
                for (double z = target_config.boundaries.min_height
                                    .force_numerical_value_in(common::cm);
                     z < target_config.boundaries.max_height
                                 .force_numerical_value_in(common::cm) -
                             kTolerance;
                     z += resolution) {
                    const common::Position3D position{
                        x * common::x_extent[common::cm],
                        y * common::y_extent[common::cm],
                        z * common::z_extent[common::cm],
                    };
                    const auto expected = origin.atVoxel(position);
                    const auto actual = target->atVoxel(position);
                    if (expected ==
                        common::types::VoxelOccupancy::OutOfBounds) {
                        throw std::invalid_argument(
                            "Target comparison bounds exceed the original map.");
                    }
                    if (actual == common::types::VoxelOccupancy::OutOfBounds) {
                        throw std::invalid_argument(
                            "Target comparison bounds exceed the target map.");
                    }
                    ++compared;
                    if (expected == actual) {
                        ++matching;
                    }
                }
            }
        }
        scores.push_back(
            compared == 0
                ? 0.0
                : 100.0 * static_cast<double>(matching) /
                      static_cast<double>(compared));
    }
    return scores;
}

} // namespace simulator
