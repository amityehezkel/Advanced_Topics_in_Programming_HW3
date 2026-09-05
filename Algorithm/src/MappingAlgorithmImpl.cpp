#include <Algorithm/MappingAlgorithmImpl.h>

#include <Common/IMap3D.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace algorithm_212200943
{
    using common::altitude_angle;
    using common::horizontal_angle;

    namespace
    {

        constexpr double kTolerance = 1e-9;

        [[nodiscard]] double centimeters(common::PhysicalLength value)
        {
            return value.force_numerical_value_in(common::cm);
        }

        [[nodiscard]] double normalizedDegrees(common::HorizontalAngle angle)
        {
            double value = std::fmod(angle.force_numerical_value_in(common::deg), 360.0);
            if (value < 0.0)
            {
                value += 360.0;
            }
            return value;
        }

        const std::array<common::Orientation, 6> kScanOrientations{{
            {0.0 * horizontal_angle[common::deg], 0.0 * altitude_angle[common::deg]},
            {90.0 * horizontal_angle[common::deg], 0.0 * altitude_angle[common::deg]},
            {180.0 * horizontal_angle[common::deg], 0.0 * altitude_angle[common::deg]},
            {270.0 * horizontal_angle[common::deg], 0.0 * altitude_angle[common::deg]},
            {0.0 * horizontal_angle[common::deg], 90.0 * altitude_angle[common::deg]},
            {0.0 * horizontal_angle[common::deg], -90.0 * altitude_angle[common::deg]},
        }};

    } // namespace

    common::types::MappingStepCommand MappingAlgorithmImpl::nextStep(const common::types::DroneState &state,
                                                                     const common::types::LidarScanResult *latest_scan)
    {
        (void)latest_scan;

        const auto current = positionToIndex(state.position);
        if (!current)
        {
            throw std::runtime_error("The drone position is outside the output map.");
        }

        if (!initialized_)
        {
            const auto config = output_map_.getMapConfig();
            const double resolution = centimeters(config.resolution);
            if (resolution <= 0.0)
            {
                throw std::runtime_error("The mapping resolution must be positive.");
            }
            const auto axisSize = [resolution](double minimum, double maximum)
            {
                const double cells =
                    std::floor((maximum - minimum) / resolution + kTolerance);
                if (cells <= 0.0 ||
                    cells > static_cast<double>(std::numeric_limits<std::size_t>::max()))
                {
                    throw std::runtime_error("The output map dimensions are invalid.");
                }
                return static_cast<std::size_t>(cells);
            };
            const std::size_t x_size = axisSize(
                config.boundaries.min_x.force_numerical_value_in(common::cm),
                config.boundaries.max_x.force_numerical_value_in(common::cm));
            const std::size_t y_size = axisSize(
                config.boundaries.min_y.force_numerical_value_in(common::cm),
                config.boundaries.max_y.force_numerical_value_in(common::cm));
            const std::size_t z_size = axisSize(
                config.boundaries.min_height.force_numerical_value_in(common::cm),
                config.boundaries.max_height.force_numerical_value_in(common::cm));
            if (x_size > std::numeric_limits<std::size_t>::max() / y_size ||
                x_size * y_size > std::numeric_limits<std::size_t>::max() / z_size)
            {
                throw std::runtime_error("The output map is too large.");
            }
            visited_.assign(x_size * y_size * z_size, false);
            initialized_ = true;
        }

        if (!movement_commands_.empty())
        {
            common::types::MovementCommand command = movement_commands_.front();
            movement_commands_.pop_front();
            return common::types::MappingStepCommand{command, std::nullopt, common::types::AlgorithmStatus::Working};
        }

        if (next_scan_orientation_ < kScanOrientations.size())
        {
            const common::Orientation scan = kScanOrientations[next_scan_orientation_++];
            return common::types::MappingStepCommand{
                std::nullopt, scan, common::types::AlgorithmStatus::Working};
        }

        visited_[flatIndex(*current)] = true;
        const auto path = pathToNearestUnvisited(*current);
        if (!path)
        {
            return common::types::MappingStepCommand{
                std::nullopt,
                std::nullopt,
                hasUnmappedVoxels()
                    ? common::types::AlgorithmStatus::FinishedWithUnmappableVoxels
                    : common::types::AlgorithmStatus::Finished,
            };
        }

        enqueuePath(*current, *path, state.heading.horizontal);
        next_scan_orientation_ = 0;
        if (movement_commands_.empty())
        {
            throw std::runtime_error("Mapping path did not produce a movement command.");
        }
        common::types::MovementCommand command = movement_commands_.front();
        movement_commands_.pop_front();
        return common::types::MappingStepCommand{command, std::nullopt, common::types::AlgorithmStatus::Working};
    }

    std::optional<MappingAlgorithmImpl::GridIndex3D>
    MappingAlgorithmImpl::positionToIndex(const common::Position3D &position) const
    {
        const auto config = output_map_.getMapConfig();
        const double resolution = centimeters(config.resolution);
        if (resolution <= 0.0 || !output_map_.isInBounds(position))
        {
            return std::nullopt;
        }
        const auto convert = [resolution](double coordinate, double offset)
        {
            const double raw = (coordinate + offset) / resolution;
            const double rounded = std::round(raw);
            return static_cast<int>(
                std::abs(raw - rounded) < kTolerance ? rounded : std::floor(raw));
        };
        const GridIndex3D index{
            convert(position.x.force_numerical_value_in(common::cm),
                    config.offset.x.force_numerical_value_in(common::cm)),
            convert(position.y.force_numerical_value_in(common::cm),
                    config.offset.y.force_numerical_value_in(common::cm)),
            convert(position.z.force_numerical_value_in(common::cm),
                    config.offset.z.force_numerical_value_in(common::cm)),
        };
        return isValidIndex(index) ? std::optional<GridIndex3D>{index} : std::nullopt;
    }

    common::Position3D MappingAlgorithmImpl::indexToPosition(const GridIndex3D &index) const
    {
        const auto config = output_map_.getMapConfig();
        const double resolution = centimeters(config.resolution);
        return common::Position3D{
            -config.offset.x + static_cast<double>(index.x) * resolution * common::x_extent[common::cm],
            -config.offset.y + static_cast<double>(index.y) * resolution * common::y_extent[common::cm],
            -config.offset.z + static_cast<double>(index.z) * resolution * common::z_extent[common::cm],
        };
    }

    bool MappingAlgorithmImpl::isValidIndex(const GridIndex3D &index) const
    {
        if (index.x < 0 || index.y < 0 || index.z < 0)
        {
            return false;
        }
        return output_map_.isInBounds(indexToPosition(index));
    }

    std::size_t MappingAlgorithmImpl::flatIndex(const GridIndex3D &index) const
    {
        const auto config = output_map_.getMapConfig();
        const double resolution = centimeters(config.resolution);
        const auto axisSize = [resolution](double minimum, double maximum)
        {
            return static_cast<std::size_t>(
                std::floor((maximum - minimum) / resolution + kTolerance));
        };
        const std::size_t y_size = axisSize(
            config.boundaries.min_y.force_numerical_value_in(common::cm),
            config.boundaries.max_y.force_numerical_value_in(common::cm));
        const std::size_t z_size = axisSize(
            config.boundaries.min_height.force_numerical_value_in(common::cm),
            config.boundaries.max_height.force_numerical_value_in(common::cm));
        return static_cast<std::size_t>(index.x) * y_size * z_size +
               static_cast<std::size_t>(index.y) * z_size +
               static_cast<std::size_t>(index.z);
    }

    bool MappingAlgorithmImpl::hasPassageClearance(const GridIndex3D &index) const
    {
        if (!isValidIndex(index))
        {
            return false;
        }
        const common::types::MapConfig config = output_map_.getMapConfig();
        const double resolution = centimeters(config.resolution);
        const double radius = centimeters(drone_config_.radius);
        const common::Position3D center = indexToPosition(index);
        if (center.x - radius * common::x_extent[common::cm] < config.boundaries.min_x ||
            center.x + radius * common::x_extent[common::cm] > config.boundaries.max_x ||
            center.y - radius * common::y_extent[common::cm] < config.boundaries.min_y ||
            center.y + radius * common::y_extent[common::cm] > config.boundaries.max_y ||
            center.z - radius * common::z_extent[common::cm] < config.boundaries.min_height ||
            center.z + radius * common::z_extent[common::cm] > config.boundaries.max_height)
        {
            return false;
        }
        const int cells = static_cast<int>(std::ceil(radius / resolution));
        const double radius_squared = radius * radius;

        for (int dz = -cells; dz <= cells; ++dz)
        {
            for (int dy = -cells; dy <= cells; ++dy)
            {
                for (int dx = -cells; dx <= cells; ++dx)
                {
                    const double x_distance =
                        std::max(std::abs(static_cast<double>(dx) * resolution) -
                                     resolution / 2.0,
                                 0.0);
                    const double y_distance =
                        std::max(std::abs(static_cast<double>(dy) * resolution) -
                                     resolution / 2.0,
                                 0.0);
                    const double z_distance =
                        std::max(std::abs(static_cast<double>(dz) * resolution) -
                                     resolution / 2.0,
                                 0.0);
                    if (x_distance * x_distance + y_distance * y_distance +
                            z_distance * z_distance >
                        radius_squared + kTolerance)
                    {
                        continue;
                    }
                    const GridIndex3D nearby{index.x + dx, index.y + dy, index.z + dz};
                    if (!isValidIndex(nearby) ||
                        output_map_.atVoxel(indexToPosition(nearby)) !=
                            common::types::VoxelOccupancy::Empty)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    std::optional<std::vector<MappingAlgorithmImpl::GridIndex3D>>
    MappingAlgorithmImpl::pathToNearestUnvisited(const GridIndex3D &start) const
    {
        std::vector<bool> discovered(visited_.size(), false);
        std::vector<std::optional<GridIndex3D>> parents(visited_.size(), std::nullopt);
        std::queue<GridIndex3D> queue;
        discovered[flatIndex(start)] = true;
        queue.push(start);

        while (!queue.empty())
        {
            const GridIndex3D current = queue.front();
            queue.pop();
            const std::size_t current_flat = flatIndex(current);
            if (!(current == start) && !visited_[current_flat] &&
                hasPassageClearance(current))
            {
                std::vector<GridIndex3D> reversed;
                GridIndex3D cursor = current;
                while (!(cursor == start))
                {
                    reversed.push_back(cursor);
                    const auto parent = parents[flatIndex(cursor)];
                    if (!parent)
                    {
                        throw std::runtime_error("Broken BFS parent chain.");
                    }
                    cursor = *parent;
                }
                std::reverse(reversed.begin(), reversed.end());
                return reversed;
            }
            for (const GridIndex3D &neighbor : neighbors(current))
            {
                if (!isValidIndex(neighbor) || !hasPassageClearance(neighbor))
                {
                    continue;
                }
                const std::size_t neighbor_flat = flatIndex(neighbor);
                if (!discovered[neighbor_flat])
                {
                    discovered[neighbor_flat] = true;
                    parents[neighbor_flat] = current;
                    queue.push(neighbor);
                }
            }
        }
        return std::nullopt;
    }

    bool MappingAlgorithmImpl::hasUnmappedVoxels() const
    {
        const auto config = output_map_.getMapConfig();
        const double resolution = centimeters(config.resolution);
        for (double x = config.boundaries.min_x.force_numerical_value_in(common::cm);
             x < config.boundaries.max_x.force_numerical_value_in(common::cm) - kTolerance;
             x += resolution)
        {
            for (double y = config.boundaries.min_y.force_numerical_value_in(common::cm);
                 y < config.boundaries.max_y.force_numerical_value_in(common::cm) - kTolerance;
                 y += resolution)
            {
                for (double z =
                         config.boundaries.min_height.force_numerical_value_in(common::cm);
                     z < config.boundaries.max_height.force_numerical_value_in(common::cm) -
                             kTolerance;
                     z += resolution)
                {
                    if (output_map_.atVoxel(common::Position3D{
                            x * common::x_extent[common::cm],
                            y * common::y_extent[common::cm],
                            z * common::z_extent[common::cm]}) == common::types::VoxelOccupancy::Unmapped)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void MappingAlgorithmImpl::enqueuePath(const GridIndex3D &start,
                                           const std::vector<GridIndex3D> &path,
                                           common::HorizontalAngle current_heading)
    {
        GridIndex3D previous = start;
        common::HorizontalAngle predicted_heading = current_heading;
        for (const GridIndex3D &destination : path)
        {
            const int dx = destination.x - previous.x;
            const int dy = destination.y - previous.y;
            const int dz = destination.z - previous.z;
            const double resolution = centimeters(output_map_.getMapConfig().resolution);
            if (dx == 1 && dy == 0 && dz == 0)
            {
                enqueueRotation(predicted_heading, 0.0 * horizontal_angle[common::deg]);
                enqueueAdvance(resolution * common::cm);
            }
            else if (dx == -1 && dy == 0 && dz == 0)
            {
                enqueueRotation(predicted_heading, 180.0 * horizontal_angle[common::deg]);
                enqueueAdvance(resolution * common::cm);
            }
            else if (dx == 0 && dy == 1 && dz == 0)
            {
                enqueueRotation(predicted_heading, 90.0 * horizontal_angle[common::deg]);
                enqueueAdvance(resolution * common::cm);
            }
            else if (dx == 0 && dy == -1 && dz == 0)
            {
                enqueueRotation(predicted_heading, 270.0 * horizontal_angle[common::deg]);
                enqueueAdvance(resolution * common::cm);
            }
            else if (dx == 0 && dy == 0 && dz == 1)
            {
                enqueueElevation(resolution * common::cm);
            }
            else if (dx == 0 && dy == 0 && dz == -1)
            {
                enqueueElevation(-resolution * common::cm);
            }
            else
            {
                throw std::runtime_error("Mapping path contains non-adjacent cells.");
            }
            previous = destination;
        }
    }

    void MappingAlgorithmImpl::enqueueRotation(common::HorizontalAngle &current_heading,
                                               common::HorizontalAngle target)
    {
        const double current = normalizedDegrees(current_heading);
        const double desired = normalizedDegrees(target);
        const double right = std::fmod(desired - current + 360.0, 360.0);
        const common::types::RotationDirection direction =
            right <= 180.0 ? common::types::RotationDirection::Right : common::types::RotationDirection::Left;
        double remaining = right <= 180.0 ? right : 360.0 - right;
        const double maximum = drone_config_.max_rotate.force_numerical_value_in(common::deg);
        if (maximum <= 0.0 && remaining > kTolerance)
        {
            throw std::runtime_error("Drone rotation limit must be positive.");
        }
        while (remaining > kTolerance)
        {
            const double amount = std::min(remaining, maximum);
            movement_commands_.push_back(common::types::MovementCommand{
                common::types::MovementCommandType::Rotate,
                direction,
                amount * horizontal_angle[common::deg],
                {},
            });
            remaining -= amount;
        }
        current_heading = target;
    }

    void MappingAlgorithmImpl::enqueueAdvance(common::PhysicalLength distance)
    {
        double remaining = std::abs(centimeters(distance));
        const double maximum = centimeters(drone_config_.max_advance);
        if (maximum <= 0.0)
        {
            throw std::runtime_error("Drone advance limit must be positive.");
        }
        while (remaining > kTolerance)
        {
            const double amount = std::min(remaining, maximum);
            movement_commands_.push_back(common::types::MovementCommand{
                common::types::MovementCommandType::Advance,
                common::types::RotationDirection::Left,
                {},
                amount * common::cm,
            });
            remaining -= amount;
        }
    }

    void MappingAlgorithmImpl::enqueueElevation(common::PhysicalLength distance)
    {
        const double signed_distance = centimeters(distance);
        const double sign = signed_distance < 0.0 ? -1.0 : 1.0;
        double remaining = std::abs(signed_distance);
        const double maximum = centimeters(drone_config_.max_elevate);
        if (maximum <= 0.0)
        {
            throw std::runtime_error("Drone elevation limit must be positive.");
        }
        while (remaining > kTolerance)
        {
            const double amount = std::min(remaining, maximum);
            movement_commands_.push_back(common::types::MovementCommand{
                common::types::MovementCommandType::Elevate,
                common::types::RotationDirection::Left,
                {},
                sign * amount * common::cm,
            });
            remaining -= amount;
        }
    }

    std::array<MappingAlgorithmImpl::GridIndex3D, 6>
    MappingAlgorithmImpl::neighbors(const GridIndex3D &index)
    {
        return {{
            {index.x + 1, index.y, index.z},
            {index.x - 1, index.y, index.z},
            {index.x, index.y + 1, index.z},
            {index.x, index.y - 1, index.z},
            {index.x, index.y, index.z + 1},
            {index.x, index.y, index.z - 1},
        }};
    }

    REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl);

} // namespace algorithm_212200943
