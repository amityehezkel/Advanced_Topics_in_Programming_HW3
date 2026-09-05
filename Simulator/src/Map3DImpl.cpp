#include <Simulator/Map3DImpl.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <typeinfo>
#include <utility>

namespace simulator
{
namespace
{

constexpr double kCoordinateTolerance = 1e-9;

[[nodiscard]] double inCentimeters(common::PhysicalLength value)
{
    return value.force_numerical_value_in(common::cm);
}

template<typename Quantity>
[[nodiscard]] double coordinateInCentimeters(Quantity value)
{
    return value.force_numerical_value_in(common::cm);
}

[[nodiscard]] bool validOccupancy(int value)
{
    return value >= static_cast<int>(common::types::VoxelOccupancy::PotentiallyOccupied) &&
           value <= static_cast<int>(common::types::VoxelOccupancy::Occupied);
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), common::types::MapConfig{})
{
}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr,
                     common::types::MapConfig map_config)
    : map_(std::move(map_ptr)), config_(map_config)
{
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
    validate();
}

common::types::VoxelOccupancy Map3DImpl::atVoxel(
    const common::Position3D& pos) const
{
    const auto index = positionToIndex(pos);
    if (!index) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }
    return valueAt(flatIndex(*index));
}

common::types::MapConfig Map3DImpl::getMapConfig() const
{
    return config_;
}

bool Map3DImpl::isInBounds(const common::Position3D& pos) const
{
    return positionToIndex(pos).has_value();
}

void Map3DImpl::set(const common::Position3D& pos,
                    common::types::VoxelOccupancy value)
{
    const auto index = positionToIndex(pos);
    if (!index) {
        return;
    }
    setValue(flatIndex(*index), value);
}

void Map3DImpl::save(const std::filesystem::path& output_path) const
{
    std::error_code error;
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path(), error);
        if (error) {
            throw std::runtime_error("Failed to create output map directory: " +
                                     error.message());
        }
    }
    const char* save_error = map_->SaveNPY(output_path.string(), false);
    if (save_error != nullptr) {
        throw std::runtime_error("Failed to save NPY map '" +
                                 output_path.string() + "': " + save_error);
    }
}

std::optional<Map3DImpl::GridIndex3D> Map3DImpl::positionToIndex(
    const common::Position3D& pos) const
{
    if (map_->Shape().size() != 3 || inCentimeters(config_.resolution) <= 0.0) {
        return std::nullopt;
    }

    const double resolution = inCentimeters(config_.resolution);
    const auto axis_index =
        [resolution](double coordinate,
                     double offset,
                     double minimum,
                     double maximum,
                     std::size_t size) -> std::optional<std::size_t> {
        if (coordinate < minimum - kCoordinateTolerance ||
            coordinate >= maximum - kCoordinateTolerance) {
            return std::nullopt;
        }
        const double raw = (coordinate + offset) / resolution;
        const double rounded = std::round(raw);
        const double stable = std::abs(raw - rounded) < kCoordinateTolerance
                                  ? rounded
                                  : std::floor(raw);
        if (stable < 0.0 ||
            stable > static_cast<double>(
                         std::numeric_limits<std::size_t>::max())) {
            return std::nullopt;
        }
        const auto index = static_cast<std::size_t>(stable);
        if (index >= size) {
            return std::nullopt;
        }
        return index;
    };

    const auto x = axis_index(
        coordinateInCentimeters(pos.x),
        coordinateInCentimeters(config_.offset.x),
        coordinateInCentimeters(config_.boundaries.min_x),
        coordinateInCentimeters(config_.boundaries.max_x), map_->Shape()[0]);
    const auto y = axis_index(
        coordinateInCentimeters(pos.y),
        coordinateInCentimeters(config_.offset.y),
        coordinateInCentimeters(config_.boundaries.min_y),
        coordinateInCentimeters(config_.boundaries.max_y), map_->Shape()[1]);
    const auto z = axis_index(
        coordinateInCentimeters(pos.z),
        coordinateInCentimeters(config_.offset.z),
        coordinateInCentimeters(config_.boundaries.min_height),
        coordinateInCentimeters(config_.boundaries.max_height), map_->Shape()[2]);
    if (!x || !y || !z) {
        return std::nullopt;
    }
    return GridIndex3D{*x, *y, *z};
}

std::size_t Map3DImpl::flatIndex(const GridIndex3D& index) const
{
    const auto& shape = map_->Shape();
    return index.x * shape[1] * shape[2] + index.y * shape[2] + index.z;
}

common::types::VoxelOccupancy Map3DImpl::valueAt(std::size_t index) const
{
    int value = 0;
    if (map_->ValueType() == typeid(int)) {
        value = map_->Data<int>()[index];
    } else if (map_->ValueType() == typeid(std::int8_t)) {
        value = static_cast<int>(map_->Data<std::int8_t>()[index]);
    } else if (map_->ValueType() == typeid(std::uint8_t)) {
        value = static_cast<int>(map_->Data<std::uint8_t>()[index]);
    } else if (map_->ValueType() == typeid(char)) {
        value = static_cast<int>(map_->Data<char>()[index]);
    } else {
        throw std::runtime_error("Map3DImpl contains an unsupported NPY dtype.");
    }
    if (value > 0) {
        return common::types::VoxelOccupancy::Occupied;
    }
    if (!validOccupancy(value)) {
        throw std::runtime_error("Map3DImpl contains an invalid occupancy value.");
    }
    return static_cast<common::types::VoxelOccupancy>(value);
}

void Map3DImpl::setValue(std::size_t index,
                         common::types::VoxelOccupancy value)
{
    const int raw = static_cast<int>(value);
    if (!validOccupancy(raw) ||
        value == common::types::VoxelOccupancy::OutOfBounds) {
        throw std::invalid_argument("Cannot store the requested occupancy value.");
    }
    if (map_->ValueType() == typeid(int)) {
        map_->Data<int>()[index] = raw;
    } else if (map_->ValueType() == typeid(std::int8_t)) {
        map_->Data<std::int8_t>()[index] = static_cast<std::int8_t>(raw);
    } else if (map_->ValueType() == typeid(std::uint8_t) && raw >= 0) {
        map_->Data<std::uint8_t>()[index] = static_cast<std::uint8_t>(raw);
    } else if (map_->ValueType() == typeid(char)) {
        map_->Data<char>()[index] = static_cast<char>(raw);
    } else {
        throw std::runtime_error("The NPY dtype cannot store this occupancy value.");
    }
}

void Map3DImpl::validate() const
{
    if (map_->IsEmpty()) {
        throw std::invalid_argument("Map3DImpl requires allocated NPY data.");
    }
    const auto& shape = map_->Shape();
    if (shape.size() != 3 || shape[0] == 0 || shape[1] == 0 ||
        shape[2] == 0) {
        throw std::invalid_argument(
            "Map3DImpl requires a non-empty [X,Y,Z] array.");
    }
    if (map_->ColMajor()) {
        throw std::invalid_argument("Map3DImpl requires a row-major NPY array.");
    }
    if (map_->ValueType() != typeid(int) &&
        map_->ValueType() != typeid(std::int8_t) &&
        map_->ValueType() != typeid(std::uint8_t) &&
        map_->ValueType() != typeid(char)) {
        throw std::invalid_argument(
            "Map3DImpl supports int, char, int8, or uint8 arrays.");
    }
    if (inCentimeters(config_.resolution) < 0.0) {
        throw std::invalid_argument("Map resolution cannot be negative.");
    }
    if (inCentimeters(config_.resolution) == 0.0) {
        return;
    }
    if (config_.boundaries.min_x > config_.boundaries.max_x ||
        config_.boundaries.min_y > config_.boundaries.max_y ||
        config_.boundaries.min_height > config_.boundaries.max_height) {
        throw std::invalid_argument("Map boundaries must be ordered.");
    }
}

} // namespace simulator
