/**
 * @file Map3DImpl.h
 * @brief Declares the Simulator's TinyNPY-backed mutable occupancy map.
 *
 * The implementation adapts course map interfaces to row-major NPY storage
 * while preserving world-space bounds, offsets, and physical resolution.
 */
#pragma once

#include <Common/IMutableMap3D.h>
#include <TinyNPY.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>

namespace simulator
{

/**
 * @brief Mutable world-coordinate occupancy map backed by a TinyNPY array.
 *
 * Arrays use row-major [X,Y,Z] order. World positions are translated through
 * MapConfig::offset and MapConfig::resolution. Out-of-range reads return
 * VoxelOccupancy::OutOfBounds.
 */
class Map3DImpl final : public common::IMutableMap3D
{
public:
    /**
     * @brief Constructs a map adapter with unspecified geometry.
     * @param map_ptr Shared TinyNPY array supplied by the external library.
     * @throws std::invalid_argument If the array is null or unsupported.
     */
    explicit Map3DImpl(std::shared_ptr<NpyArray> map_ptr);

    /**
     * @brief Constructs a fully configured map adapter.
     * @param map_ptr Shared TinyNPY array supplied by the external library.
     * @param map_config Bounds, offset, and resolution used for coordinate conversion.
     * @throws std::invalid_argument If the array or configuration is invalid.
     */
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr,
              common::types::MapConfig map_config);

    /**
     * @brief Reads the semantic occupancy of the voxel containing a position.
     * @param pos World-space position to query.
     * @return Stored occupancy, or OutOfBounds when the position is outside.
     */
    [[nodiscard]] common::types::VoxelOccupancy atVoxel(
        const common::Position3D& pos) const override;

    /**
     * @brief Returns the map's immutable coordinate configuration.
     * @return Bounds, offset, and resolution by value.
     */
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;

    /**
     * @brief Tests whether a world-space position maps to allocated storage.
     * @param pos Position to test.
     * @return true when the position can be read or written.
     */
    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override;

    /**
     * @brief Stores a semantic occupancy value for an in-bounds position.
     * @param pos World-space position to update.
     * @param value Occupancy state to store.
     */
    void set(const common::Position3D& pos,
             common::types::VoxelOccupancy value) override;

    /**
     * @brief Serializes the complete array as an NPY file.
     * @param output_path Destination path to create or replace.
     * @throws std::runtime_error If serialization fails.
     */
    void save(const std::filesystem::path& output_path) const override;

private:
    /** @brief Zero-based storage coordinates for one voxel. */
    struct GridIndex3D
    {
        std::size_t x = 0; ///< X storage coordinate.
        std::size_t y = 0; ///< Y storage coordinate.
        std::size_t z = 0; ///< Z storage coordinate.
    };

    /**
     * @brief Converts a world position to storage coordinates.
     * @param pos Position to convert.
     * @return Storage index, or std::nullopt when outside the map.
     */
    [[nodiscard]] std::optional<GridIndex3D> positionToIndex(
        const common::Position3D& pos) const;

    /**
     * @brief Converts a valid three-dimensional index to row-major storage.
     * @param index Valid storage index.
     * @return Flat array offset.
     */
    [[nodiscard]] std::size_t flatIndex(const GridIndex3D& index) const;

    /**
     * @brief Decodes one external-array element as a course occupancy value.
     * @param index Flat array offset.
     * @return Semantic occupancy value.
     */
    [[nodiscard]] common::types::VoxelOccupancy valueAt(std::size_t index) const;

    /**
     * @brief Encodes one course occupancy value into external-array storage.
     * @param index Flat array offset.
     * @param value Semantic occupancy value.
     */
    void setValue(std::size_t index, common::types::VoxelOccupancy value);

    /**
     * @brief Validates array shape, value type, and map geometry consistency.
     * @throws std::invalid_argument If the map cannot be used safely.
     */
    void validate() const;

    std::shared_ptr<NpyArray> map_; ///< Shared because TinyNPY returns shared ownership.
    common::types::MapConfig config_; ///< Canonical coordinate conversion data.
};

} // namespace simulator
