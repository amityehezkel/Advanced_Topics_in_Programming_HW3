/**
 * @file MappingAlgorithmImpl.h
 * @brief Declares the submitted mapping algorithm implementation.
 *
 * This header owns the persistent scan-and-path-planning state used by the
 * Algorithm shared library. It depends only on the immutable course Common
 * API so that the library can be loaded with any compatible Simulator and
 * MissionControl implementation.
 */
#pragma once

#include <Common/IMappingAlgorithm.h>

#include <array>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

namespace algorithm_212200943
{

/**
 * @brief Deterministic scan-and-BFS mapping strategy.
 *
 * The algorithm observes only the generated output map. It scans the six
 * axial orientations at each reached cell, searches known-free space for the
 * nearest unvisited destination, and emits one drone-limit-compliant command
 * per call.
 */
class MappingAlgorithmImpl final : public common::IMappingAlgorithm
{
public:
    /**
     * @brief Inherits the constructor accepting MappingAlgorithmDependencies.
     *
     * The inherited constructor copies the typed configurations and retains a
     * non-owning reference to the output map for the algorithm's lifetime.
     */
    using common::IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Advances the persistent mapping plan by one command.
     * @param state Current position, heading, and step index reported by the drone.
     * @param latest_scan Most recent scan, or nullptr before the first scan.
     * @return The movement, scan request, and continuing or terminal status.
     */
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    /** @brief Integer coordinates of one voxel in the output-map grid. */
    struct GridIndex3D
    {
        int x = 0; ///< Zero-based X coordinate.
        int y = 0; ///< Zero-based Y coordinate.
        int z = 0; ///< Zero-based Z coordinate.

        /**
         * @brief Compares all three grid coordinates.
         * @param other Grid index to compare with this index.
         * @return true when all coordinates are equal.
         */
        [[nodiscard]] bool operator==(const GridIndex3D& other) const = default;
    };

    /**
     * @brief Converts a world-space position to an output-map grid index.
     * @param position Position expressed in course physical units.
     * @return The corresponding index, or std::nullopt when outside the map.
     */
    [[nodiscard]] std::optional<GridIndex3D> positionToIndex(
        const common::Position3D& position) const;

    /**
     * @brief Converts a grid index to its representative world-space position.
     * @param index Valid output-map grid index.
     * @return World-space position represented by the index.
     */
    [[nodiscard]] common::Position3D indexToPosition(const GridIndex3D& index) const;

    /**
     * @brief Tests whether an index lies inside the configured map dimensions.
     * @param index Index to test.
     * @return true when the index can be safely accessed.
     */
    [[nodiscard]] bool isValidIndex(const GridIndex3D& index) const;

    /**
     * @brief Converts a valid three-dimensional index to row-major storage.
     * @param index Valid output-map grid index.
     * @return Flat visitation-array index.
     */
    [[nodiscard]] std::size_t flatIndex(const GridIndex3D& index) const;

    /**
     * @brief Checks whether the drone body can occupy the indexed voxel.
     * @param index Candidate center voxel.
     * @return true when the known map provides the required clearance.
     */
    [[nodiscard]] bool hasPassageClearance(const GridIndex3D& index) const;

    /**
     * @brief Finds the shortest known-free path to an unvisited voxel.
     * @param start Current grid index.
     * @return Reconstructed path, or std::nullopt when no destination is reachable.
     */
    [[nodiscard]] std::optional<std::vector<GridIndex3D>> pathToNearestUnvisited(
        const GridIndex3D& start) const;

    /**
     * @brief Determines whether the mission bounds still contain unknown voxels.
     * @return true when at least one relevant voxel remains unmapped.
     */
    [[nodiscard]] bool hasUnmappedVoxels() const;

    /**
     * @brief Converts a reconstructed grid path into bounded movement commands.
     * @param start Current grid index.
     * @param path Ordered path from the current voxel to the destination.
     * @param current_heading Current horizontal heading used while planning turns.
     */
    void enqueuePath(const GridIndex3D& start,
                     const std::vector<GridIndex3D>& path,
                     common::HorizontalAngle current_heading);

    /**
     * @brief Enqueues the shortest allowed rotations toward a target heading.
     * @param current_heading Heading updated as rotations are enqueued.
     * @param target Desired final horizontal heading.
     */
    void enqueueRotation(common::HorizontalAngle& current_heading,
                         common::HorizontalAngle target);

    /**
     * @brief Splits and enqueues a horizontal translation within drone limits.
     * @param distance Signed forward distance to enqueue.
     */
    void enqueueAdvance(common::PhysicalLength distance);

    /**
     * @brief Splits and enqueues a vertical translation within drone limits.
     * @param distance Signed elevation distance to enqueue.
     */
    void enqueueElevation(common::PhysicalLength distance);

    /**
     * @brief Returns the six axis-adjacent indices around a voxel.
     * @param index Center grid index.
     * @return Neighbors in a deterministic order.
     */
    [[nodiscard]] static std::array<GridIndex3D, 6> neighbors(
        const GridIndex3D& index);

    std::vector<bool> visited_; ///< One visitation bit per output voxel.
    std::deque<common::types::MovementCommand> movement_commands_; ///< Pending path commands.
    std::size_t next_scan_orientation_ = 0; ///< Next entry in the six-orientation scan cycle.
    bool initialized_ = false; ///< Defers grid allocation until the first state is available.
};

} // namespace algorithm_212200943
