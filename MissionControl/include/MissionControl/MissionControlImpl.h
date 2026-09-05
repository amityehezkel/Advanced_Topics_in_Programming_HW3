/**
 * @file MissionControlImpl.h
 * @brief Declares the submitted single-mission coordinator.
 *
 * MissionControl owns its DroneControl implementation, executes the bounded
 * mission loop, saves the generated map on every terminal path, and optionally
 * emits verbose output. Configuration parsing and worker threads deliberately
 * remain Simulator responsibilities.
 */
#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/IDroneControl.h>
#include <MissionControl/VerboseLogger.h>

#include <filesystem>
#include <memory>

namespace mission_control_212200943
{

/**
 * @brief Runs one mapping mission using injected per-run dependencies.
 *
 * Separate instances share no mutable state and may therefore be invoked by
 * separate Simulator worker threads. Every referenced dependency must outlive
 * this object.
 */
class MissionControlImpl final : public common::IMissionControl
{
public:
    /**
     * @brief Constructs MissionControl and its owned DroneControl adapter.
     * @param dependencies Mission, hardware, map, algorithm, output, and verbosity inputs.
     */
    explicit MissionControlImpl(common::MissionControlDependencies dependencies);

    /**
     * @brief Executes steps until completion, error, or the mission step limit.
     * @return Terminal mission result including executed steps and errors.
     */
    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    /**
     * @brief Saves the output map and appends a save failure to a result.
     * @param result Terminal result produced by the mission loop.
     * @return The original result, or an Error result when saving failed.
     */
    [[nodiscard]] common::types::MissionRunResult saveMap(
        common::types::MissionRunResult result) const;

    common::types::MissionConfigData mission_; ///< Copied mission limits.
    common::IMutableMap3D& output_map_; ///< Per-run map supplied by Simulator.
    std::filesystem::path output_map_file_; ///< Unique map destination.
    VerboseLogger verbose_logger_; ///< Optional unique diagnostic stream.
    std::unique_ptr<mission_control::IDroneControl> drone_control_; ///< Owned controller.
};

} // namespace mission_control_212200943
