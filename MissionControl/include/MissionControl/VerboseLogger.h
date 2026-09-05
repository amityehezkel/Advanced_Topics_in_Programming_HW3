/**
 * @file VerboseLogger.h
 * @brief Declares optional per-mission diagnostic output.
 *
 * The logger creates and appends to a verbose file only when the Simulator
 * supplied the -verbose flag through MissionControlDependencies.
 */
#pragma once

#include <Common/Types.h>

#include <filesystem>
#include <fstream>
#include <string_view>

namespace mission_control_212200943
{

/**
 * @brief Writes human-readable progress for one MissionControl instance.
 *
 * The output filename is derived from the already unique output-map filename,
 * so independent worker threads never share a verbose file.
 */
class VerboseLogger final
{
public:
    /**
     * @brief Configures optional verbose output for one mission.
     * @param output_map_file Unique map path used to derive the log filename.
     * @param enabled Whether a verbose file may be created.
     * @throws std::runtime_error If enabled output cannot be created.
     */
    VerboseLogger(const std::filesystem::path& output_map_file, bool enabled);

    /**
     * @brief Reports whether verbose output is enabled.
     * @return true when logging calls write to disk.
     */
    [[nodiscard]] bool enabled() const noexcept;

    /**
     * @brief Returns the derived verbose-log path.
     * @return Path reserved for this mission's verbose log.
     */
    [[nodiscard]] const std::filesystem::path& path() const noexcept;

    /**
     * @brief Appends the outcome of one drone-control cycle.
     * @param state Drone state observed after the cycle.
     * @param result Continue, completion, or error result from the controller.
     * @throws std::runtime_error If enabled output cannot be appended.
     */
    void logStep(const common::types::DroneState& state,
                 const common::types::DroneStepResult& result) const;

    /**
     * @brief Appends the final mission outcome.
     * @param result Completed, max-steps, or error mission result.
     * @throws std::runtime_error If enabled output cannot be appended.
     */
    void logTerminal(const common::types::MissionRunResult& result) const;

    /**
     * @brief Appends an implementation-defined diagnostic message.
     * @param message Human-readable message without a trailing newline requirement.
     * @throws std::runtime_error If enabled output cannot be appended.
     */
    void logMessage(std::string_view message) const;

private:
    /**
     * @brief Derives a sibling log path from an output-map path.
     * @param output_map_file Unique output-map filename.
     * @return Filename with a verbose-log suffix.
     */
    [[nodiscard]] static std::filesystem::path makeVerbosePath(
        const std::filesystem::path& output_map_file);

    /**
     * @brief Appends one complete line when logging is enabled.
     * @param line Text to append.
     * @throws std::runtime_error If the file cannot be opened or written.
     */
    void appendLine(std::string_view line) const;

    std::filesystem::path log_file_; ///< Unique per-mission verbose path.
    bool enabled_ = false; ///< Prevents all file creation when false.
    mutable std::ofstream stream_; ///< Kept open to avoid per-step reopen overhead.
};

} // namespace mission_control_212200943
