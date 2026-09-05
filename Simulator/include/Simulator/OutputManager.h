/**
 * @file OutputManager.h
 * @brief Declares deterministic and collision-free Simulator output naming.
 *
 * Output creation is centralized so every worker receives a unique directory
 * and no invocation removes or overwrites a previous result set.
 */
#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>

namespace simulator
{

/** @brief Creates result directories and safe plugin-derived filenames. */
class OutputManager final
{
public:
    /**
     * @brief Creates a new non-colliding results directory below a parent.
     * @param parent Existing writable plugin folder.
     * @param prefix Required mode prefix such as comparative_results or competition.
     * @return Newly created results directory.
     * @throws std::runtime_error If no directory can be created.
     */
    [[nodiscard]] static std::filesystem::path createResultsDirectory(
        const std::filesystem::path& parent,
        std::string_view prefix);

    /**
     * @brief Creates a unique subdirectory for one pre-indexed simulation run.
     * @param results_directory Root directory returned by createResultsDirectory.
     * @param run_index Stable zero-based result-table index.
     * @param plugin_name Algorithm or MissionControl filename associated with the run.
     * @return Newly created per-run directory.
     * @throws std::runtime_error If the directory cannot be created.
     */
    [[nodiscard]] static std::filesystem::path createRunDirectory(
        const std::filesystem::path& results_directory,
        std::size_t run_index,
        std::string_view plugin_name);

    /**
     * @brief Replaces filesystem-unsafe characters in an external filename.
     * @param value Raw plugin or configuration filename.
     * @return Non-empty portable filename component.
     */
    [[nodiscard]] static std::string sanitizeFilenameComponent(
        std::string_view value);
};

} // namespace simulator
