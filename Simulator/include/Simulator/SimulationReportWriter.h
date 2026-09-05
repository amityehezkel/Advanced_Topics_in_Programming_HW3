/**
 * @file SimulationReportWriter.h
 * @brief Declares detailed, comparative, and competition YAML reporting.
 *
 * Detailed output preserves the Assignment 2 run information. Aggregate output
 * groups equal MissionControl results or ranks Algorithm results according to
 * the Assignment 3 mode-specific rules.
 */
#pragma once

#include <Simulator/ConfigurationLoader.h>
#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace simulator
{

/** @brief Aggregate totals for one successfully executed plugin. */
struct PluginResultSummary
{
    std::string plugin_file; ///< Plugin filename shown in the aggregate report.
    double total_score = 0.0; ///< Sum of all requested run scores, including -1.
    std::size_t total_steps = 0; ///< Sum of actually executed mission steps.
};

/** @brief One comparative group whose MissionControls produced equal totals. */
struct ComparativeResultGroup
{
    std::vector<std::string> mission_controls; ///< Plugins sharing score and step totals.
    double total_score = 0.0; ///< Exact common total score.
    std::size_t total_steps = 0; ///< Exact common total step count.
};

/** @brief Complete data required by the comparative aggregate YAML report. */
struct ComparativeReportData
{
    std::filesystem::path composition_file; ///< Input composition path.
    std::filesystem::path mission_control_folder; ///< Folder whose plugins were compared.
    std::string generated_at_utc; ///< ISO-8601 report timestamp.
    std::vector<PluginResultSummary> successful_results; ///< Successfully run plugin totals.
    std::vector<std::string> errors; ///< Plugins that could not be loaded or run.
};

/** @brief Complete data required by the competition aggregate YAML report. */
struct CompetitionReportData
{
    std::filesystem::path composition_file; ///< Input composition path.
    std::filesystem::path mission_control_file; ///< Fixed MissionControl plugin.
    std::string generated_at_utc; ///< ISO-8601 report timestamp.
    std::vector<PluginResultSummary> successful_results; ///< Algorithm totals to rank.
    std::vector<std::string> errors; ///< Algorithms that could not be loaded or run.
};

/** @brief Serializes all required Simulator YAML report formats. */
class SimulationReportWriter final
{
public:
    /**
     * @brief Writes one Assignment 2-style detailed report for a plugin.
     * @param report Executed run results in deterministic order.
     * @param valid_run_config_paths Source paths aligned with report.runs.
     * @param rejected_runs Invalid requested combinations receiving score -1.
     * @param composition_file Top-level composition path.
     * @param output_file YAML destination.
     * @throws std::runtime_error If the output cannot be serialized.
     */
    static void writeDetailedReport(
        const types::SimulationManagerReport& report,
        const std::vector<RunConfigurationPaths>& valid_run_config_paths,
        const std::vector<RejectedSimulationRun>& rejected_runs,
        const std::filesystem::path& composition_file,
        const std::filesystem::path& output_file);

    /**
     * @brief Writes the comparative report grouped by exact score and step totals.
     * @param report Raw successful plugin totals and plugin errors.
     * @param output_file Comparative YAML destination.
     * @throws std::runtime_error If the output cannot be serialized.
     */
    static void writeComparativeReport(
        const ComparativeReportData& report,
        const std::filesystem::path& output_file);

    /**
     * @brief Writes the competition report sorted by score then steps.
     * @param report Raw successful Algorithm totals and plugin errors.
     * @param output_file Competition YAML destination.
     * @throws std::runtime_error If the output cannot be serialized.
     */
    static void writeCompetitionReport(
        const CompetitionReportData& report,
        const std::filesystem::path& output_file);

    /**
     * @brief Groups comparative results by exact total score and total steps.
     * @param results Successful MissionControl totals.
     * @return Groups sorted by number of agreeing plugins descending.
     */
    [[nodiscard]] static std::vector<ComparativeResultGroup> groupComparativeResults(
        const std::vector<PluginResultSummary>& results);

    /**
     * @brief Sorts competition results by score descending then steps ascending.
     * @param results Successful Algorithm totals.
     * @return Sorted copy suitable for YAML output.
     */
    [[nodiscard]] static std::vector<PluginResultSummary> rankCompetitionResults(
        std::vector<PluginResultSummary> results);
};

} // namespace simulator
