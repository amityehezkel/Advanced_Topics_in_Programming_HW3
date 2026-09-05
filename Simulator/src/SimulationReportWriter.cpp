#include <Simulator/SimulationReportWriter.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>

namespace simulator
{
namespace
{

[[nodiscard]] double centimeters(common::PhysicalLength value)
{
    return value.force_numerical_value_in(common::cm);
}

[[nodiscard]] std::string_view missionStatus(
    common::types::MissionRunStatus status)
{
    switch (status) {
    case common::types::MissionRunStatus::Completed: return "completed";
    case common::types::MissionRunStatus::MaxSteps: return "max_steps";
    case common::types::MissionRunStatus::Error: return "error";
    }
    return "error";
}

[[nodiscard]] std::string_view resolutionStatus(
    types::ResolutionRequestStatus status)
{
    switch (status) {
    case types::ResolutionRequestStatus::Accepted: return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored: return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IGNORED TOO SMALL";
    }
    return "IGNORED";
}

[[nodiscard]] YAML::Node errorsNode(
    const std::vector<common::types::ErrorRef>& errors)
{
    YAML::Node node{YAML::NodeType::Sequence};
    for (const auto& error : errors) {
        YAML::Node item;
        item["code"] = error.code;
        item["message"] = error.message;
        node.push_back(item);
    }
    return node;
}

void writeYaml(const YAML::Node& root, const std::filesystem::path& output_file)
{
    std::error_code error;
    if (!output_file.parent_path().empty()) {
        std::filesystem::create_directories(output_file.parent_path(), error);
        if (error) {
            throw std::runtime_error("Cannot create report directory: " +
                                     error.message());
        }
    }
    std::ofstream output{output_file, std::ios::trunc};
    if (!output) {
        throw std::runtime_error("Cannot open report '" + output_file.string() +
                                 "'.");
    }
    output << root;
    if (!output) {
        throw std::runtime_error("Cannot write report '" + output_file.string() +
                                 "'.");
    }
}

struct HierarchicalMission
{
    std::filesystem::path file;
    bool has_resolution = false;
    double resolution = 0.0;
    std::string resolution_status;
    YAML::Node runs{YAML::NodeType::Sequence};
};

struct HierarchicalSimulation
{
    std::filesystem::path file;
    std::vector<HierarchicalMission> missions;
    std::map<std::string, std::size_t> mission_indices;
};

void appendRun(std::vector<HierarchicalSimulation>& simulations,
               std::map<std::string, std::size_t>& simulation_indices,
               const RunConfigurationPaths& paths,
               const YAML::Node& run,
               const types::SimulationResult* result)
{
    const std::string simulation_key = paths.simulation_config_file.string();
    const auto [simulation_position, inserted_simulation] =
        simulation_indices.emplace(simulation_key, simulations.size());
    if (inserted_simulation) {
        simulations.push_back({paths.simulation_config_file, {}, {}});
    }
    auto& simulation = simulations[simulation_position->second];

    const std::string mission_key = paths.mission_config_file.string();
    const auto [mission_position, inserted_mission] =
        simulation.mission_indices.emplace(mission_key,
                                           simulation.missions.size());
    if (inserted_mission) {
        simulation.missions.push_back(
            {paths.mission_config_file, false, 0.0, {},
             YAML::Node{YAML::NodeType::Sequence}});
    }
    auto& mission = simulation.missions[mission_position->second];
    if (result != nullptr) {
        mission.has_resolution = true;
        mission.resolution = centimeters(result->output_map_config.resolution);
        mission.resolution_status =
            std::string{resolutionStatus(result->resolution_request_status)};
    }
    mission.runs.push_back(YAML::Clone(run));
}

[[nodiscard]] YAML::Node executedRun(std::size_t index,
                                     const RunConfigurationPaths& paths,
                                     const types::SimulationResult& result)
{
    YAML::Node node;
    node["run_index"] = index;
    node["drone_config"] = paths.drone_config_file.string();
    node["lidar_config"] = paths.lidar_config_file.string();
    node["output_map"] = result.output_map_file.string();
    node["score"] = result.mission_score;
    if (!result.mission_results.empty()) {
        const auto& mission = result.mission_results.front();
        node["status"] = std::string{missionStatus(mission.status)};
        node["steps"] = mission.steps;
        if (!mission.errors.empty()) {
            node["errors"] = errorsNode(mission.errors);
        }
    }
    return node;
}

[[nodiscard]] YAML::Node rejectedRun(std::size_t index,
                                     const RejectedSimulationRun& rejected)
{
    YAML::Node node;
    node["run_index"] = index;
    node["drone_config"] = rejected.drone_config_file.string();
    node["lidar_config"] = rejected.lidar_config_file.string();
    node["status"] = "error";
    node["steps"] = 0;
    node["score"] = rejected.score;
    node["errors"] = errorsNode(rejected.errors);
    return node;
}

[[nodiscard]] YAML::Node flatRun(const YAML::Node& run,
                                 const RunConfigurationPaths& paths,
                                 const types::SimulationResult* result)
{
    YAML::Node node = YAML::Clone(run);
    node["simulation_config"] = paths.simulation_config_file.string();
    node["mission_config"] = paths.mission_config_file.string();
    if (result != nullptr) {
        node["resolution_cm"] =
            centimeters(result->output_map_config.resolution);
        node["resolution_request_status"] =
            std::string{resolutionStatus(result->resolution_request_status)};
    }
    return node;
}

} // namespace

void SimulationReportWriter::writeDetailedReport(
    const types::SimulationManagerReport& report,
    const std::vector<RunConfigurationPaths>& valid_run_config_paths,
    const std::vector<RejectedSimulationRun>& rejected_runs,
    const std::filesystem::path& composition_file,
    const std::filesystem::path& output_file)
{
    if (valid_run_config_paths.size() != report.runs.size()) {
        throw std::invalid_argument(
            "Detailed report requires one path descriptor per valid run.");
    }

    YAML::Node score_report;
    score_report["composition_file"] = composition_file.string();
    score_report["generated_at_utc"] = report.generated_at_utc;
    score_report["metric"] = report.metric;
    score_report["score_range"]["min"] = std::get<0>(report.score_range);
    score_report["score_range"]["max"] = std::get<1>(report.score_range);
    score_report["error_score"] = report.error_score;

    std::size_t scored = 0;
    double total = 0.0;
    double minimum = std::numeric_limits<double>::max();
    double maximum = std::numeric_limits<double>::lowest();
    for (const auto& run : report.runs) {
        if (run.mission_score >= 0.0) {
            ++scored;
            total += run.mission_score;
            minimum = std::min(minimum, run.mission_score);
            maximum = std::max(maximum, run.mission_score);
        }
    }
    const std::size_t requested = report.runs.size() + rejected_runs.size();
    score_report["summary"]["total_runs"] = requested;
    score_report["summary"]["scored_runs"] = scored;
    score_report["summary"]["error_runs"] = requested - scored;
    score_report["summary"]["average_score"] =
        scored == 0 ? 0.0 : total / static_cast<double>(scored);
    score_report["summary"]["min_score"] = scored == 0 ? 0.0 : minimum;
    score_report["summary"]["max_score"] = scored == 0 ? 0.0 : maximum;

    std::vector<HierarchicalSimulation> simulations_data;
    std::map<std::string, std::size_t> simulation_indices;
    YAML::Node flat_runs{YAML::NodeType::Sequence};
    std::size_t run_index = 1;
    for (std::size_t index = 0; index < report.runs.size(); ++index) {
        const auto& result = report.runs[index];
        const auto& paths = valid_run_config_paths[index];
        const auto node = executedRun(run_index++, paths, result);
        appendRun(simulations_data, simulation_indices, paths, node, &result);
        flat_runs.push_back(flatRun(node, paths, &result));
    }
    for (const auto& rejected : rejected_runs) {
        const RunConfigurationPaths paths{
            rejected.simulation_config_file, rejected.mission_config_file,
            rejected.drone_config_file, rejected.lidar_config_file};
        const auto node = rejectedRun(run_index++, rejected);
        appendRun(simulations_data, simulation_indices, paths, node, nullptr);
        flat_runs.push_back(flatRun(node, paths, nullptr));
    }

    YAML::Node simulations{YAML::NodeType::Sequence};
    for (const auto& simulation : simulations_data) {
        YAML::Node simulation_node;
        simulation_node["simulation_config"] = simulation.file.string();
        YAML::Node missions{YAML::NodeType::Sequence};
        for (const auto& mission : simulation.missions) {
            YAML::Node mission_node;
            mission_node["mission_config"] = mission.file.string();
            if (mission.has_resolution) {
                mission_node["resolution_cm"] = mission.resolution;
                mission_node["resolution_request_status"] =
                    mission.resolution_status;
            }
            mission_node["runs"] = mission.runs;
            missions.push_back(mission_node);
        }
        simulation_node["missions"] = missions;
        simulations.push_back(simulation_node);
    }
    score_report["simulations"] = simulations;
    score_report["runs"] = flat_runs;
    YAML::Node root;
    root["score_report"] = score_report;
    writeYaml(root, output_file);
}

std::vector<ComparativeResultGroup>
SimulationReportWriter::groupComparativeResults(
    const std::vector<PluginResultSummary>& results)
{
    std::map<std::pair<double, std::size_t>, std::vector<std::string>> groups;
    for (const auto& result : results) {
        groups[{result.total_score, result.total_steps}].push_back(
            result.plugin_file);
    }
    std::vector<ComparativeResultGroup> grouped;
    grouped.reserve(groups.size());
    for (auto& [key, plugins] : groups) {
        std::sort(plugins.begin(), plugins.end());
        grouped.push_back(
            {std::move(plugins), key.first, key.second});
    }
    std::sort(grouped.begin(), grouped.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.mission_controls.size() != rhs.mission_controls.size()) {
            return lhs.mission_controls.size() > rhs.mission_controls.size();
        }
        if (lhs.total_score != rhs.total_score) {
            return lhs.total_score > rhs.total_score;
        }
        if (lhs.total_steps != rhs.total_steps) {
            return lhs.total_steps < rhs.total_steps;
        }
        return lhs.mission_controls < rhs.mission_controls;
    });
    return grouped;
}

std::vector<PluginResultSummary>
SimulationReportWriter::rankCompetitionResults(
    std::vector<PluginResultSummary> results)
{
    std::sort(results.begin(), results.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.total_score != rhs.total_score) {
            return lhs.total_score > rhs.total_score;
        }
        if (lhs.total_steps != rhs.total_steps) {
            return lhs.total_steps < rhs.total_steps;
        }
        return lhs.plugin_file < rhs.plugin_file;
    });
    return results;
}

void SimulationReportWriter::writeComparativeReport(
    const ComparativeReportData& report,
    const std::filesystem::path& output_file)
{
    YAML::Node body;
    body["composition_file"] = report.composition_file.string();
    body["mission_control_folder"] = report.mission_control_folder.string();
    body["generated_at_utc"] = report.generated_at_utc;
    YAML::Node results{YAML::NodeType::Sequence};
    for (const auto& group : groupComparativeResults(report.successful_results)) {
        YAML::Node node;
        node["same_results"] = group.mission_controls;
        node["total_score"] = group.total_score;
        node["total_steps"] = group.total_steps;
        results.push_back(node);
    }
    body["results_summary"] = results;
    body["errors"] = report.errors;
    YAML::Node root;
    root["comparative_report"] = body;
    writeYaml(root, output_file);
}

void SimulationReportWriter::writeCompetitionReport(
    const CompetitionReportData& report,
    const std::filesystem::path& output_file)
{
    YAML::Node body;
    body["composition_file"] = report.composition_file.string();
    body["mission_control"] = report.mission_control_file.string();
    body["generated_at_utc"] = report.generated_at_utc;
    YAML::Node results{YAML::NodeType::Sequence};
    for (const auto& result : rankCompetitionResults(report.successful_results)) {
        YAML::Node node;
        node["algorithm"] = result.plugin_file;
        node["total_score"] = result.total_score;
        node["total_steps"] = result.total_steps;
        results.push_back(node);
    }
    body["results_summary"] = results;
    body["errors"] = report.errors;
    YAML::Node root;
    root["competitive_report"] = body;
    writeYaml(root, output_file);
}

} // namespace simulator
