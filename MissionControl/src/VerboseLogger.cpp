#include <MissionControl/VerboseLogger.h>

#include <Common/Units.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace mission_control_212200943
{
namespace
{

[[nodiscard]] std::string droneStepStatusName(common::types::DroneStepStatus status)
{
    switch (status)
    {
    case common::types::DroneStepStatus::Continue:
        return "Continue";
    case common::types::DroneStepStatus::Completed:
        return "Completed";
    case common::types::DroneStepStatus::Error:
        return "Error";
    }
    return "Unknown";
}

[[nodiscard]] std::string missionRunStatusName(common::types::MissionRunStatus status)
{
    switch (status)
    {
    case common::types::MissionRunStatus::Completed:
        return "Completed";
    case common::types::MissionRunStatus::MaxSteps:
        return "MaxSteps";
    case common::types::MissionRunStatus::Error:
        return "Error";
    }
    return "Unknown";
}

} // namespace

VerboseLogger::VerboseLogger(const std::filesystem::path& output_map_file, bool enabled)
    : log_file_(makeVerbosePath(output_map_file)),
      enabled_(enabled)
{
    if (!enabled_)
    {
        return;
    }

    std::error_code error;
    const std::filesystem::path parent = log_file_.parent_path();
    if (!parent.empty())
    {
        std::filesystem::create_directories(parent, error);
        if (error)
        {
            throw std::runtime_error(
                "Cannot create verbose-output directory: " + error.message());
        }
    }

    stream_.open(log_file_, std::ios::trunc);
    if (!stream_)
    {
        throw std::runtime_error(
            "Cannot create verbose-output file: " + log_file_.string());
    }
    stream_ << "MissionControl verbose trace\n";
    if (!stream_)
    {
        throw std::runtime_error(
            "Cannot initialize verbose-output file: " + log_file_.string());
    }
}

bool VerboseLogger::enabled() const noexcept
{
    return enabled_;
}

const std::filesystem::path& VerboseLogger::path() const noexcept
{
    return log_file_;
}

void VerboseLogger::logStep(const common::types::DroneState& state,
                            const common::types::DroneStepResult& result) const
{
    if (!enabled_)
    {
        return;
    }

    std::ostringstream line;
    line << "step=" << state.step_index
         << " status=" << droneStepStatusName(result.status)
         << " position_cm=("
         << state.position.x.force_numerical_value_in(common::cm) << ','
         << state.position.y.force_numerical_value_in(common::cm) << ','
         << state.position.z.force_numerical_value_in(common::cm) << ')'
         << " heading_deg=("
         << state.heading.horizontal.force_numerical_value_in(common::deg) << ','
         << state.heading.altitude.force_numerical_value_in(common::deg) << ')';
    if (!result.message.empty())
    {
        line << " message=" << result.message;
    }
    appendLine(line.str());
}

void VerboseLogger::logTerminal(const common::types::MissionRunResult& result) const
{
    if (!enabled_)
    {
        return;
    }

    std::ostringstream line;
    line << "mission_status=" << missionRunStatusName(result.status)
         << " steps=" << result.steps
         << " errors=" << result.errors.size();
    appendLine(line.str());
    for (const common::types::ErrorRef& error : result.errors)
    {
        appendLine("error=" + error.code + ": " + error.message);
    }
    stream_.flush();
    if (!stream_)
    {
        throw std::runtime_error(
            "Cannot flush verbose-output file: " + log_file_.string());
    }
}

void VerboseLogger::logMessage(std::string_view message) const
{
    if (enabled_)
    {
        appendLine(message);
    }
}

std::filesystem::path VerboseLogger::makeVerbosePath(
    const std::filesystem::path& output_map_file)
{
    const std::string stem = output_map_file.stem().string();
    return output_map_file.parent_path() / (stem + "_verbose.log");
}

void VerboseLogger::appendLine(std::string_view line) const
{
    if (!stream_)
    {
        throw std::runtime_error(
            "Verbose-output file is not writable: " + log_file_.string());
    }
    stream_ << line << '\n';
    if (!stream_)
    {
        throw std::runtime_error(
            "Cannot write verbose-output file: " + log_file_.string());
    }
}

} // namespace mission_control_212200943
