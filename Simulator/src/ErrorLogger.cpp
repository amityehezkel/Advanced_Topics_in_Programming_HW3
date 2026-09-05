#include <Simulator/ErrorLogger.h>

#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace simulator
{

std::string_view errorCodeName(ErrorCode code) noexcept
{
    switch (code) {
    case ErrorCode::InitialAngleMissing: return "INITIAL_ANGLE_MISSING";
    case ErrorCode::MissionBoundaryDefaulted: return "MISSION_BOUNDARY_DEFAULTED";
    case ErrorCode::IgnoredOutputResolutionFactor: return "IGNORED_OUTPUT_RESOLUTION_FACTOR";
    case ErrorCode::GpsResolutionMissing: return "GPS_RESOLUTION_MISSING";
    case ErrorCode::DroneMaxRotateDefaulted: return "DRONE_MAX_ROTATE_DEFAULTED";
    case ErrorCode::DroneMaxAdvanceDefaulted: return "DRONE_MAX_ADVANCE_DEFAULTED";
    case ErrorCode::DroneMaxElevateDefaulted: return "DRONE_MAX_ELEVATE_DEFAULTED";
    case ErrorCode::LidarZMinDefaulted: return "LIDAR_Z_MIN_DEFAULTED";
    case ErrorCode::LidarZMaxDefaulted: return "LIDAR_Z_MAX_DEFAULTED";
    case ErrorCode::LidarSpacingDefaulted: return "LIDAR_SPACING_DEFAULTED";
    case ErrorCode::LidarFovCirclesDefaulted: return "LIDAR_FOV_CIRCLES_DEFAULTED";
    case ErrorCode::SimulationConfigRejected: return "SIMULATION_CONFIG_REJECTED";
    case ErrorCode::MissionConfigRejected: return "MISSION_CONFIG_REJECTED";
    case ErrorCode::DroneConfigRejected: return "DRONE_CONFIG_REJECTED";
    case ErrorCode::LidarConfigRejected: return "LIDAR_CONFIG_REJECTED";
    case ErrorCode::PluginLoadFailed: return "PLUGIN_LOAD_FAILED";
    case ErrorCode::SimulationRunFailed: return "SIMULATION_RUN_FAILED";
    case ErrorCode::ConfigurationError: return "CONFIGURATION_ERROR";
    case ErrorCode::ProgramFailure: return "PROGRAM_FAILURE";
    }
    return "UNKNOWN_ERROR";
}

ErrorLogger::ErrorLogger(std::filesystem::path log_file)
    : log_file_(std::move(log_file))
{
    std::error_code error;
    if (!log_file_.parent_path().empty()) {
        std::filesystem::create_directories(log_file_.parent_path(), error);
        if (error) {
            throw std::runtime_error("Cannot create error-log directory '" +
                                     log_file_.parent_path().string() + "': " +
                                     error.message());
        }
    }
    std::ofstream output{log_file_, std::ios::trunc};
    if (!output) {
        throw std::runtime_error("Cannot initialize error log '" +
                                 log_file_.string() + "'.");
    }
}

void ErrorLogger::log(ErrorSeverity severity,
                      ErrorCode code,
                      std::string_view message) const
{
    std::string_view severity_name = "FATAL";
    switch (severity) {
    case ErrorSeverity::Recovered: severity_name = "RECOVERED"; break;
    case ErrorSeverity::ScenarioFailure: severity_name = "SCENARIO_ERROR"; break;
    case ErrorSeverity::Fatal: severity_name = "FATAL"; break;
    }

    const std::scoped_lock lock{mutex_};
    std::error_code error;
    if (!log_file_.parent_path().empty()) {
        std::filesystem::create_directories(log_file_.parent_path(), error);
        if (error) {
            throw std::runtime_error("Cannot create error-log directory '" +
                                     log_file_.parent_path().string() + "': " +
                                     error.message());
        }
    }
    std::ofstream output{log_file_, std::ios::app};
    if (!output) {
        throw std::runtime_error("Cannot open error log '" +
                                 log_file_.string() + "'.");
    }
    output << '[' << severity_name << "] " << errorCodeName(code) << ": "
           << message << '\n';
    output.flush();
    if (!output) {
        throw std::runtime_error("Cannot write error log '" +
                                 log_file_.string() + "'.");
    }
}

void ErrorLogger::recovered(ErrorCode code, std::string_view message) const
{
    log(ErrorSeverity::Recovered, code, message);
}

void ErrorLogger::scenarioFailure(ErrorCode code,
                                  std::string_view message) const
{
    log(ErrorSeverity::ScenarioFailure, code, message);
}

void ErrorLogger::fatal(ErrorCode code, std::string_view message) const
{
    log(ErrorSeverity::Fatal, code, message);
}

} // namespace simulator
