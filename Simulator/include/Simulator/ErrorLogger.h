/**
 * @file ErrorLogger.h
 * @brief Declares thread-safe Simulator error logging.
 *
 * Configuration, plugin, and run failures are written immediately so one bad
 * job cannot erase diagnostics from otherwise independent jobs.
 */
#pragma once

#include <filesystem>
#include <mutex>
#include <string_view>

namespace simulator
{

/** @brief Classifies how a logged problem affects execution. */
enum class ErrorSeverity
{
    Recovered, ///< A documented fallback was applied.
    ScenarioFailure, ///< Affected runs score -1 while independent runs continue.
    Fatal, ///< The requested simulation cannot continue.
};

/** @brief Stable machine-readable categories used in Simulator logs. */
enum class ErrorCode
{
    InitialAngleMissing, ///< A default initial heading was used.
    MissionBoundaryDefaulted, ///< A mission boundary was recovered from the map.
    IgnoredOutputResolutionFactor, ///< A requested output resolution was unsupported.
    GpsResolutionMissing, ///< A default GPS resolution was used.
    DroneMaxRotateDefaulted, ///< A default rotation limit was used.
    DroneMaxAdvanceDefaulted, ///< A default advance limit was used.
    DroneMaxElevateDefaulted, ///< A default elevation limit was used.
    LidarZMinDefaulted, ///< A default minimum range was used.
    LidarZMaxDefaulted, ///< A default maximum range was used.
    LidarSpacingDefaulted, ///< A default beam spacing was used.
    LidarFovCirclesDefaulted, ///< A default field-of-view circle count was used.
    SimulationConfigRejected, ///< A simulation YAML file was rejected.
    MissionConfigRejected, ///< A mission YAML file was rejected.
    DroneConfigRejected, ///< A drone YAML file was rejected.
    LidarConfigRejected, ///< A LiDAR YAML file was rejected.
    PluginLoadFailed, ///< A shared library could not be loaded or registered.
    SimulationRunFailed, ///< One executable simulation run failed.
    ConfigurationError, ///< The top-level composition was invalid.
    ProgramFailure, ///< An unexpected non-plugin program failure occurred.
};

/**
 * @brief Returns the stable text representation of an error code.
 * @param code Error category to convert.
 * @return Static name suitable for logs and YAML reports.
 */
[[nodiscard]] std::string_view errorCodeName(ErrorCode code) noexcept;

/**
 * @brief Immediately appends complete structured entries to one shared log.
 *
 * Calls are serialized, allowing Simulator worker threads to report failures
 * without interleaving their lines.
 */
class ErrorLogger final
{
public:
    /**
     * @brief Stores the destination used by subsequent log operations.
     * @param log_file Error-log path, created immediately (empty when no errors occur).
     * @throws std::runtime_error If the destination cannot be initialized.
     */
    explicit ErrorLogger(std::filesystem::path log_file);

    /**
     * @brief Writes and flushes one structured log entry.
     * @param severity Execution impact of the problem.
     * @param code Stable machine-readable category.
     * @param message Human-readable context.
     * @throws std::runtime_error If the directory or file cannot be written.
     */
    void log(ErrorSeverity severity,
             ErrorCode code,
             std::string_view message) const;

    /**
     * @brief Logs a problem corrected with a fallback value.
     * @param code Stable machine-readable category.
     * @param message Description of the fallback.
     * @throws std::runtime_error If writing fails.
     */
    void recovered(ErrorCode code, std::string_view message) const;

    /**
     * @brief Logs a problem that rejects only affected simulation runs.
     * @param code Stable machine-readable category.
     * @param message Description of the rejected input or run.
     * @throws std::runtime_error If writing fails.
     */
    void scenarioFailure(ErrorCode code, std::string_view message) const;

    /**
     * @brief Logs a problem that prevents the requested invocation from running.
     * @param code Stable machine-readable category.
     * @param message Description of the fatal problem.
     * @throws std::runtime_error If writing fails.
     */
    void fatal(ErrorCode code, std::string_view message) const;

private:
    std::filesystem::path log_file_; ///< Destination for all entries.
    mutable std::mutex mutex_; ///< Prevents line interleaving between workers.
};

} // namespace simulator
