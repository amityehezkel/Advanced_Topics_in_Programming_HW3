/**
 * @file SimulatorApplication.h
 * @brief Declares top-level orchestration for one validated Simulator invocation.
 *
 * This class keeps main() small and coordinates output creation, configuration
 * loading, plugin lifetime, run execution, and mode-specific reporting.
 */
#pragma once

#include <Simulator/CommandLineParser.h>

#include <iosfwd>

namespace simulator
{

/** @brief Executes comparative or competition mode from validated options. */
class SimulatorApplication final
{
public:
    /**
     * @brief Constructs an application with injectable console streams.
     * @param output Normal status and usage output.
     * @param error Error and fatal diagnostic output.
     */
    SimulatorApplication(std::ostream& output, std::ostream& error) noexcept;

    /**
     * @brief Runs one complete Simulator invocation.
     * @param options Fully validated command-line options.
     * @return Process-style exit code: zero on success, nonzero on failure.
     */
    [[nodiscard]] int run(const CommandLineOptions& options) const;

private:
    /**
     * @brief Runs one Algorithm against every MissionControl plugin.
     * @param options Comparative-mode options.
     * @return Process-style exit code.
     */
    [[nodiscard]] int runComparative(const CommandLineOptions& options) const;

    /**
     * @brief Runs one MissionControl against every Algorithm plugin.
     * @param options Competition-mode options.
     * @return Process-style exit code.
     */
    [[nodiscard]] int runCompetition(const CommandLineOptions& options) const;

    std::ostream& output_; ///< Normal user-facing output stream.
    std::ostream& error_; ///< Error output stream.
};

} // namespace simulator
