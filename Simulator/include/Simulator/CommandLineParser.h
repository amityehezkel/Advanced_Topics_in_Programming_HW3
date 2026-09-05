/**
 * @file CommandLineParser.h
 * @brief Declares validation of comparative and competition command lines.
 *
 * The parser accepts arguments in any order, reports all unsupported or
 * missing arguments, validates requested paths, and converts thread semantics
 * into one strongly typed options object.
 */
#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

namespace simulator
{

/** @brief Selects which plugin dimension is compared by the Simulator. */
enum class RunMode
{
    Comparative, ///< One Algorithm with every MissionControl in a folder.
    Competition, ///< One MissionControl with every Algorithm in a folder.
};

/** @brief Fully validated inputs for one Simulator invocation. */
struct CommandLineOptions
{
    RunMode mode = RunMode::Comparative; ///< Selected operation mode.
    std::filesystem::path composition_file; ///< Simulation-composition YAML.
    std::optional<std::filesystem::path> mission_control_folder; ///< Comparative plugin folder.
    std::optional<std::filesystem::path> algorithm_file; ///< Comparative Algorithm library.
    std::optional<std::filesystem::path> mission_control_file; ///< Competition MissionControl library.
    std::optional<std::filesystem::path> algorithms_folder; ///< Competition plugin folder.
    std::size_t simulation_threads = 1; ///< Main-only for 1, otherwise worker count.
    bool verbose = false; ///< Enables per-mission verbose output.
};

/** @brief Reports invalid command-line syntax or arguments. */
class CommandLineError final : public std::runtime_error
{
public:
    /** @brief Inherits constructors accepting an explanatory error string. */
    using std::runtime_error::runtime_error;
};

/** @brief Parses and validates the complete Simulator command line. */
class CommandLineParser final
{
public:
    /**
     * @brief Converts process arguments into validated Simulator options.
     * @param argc Number of process arguments including the executable name.
     * @param argv Process argument array.
     * @return Mode-specific paths, thread count, and verbosity.
     * @throws CommandLineError If any argument is missing, unsupported, or invalid.
     */
    [[nodiscard]] static CommandLineOptions parse(
        int argc,
        const char* const argv[]);

    /**
     * @brief Returns usage text for both supported modes.
     * @return Human-readable usage description.
     */
    [[nodiscard]] static std::string usage();
};

} // namespace simulator
