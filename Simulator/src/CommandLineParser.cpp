#include <Simulator/CommandLineParser.h>

#include <charconv>
#include <fstream>
#include <limits>
#include <sstream>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace simulator
{
namespace
{

[[nodiscard]] std::filesystem::path requireFile(std::string_view value,
                                                std::string_view key)
{
    const std::filesystem::path path{value};
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error) || error) {
        throw CommandLineError("'" + std::string{key} +
                               "' must name a readable file: " +
                               path.string());
    }
    std::ifstream input{path, std::ios::binary};
    if (!input) {
        throw CommandLineError("'" + std::string{key} +
                               "' cannot be opened for reading: " +
                               path.string());
    }
    if (key != "simulation" && path.extension() != ".so") {
        throw CommandLineError("'" + std::string{key} +
                               "' must name a .so file: " + path.string());
    }
    return std::filesystem::absolute(path, error).lexically_normal();
}

[[nodiscard]] std::filesystem::path requireDirectory(std::string_view value,
                                                     std::string_view key)
{
    const std::filesystem::path path{value};
    std::error_code error;
    if (!std::filesystem::is_directory(path, error) || error) {
        throw CommandLineError("'" + std::string{key} +
                               "' must name a readable directory: " +
                               path.string());
    }
    bool found_library = false;
    for (std::filesystem::directory_iterator iterator{path, error}, end;
         !error && iterator != end; iterator.increment(error)) {
        std::error_code entry_error;
        if (iterator->is_regular_file(entry_error) && !entry_error &&
            iterator->path().extension() == ".so") {
            found_library = true;
        }
    }
    if (error) {
        throw CommandLineError("'" + std::string{key} +
                               "' cannot be traversed: " + path.string());
    }
    if (!found_library) {
        throw CommandLineError("'" + std::string{key} +
                               "' contains no .so files: " + path.string());
    }
    return std::filesystem::absolute(path, error).lexically_normal();
}

[[nodiscard]] std::size_t parseThreadCount(std::string_view value)
{
    if (value.empty() || value.front() == '-') {
        throw CommandLineError("'num_threads' must be a positive integer.");
    }
    unsigned long long parsed = 0;
    const auto conversion =
        std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (conversion.ec != std::errc{} ||
        conversion.ptr != value.data() + value.size() || parsed == 0 ||
        parsed > std::numeric_limits<std::size_t>::max()) {
        throw CommandLineError("'num_threads' must be a positive integer.");
    }
    return static_cast<std::size_t>(parsed);
}

} // namespace

CommandLineOptions CommandLineParser::parse(int argc,
                                            const char* const argv[])
{
    if (argv == nullptr) {
        throw CommandLineError("The command-line argument array is invalid.");
    }

    std::optional<RunMode> mode;
    bool verbose = false;
    std::unordered_map<std::string, std::string> values;
    std::vector<std::string> diagnostics;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument = argv[index] == nullptr
                                              ? std::string_view{}
                                              : std::string_view{argv[index]};
        if (argument == "-comparative" || argument == "-competition") {
            if (mode) {
                diagnostics.push_back(
                    "Exactly one run mode must be supplied; found an extra '" +
                    std::string{argument} + "'.");
                continue;
            }
            mode = argument == "-comparative" ? RunMode::Comparative
                                                : RunMode::Competition;
            continue;
        }
        if (argument == "-verbose") {
            if (verbose) {
                diagnostics.push_back("Duplicate argument '-verbose'.");
                continue;
            }
            verbose = true;
            continue;
        }
        const auto separator = argument.find('=');
        if (separator == std::string_view::npos || separator == 0 ||
            separator + 1 == argument.size()) {
            diagnostics.push_back("Unsupported argument '" +
                                  std::string{argument} + "'.");
            continue;
        }
        const std::string key{argument.substr(0, separator)};
        const std::string value{argument.substr(separator + 1)};
        if (!values.emplace(key, value).second) {
            diagnostics.push_back("Duplicate argument '" + key + "'.");
        }
    }
    if (!mode) {
        diagnostics.push_back("Missing '-comparative' or '-competition'.");
    }

    const std::unordered_set<std::string> permitted =
        mode && *mode == RunMode::Comparative
            ? std::unordered_set<std::string>{"simulation",
                                              "mission_control_folder",
                                              "algorithm", "num_threads"}
            : std::unordered_set<std::string>{"simulation", "mission_control",
                                              "algorithms_folder", "num_threads"};
    for (const auto& [key, unused] : values) {
        static_cast<void>(unused);
        if (!permitted.contains(key)) {
            diagnostics.push_back("Unsupported argument '" + key + "'.");
        }
    }
    if (mode) {
        const std::vector<std::string_view> required_keys =
            *mode == RunMode::Comparative
                ? std::vector<std::string_view>{"simulation",
                                                "mission_control_folder",
                                                "algorithm"}
                : std::vector<std::string_view>{"simulation", "mission_control",
                                                "algorithms_folder"};
        for (const auto key : required_keys) {
            if (!values.contains(std::string{key})) {
                diagnostics.push_back("Missing required argument '" +
                                      std::string{key} + "'.");
            }
        }
    }
    if (!diagnostics.empty()) {
        std::ostringstream message;
        message << "Invalid command line:";
        for (const auto& diagnostic : diagnostics) {
            message << "\n  - " << diagnostic;
        }
        throw CommandLineError(message.str());
    }

    const auto required = [&values](std::string_view key) -> const std::string& {
        const auto found = values.find(std::string{key});
        return found->second;
    };

    CommandLineOptions options;
    options.mode = *mode;
    options.composition_file = requireFile(required("simulation"), "simulation");
    options.verbose = verbose;
    if (const auto threads = values.find("num_threads"); threads != values.end()) {
        options.simulation_threads = parseThreadCount(threads->second);
    }
    if (*mode == RunMode::Comparative) {
        options.mission_control_folder = requireDirectory(
            required("mission_control_folder"), "mission_control_folder");
        options.algorithm_file = requireFile(required("algorithm"), "algorithm");
    } else {
        options.mission_control_file =
            requireFile(required("mission_control"), "mission_control");
        options.algorithms_folder = requireDirectory(
            required("algorithms_folder"), "algorithms_folder");
    }
    return options;
}

std::string CommandLineParser::usage()
{
    return
        "Usage:\n"
        "  simulator_212200943 -comparative simulation=<file> "
        "mission_control_folder=<folder> algorithm=<file> "
        "[num_threads=<positive integer>] [-verbose]\n"
        "  simulator_212200943 -competition simulation=<file> "
        "mission_control=<file> algorithms_folder=<folder> "
        "[num_threads=<positive integer>] [-verbose]";
}

} // namespace simulator
