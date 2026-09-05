#include <Simulator/OutputManager.h>

#include <chrono>
#include <cctype>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace simulator
{
namespace
{

[[nodiscard]] std::string timestamp()
{
    const std::time_t now = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::tm value{};
#ifdef _WIN32
    localtime_s(&value, &now);
#else
    localtime_r(&now, &value);
#endif
    std::ostringstream output;
    output << std::put_time(&value, "%Y%m%d_%H%M%S");
    return output.str();
}

} // namespace

std::filesystem::path OutputManager::createResultsDirectory(
    const std::filesystem::path& parent,
    std::string_view prefix)
{
    const std::string base = sanitizeFilenameComponent(prefix) + "_" + timestamp();
    for (std::size_t suffix = 0; suffix < 10'000; ++suffix) {
        const std::filesystem::path candidate =
            parent / (suffix == 0 ? base : base + "_" + std::to_string(suffix));
        std::error_code error;
        if (std::filesystem::create_directory(candidate, error)) {
            return candidate;
        }
        if (error && error != std::errc::file_exists) {
            throw std::runtime_error("Cannot create results directory '" +
                                     candidate.string() + "': " +
                                     error.message());
        }
    }
    throw std::runtime_error("Cannot create a unique results directory under '" +
                             parent.string() + "'.");
}

std::filesystem::path OutputManager::createRunDirectory(
    const std::filesystem::path& results_directory,
    std::size_t run_index,
    std::string_view plugin_name)
{
    const std::filesystem::path directory =
        results_directory /
        ("run_" + std::to_string(run_index + 1) + "_" +
         sanitizeFilenameComponent(plugin_name));
    std::error_code error;
    if (!std::filesystem::create_directories(directory, error) &&
        (error || !std::filesystem::is_directory(directory))) {
        throw std::runtime_error("Cannot create run directory '" +
                                 directory.string() + "': " + error.message());
    }
    return directory;
}

std::string OutputManager::sanitizeFilenameComponent(std::string_view value)
{
    std::string result;
    result.reserve(value.size());
    for (const unsigned char character : value) {
        if (std::isalnum(character) || character == '-' || character == '_') {
            result.push_back(static_cast<char>(character));
        } else {
            result.push_back('_');
        }
    }
    if (result.empty()) {
        return "unnamed";
    }
    return result;
}

} // namespace simulator
