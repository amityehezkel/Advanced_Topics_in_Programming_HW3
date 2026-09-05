#include <Simulator/ConfigurationLoader.h>

#include <TinyNPY.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>

namespace simulator
{
namespace
{

[[noreturn]] void fail(const std::string& message)
{
    throw ConfigurationError(message);
}

[[nodiscard]] std::filesystem::path absolutePath(
    const std::filesystem::path& path)
{
    std::error_code error;
    const auto result = std::filesystem::absolute(path, error);
    if (error) {
        fail("Cannot resolve path '" + path.string() + "': " + error.message());
    }
    return result.lexically_normal();
}

[[nodiscard]] std::filesystem::path resolveReference(
    const std::filesystem::path& owner,
    const std::string& reference)
{
    if (reference.empty()) {
        fail("An empty file path was found in '" + owner.string() + "'.");
    }
    const std::filesystem::path value{reference};
    return value.is_absolute() ? value.lexically_normal()
                               : (owner.parent_path() / value).lexically_normal();
}

[[nodiscard]] std::filesystem::path resolveAsset(
    const std::filesystem::path& owner,
    const std::filesystem::path& composition,
    const std::string& reference)
{
    const auto owner_relative = resolveReference(owner, reference);
    if (std::filesystem::is_regular_file(owner_relative)) {
        return owner_relative;
    }
    const auto composition_relative = resolveReference(composition, reference);
    return std::filesystem::is_regular_file(composition_relative)
               ? composition_relative
               : owner_relative;
}

void requireFile(const std::filesystem::path& path, std::string_view role)
{
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error) || error) {
        fail(std::string{role} + " file does not exist or is not readable: '" +
             path.string() + "'.");
    }
}

[[nodiscard]] YAML::Node loadYaml(const std::filesystem::path& path,
                                  std::string_view role)
{
    requireFile(path, role);
    try {
        return YAML::LoadFile(path.string());
    } catch (const YAML::Exception& error) {
        fail("Invalid YAML in '" + path.string() + "': " + error.what());
    }
}

[[nodiscard]] YAML::Node mapping(const YAML::Node& parent,
                                 std::string_view key,
                                 const std::filesystem::path& file)
{
    const YAML::Node value = parent[std::string{key}];
    if (!value || !value.IsMap()) {
        fail("'" + std::string{key} + "' must be a mapping in '" +
             file.string() + "'.");
    }
    return value;
}

[[nodiscard]] YAML::Node sequence(const YAML::Node& parent,
                                  std::string_view key,
                                  const std::filesystem::path& file)
{
    const YAML::Node value = parent[std::string{key}];
    if (!value || !value.IsSequence() || value.size() == 0) {
        fail("'" + std::string{key} + "' must be a non-empty sequence in '" +
             file.string() + "'.");
    }
    return value;
}

template<typename Value>
[[nodiscard]] Value scalar(const YAML::Node& parent,
                           std::string_view key,
                           const std::filesystem::path& file)
{
    const YAML::Node value = parent[std::string{key}];
    if (!value || !value.IsScalar()) {
        fail("Missing scalar '" + std::string{key} + "' in '" +
             file.string() + "'.");
    }
    try {
        return value.as<Value>();
    } catch (const YAML::Exception& error) {
        fail("Invalid value for '" + std::string{key} + "' in '" +
             file.string() + "': " + error.what());
    }
}

[[nodiscard]] double finiteNumber(const YAML::Node& parent,
                                  std::string_view key,
                                  const std::filesystem::path& file)
{
    const double value = scalar<double>(parent, key, file);
    if (!std::isfinite(value)) {
        fail("'" + std::string{key} + "' must be finite in '" +
             file.string() + "'.");
    }
    return value;
}

[[nodiscard]] double positiveNumber(const YAML::Node& parent,
                                    std::string_view key,
                                    const std::filesystem::path& file)
{
    const double value = finiteNumber(parent, key, file);
    if (value <= 0.0) {
        fail("'" + std::string{key} + "' must be greater than zero in '" +
             file.string() + "'.");
    }
    return value;
}

[[nodiscard]] double numberOrDefault(const YAML::Node& parent,
                                     std::string_view key,
                                     double fallback,
                                     const std::filesystem::path& file,
                                     const ErrorLogger& logger,
                                     ErrorCode code,
                                     bool positive = false,
                                     std::string_view display_key = {})
{
    const std::string name = display_key.empty() ? std::string{key}
                                                  : std::string{display_key};
    const YAML::Node node = parent[std::string{key}];
    if (!node) {
        logger.recovered(code, "Missing '" + name + "' in '" + file.string() +
                                   "'; using default value " +
                                   std::to_string(fallback) + ".");
        return fallback;
    }
    const double value = finiteNumber(parent, key, file);
    if (positive && value <= 0.0) {
        logger.recovered(code, "Invalid '" + name + "' in '" + file.string() +
                                   "'; using default value " +
                                   std::to_string(fallback) + ".");
        return fallback;
    }
    return value;
}

[[nodiscard]] std::size_t sizeOrDefault(const YAML::Node& parent,
                                        std::string_view key,
                                        std::size_t fallback,
                                        const std::filesystem::path& file,
                                        const ErrorLogger& logger,
                                        ErrorCode code)
{
    if (!parent[std::string{key}]) {
        logger.recovered(code, "Missing '" + std::string{key} + "' in '" +
                                   file.string() + "'; using default value " +
                                   std::to_string(fallback) + ".");
        return fallback;
    }
    const long long value = scalar<long long>(parent, key, file);
    if (value < 0) {
        logger.recovered(code, "Invalid '" + std::string{key} + "' in '" +
                                   file.string() + "'; using default value " +
                                   std::to_string(fallback) + ".");
        return fallback;
    }
    return static_cast<std::size_t>(value);
}

[[nodiscard]] std::size_t positiveSize(const YAML::Node& parent,
                                       std::string_view key,
                                       const std::filesystem::path& file)
{
    const long long value = scalar<long long>(parent, key, file);
    if (value <= 0 || static_cast<unsigned long long>(value) >
                          std::numeric_limits<std::size_t>::max()) {
        fail("'" + std::string{key} + "' must be a positive integer in '" +
             file.string() + "'.");
    }
    return static_cast<std::size_t>(value);
}

[[nodiscard]] common::Position3D parsePosition(
    const YAML::Node& node,
    const std::filesystem::path& file)
{
    return {
        finiteNumber(node, "x_cm", file) * common::x_extent[common::cm],
        finiteNumber(node, "y_cm", file) * common::y_extent[common::cm],
        finiteNumber(node, "height_cm", file) * common::z_extent[common::cm]};
}

[[nodiscard]] types::SimulationConfigData loadSimulation(
    const std::filesystem::path& raw_file,
    const std::filesystem::path& composition_file,
    const ErrorLogger& logger)
{
    const auto file = absolutePath(raw_file);
    const auto config = mapping(loadYaml(file, "Simulation configuration"),
                                "simulation_config", file);
    const auto map_file = resolveAsset(
        file, composition_file, scalar<std::string>(config, "map_filename", file));
    requireFile(map_file, "NPY map");
    const auto position = mapping(config, "initial_drone_position", file);
    const auto offset = mapping(config, "map_axes_offset", file);
    double initial_angle = 0.0;
    if (config["initial_angle_deg"]) {
        initial_angle = finiteNumber(config, "initial_angle_deg", file);
    } else {
        logger.recovered(ErrorCode::InitialAngleMissing,
                         "Missing 'initial_angle_deg' in '" + file.string() +
                             "'; using default value 0 degrees.");
    }
    if (initial_angle < 0.0 || initial_angle >= 360.0) {
        fail("'initial_angle_deg' must be in [0, 360) in '" + file.string() +
             "'.");
    }
    return {
        map_file,
        positiveNumber(config, "map_resolution_cm", file) * common::cm,
        {finiteNumber(offset, "x_offset", file) * common::x_extent[common::cm],
         finiteNumber(offset, "y_offset", file) * common::y_extent[common::cm],
         finiteNumber(offset, "height_offset", file) *
             common::z_extent[common::cm]},
        parsePosition(position, file),
        initial_angle * common::horizontal_angle[common::deg]};
}

[[nodiscard]] common::types::MappingBounds mapBounds(
    const types::SimulationConfigData& simulation)
{
    NpyArray array;
    const char* error = array.LoadNPY(simulation.map_filename.string());
    if (error != nullptr) {
        fail("Cannot read NPY map '" + simulation.map_filename.string() +
             "': " + error);
    }
    const auto& shape = array.Shape();
    if (shape.size() != 3 || shape[0] == 0 || shape[1] == 0 || shape[2] == 0) {
        fail("NPY map must contain a non-empty three-dimensional array: '" +
             simulation.map_filename.string() + "'.");
    }
    const double resolution =
        simulation.map_resolution.force_numerical_value_in(common::cm);
    return {
        -simulation.map_offset.x,
        (static_cast<double>(shape[0]) * resolution -
         simulation.map_offset.x.force_numerical_value_in(common::cm)) *
            common::x_extent[common::cm],
        -simulation.map_offset.y,
        (static_cast<double>(shape[1]) * resolution -
         simulation.map_offset.y.force_numerical_value_in(common::cm)) *
            common::y_extent[common::cm],
        -simulation.map_offset.z,
        (static_cast<double>(shape[2]) * resolution -
         simulation.map_offset.z.force_numerical_value_in(common::cm)) *
            common::z_extent[common::cm]};
}

template<typename Length, typename Unit>
[[nodiscard]] std::pair<Length, Length> boundaryOrDefault(
    const YAML::Node& boundaries,
    std::string_view name,
    Length map_min,
    Length map_max,
    const std::filesystem::path& file,
    const ErrorLogger& logger,
    Unit unit)
{
    const YAML::Node boundary = boundaries[std::string{name}];
    if (boundary && !boundary.IsMap()) {
        fail("'" + std::string{name} + "' must be a mapping in '" +
             file.string() + "'.");
    }
    const YAML::Node usable = boundary ? boundary : YAML::Node{YAML::NodeType::Map};
    const double map_min_value = map_min.force_numerical_value_in(common::cm);
    const double map_max_value = map_max.force_numerical_value_in(common::cm);
    const double minimum = numberOrDefault(
        usable, "min_cm", map_min_value, file, logger,
        ErrorCode::MissionBoundaryDefaulted, false,
        std::string{name} + ".min_cm");
    const double maximum = numberOrDefault(
        usable, "max_cm", map_max_value, file, logger,
        ErrorCode::MissionBoundaryDefaulted, false,
        std::string{name} + ".max_cm");
    if (minimum > maximum || minimum < map_min_value || maximum > map_max_value) {
        fail("'" + std::string{name} +
             "' must be ordered and inside the input map in '" +
             file.string() + "'.");
    }
    return {minimum * unit, maximum * unit};
}

[[nodiscard]] common::types::MissionConfigData loadMission(
    const std::filesystem::path& raw_file,
    const types::SimulationConfigData& simulation,
    const ErrorLogger& logger)
{
    const auto file = absolutePath(raw_file);
    const auto config = mapping(loadYaml(file, "Mission configuration"),
                                "mission_config", file);
    double factor = 1.0;
    if (config["output_mapping_resolution_factor"]) {
        factor = finiteNumber(config, "output_mapping_resolution_factor", file);
        if (std::floor(factor) != factor) {
            fail("'output_mapping_resolution_factor' must be an integer in '" +
                 file.string() + "'.");
        }
        if (factor < 1.0) {
            logger.recovered(
                ErrorCode::IgnoredOutputResolutionFactor,
                "Value below 1 in '" + file.string() +
                    "'; the simulation will ignore the resolution request.");
        }
    }
    const double gps_resolution = numberOrDefault(
        config, "gps_resolution_cm",
        simulation.map_resolution.force_numerical_value_in(common::cm), file,
        logger, ErrorCode::GpsResolutionMissing);
    if (gps_resolution <= 0.0) {
        fail("'gps_resolution_cm' must be greater than zero in '" +
             file.string() + "'.");
    }

    const auto defaults = mapBounds(simulation);
    const YAML::Node boundaries = config["boundaries"]
                                      ? config["boundaries"]
                                      : YAML::Node{YAML::NodeType::Map};
    if (!boundaries.IsMap()) {
        fail("'boundaries' must be a mapping in '" + file.string() + "'.");
    }
    const auto x = boundaryOrDefault(boundaries, "x_boundary", defaults.min_x,
                                     defaults.max_x, file, logger,
                                     common::x_extent[common::cm]);
    const auto y = boundaryOrDefault(boundaries, "y_boundary", defaults.min_y,
                                     defaults.max_y, file, logger,
                                     common::y_extent[common::cm]);
    const auto z = boundaryOrDefault(
        boundaries, "height_boundary", defaults.min_height,
        defaults.max_height, file, logger, common::z_extent[common::cm]);
    return {positiveSize(config, "max_steps", file), gps_resolution * common::cm,
            factor, {x.first, x.second, y.first, y.second, z.first, z.second}};
}

[[nodiscard]] common::types::DroneConfigData loadDrone(
    const std::filesystem::path& raw_file,
    const ErrorLogger& logger)
{
    const auto file = absolutePath(raw_file);
    const auto config = mapping(loadYaml(file, "Drone configuration"),
                                "drone_config", file);
    const double max_rotate = numberOrDefault(
        config, "max_rotate_deg", 360.0, file, logger,
        ErrorCode::DroneMaxRotateDefaulted, true);
    if (max_rotate > 360.0) {
        fail("'max_rotate_deg' must not exceed 360 in '" + file.string() + "'.");
    }
    return {
        positiveNumber(config, "dimensions_cm", file) / 2.0 * common::cm,
        max_rotate * common::horizontal_angle[common::deg],
        numberOrDefault(config, "max_advance_cm", 10.0, file, logger,
                        ErrorCode::DroneMaxAdvanceDefaulted, true) * common::cm,
        numberOrDefault(config, "max_elevate_cm", 10.0, file, logger,
                        ErrorCode::DroneMaxElevateDefaulted, true) * common::cm};
}

[[nodiscard]] common::types::LidarConfigData loadLidar(
    const std::filesystem::path& raw_file,
    const ErrorLogger& logger)
{
    const auto file = absolutePath(raw_file);
    const auto config = mapping(loadYaml(file, "LiDAR configuration"),
                                "lidar_config", file);
    double z_min = numberOrDefault(config, "z_min_cm", 0.0, file, logger,
                                   ErrorCode::LidarZMinDefaulted);
    if (z_min < 0.0) {
        logger.recovered(ErrorCode::LidarZMinDefaulted,
                         "Invalid 'z_min_cm' in '" + file.string() +
                             "'; using default value 0.");
        z_min = 0.0;
    }
    const double z_max = numberOrDefault(
        config, "z_max_cm", 120.0, file, logger,
        ErrorCode::LidarZMaxDefaulted, true);
    if (z_min > z_max) {
        fail("'z_min_cm' must not exceed 'z_max_cm' in '" + file.string() +
             "'.");
    }
    return {z_min * common::cm, z_max * common::cm,
            numberOrDefault(config, "d_cm", 2.5, file, logger,
                            ErrorCode::LidarSpacingDefaulted, true) * common::cm,
            sizeOrDefault(config, "fov_circles", 4, file, logger,
                          ErrorCode::LidarFovCirclesDefaulted)};
}

template<typename Config>
struct ConfigSlot
{
    std::filesystem::path file;
    std::optional<Config> value;
    std::optional<common::types::ErrorRef> error;
};

struct SimulationSlots
{
    ConfigSlot<types::SimulationConfigData> simulation;
    std::vector<ConfigSlot<common::types::MissionConfigData>> missions;
};

template<typename Config>
void reject(ConfigSlot<Config>& slot,
            ErrorCode code,
            const std::exception& error,
            const ErrorLogger& logger)
{
    slot.error = common::types::ErrorRef{
        std::string{errorCodeName(code)}, error.what()};
    logger.scenarioFailure(code, "Rejected configuration '" +
                                     slot.file.string() + "': " + error.what());
}

[[nodiscard]] CompositionLoadResult loadComposition(
    const std::filesystem::path& raw_file,
    const ErrorLogger& logger)
{
    const auto file = absolutePath(raw_file);
    const auto composition = mapping(loadYaml(file, "Simulation composition"),
                                     "simulation_compositions", file);
    const auto simulation_nodes = sequence(composition, "simulations", file);
    const auto drone_nodes = sequence(composition, "drone_configs", file);
    const auto lidar_nodes = sequence(composition, "lidar_configs", file);

    std::vector<SimulationSlots> groups;
    for (const auto& simulation_node : simulation_nodes) {
        if (!simulation_node.IsMap()) {
            fail("Every item in 'simulations' must be a mapping in '" +
                 file.string() + "'.");
        }
        SimulationSlots group;
        group.simulation.file = resolveReference(
            file, scalar<std::string>(simulation_node, "simulation_config", file));
        try {
            group.simulation.value =
                loadSimulation(group.simulation.file, file, logger);
        } catch (const std::exception& error) {
            reject(group.simulation, ErrorCode::SimulationConfigRejected, error,
                   logger);
        }
        for (const auto& mission_node :
             sequence(simulation_node, "mission_configs", file)) {
            if (!mission_node.IsScalar()) {
                fail("Every mission configuration must be a file path in '" +
                     file.string() + "'.");
            }
            ConfigSlot<common::types::MissionConfigData> mission;
            mission.file = resolveReference(file, mission_node.as<std::string>());
            if (group.simulation.value) {
                try {
                    mission.value =
                        loadMission(mission.file, *group.simulation.value, logger);
                } catch (const std::exception& error) {
                    reject(mission, ErrorCode::MissionConfigRejected, error, logger);
                }
            }
            group.missions.push_back(std::move(mission));
        }
        groups.push_back(std::move(group));
    }

    std::vector<ConfigSlot<common::types::DroneConfigData>> drones;
    for (const auto& node : drone_nodes) {
        if (!node.IsScalar()) {
            fail("Every drone configuration must be a file path in '" +
                 file.string() + "'.");
        }
        ConfigSlot<common::types::DroneConfigData> slot;
        slot.file = resolveReference(file, node.as<std::string>());
        try {
            slot.value = loadDrone(slot.file, logger);
        } catch (const std::exception& error) {
            reject(slot, ErrorCode::DroneConfigRejected, error, logger);
        }
        drones.push_back(std::move(slot));
    }

    std::vector<ConfigSlot<common::types::LidarConfigData>> lidars;
    for (const auto& node : lidar_nodes) {
        if (!node.IsScalar()) {
            fail("Every LiDAR configuration must be a file path in '" +
                 file.string() + "'.");
        }
        ConfigSlot<common::types::LidarConfigData> slot;
        slot.file = resolveReference(file, node.as<std::string>());
        try {
            slot.value = loadLidar(slot.file, logger);
        } catch (const std::exception& error) {
            reject(slot, ErrorCode::LidarConfigRejected, error, logger);
        }
        lidars.push_back(std::move(slot));
    }

    CompositionLoadResult result;
    result.valid_composition.composition_file = file;
    for (const auto& group : groups) {
        if (!group.simulation.value) continue;
        std::vector<common::types::MissionConfigData> missions;
        for (const auto& mission : group.missions) {
            if (mission.value) missions.push_back(*mission.value);
        }
        if (!missions.empty()) {
            result.valid_composition.simulation_mission_groups.emplace_back(
                *group.simulation.value, std::move(missions));
        }
    }
    for (const auto& drone : drones) {
        if (drone.value) result.valid_composition.drone_configs.push_back(*drone.value);
    }
    for (const auto& lidar : lidars) {
        if (lidar.value) result.valid_composition.lidar_configs.push_back(*lidar.value);
    }

    for (const auto& group : groups) {
        for (const auto& mission : group.missions) {
            for (const auto& drone : drones) {
                for (const auto& lidar : lidars) {
                    std::vector<common::types::ErrorRef> errors;
                    if (group.simulation.error) errors.push_back(*group.simulation.error);
                    if (mission.error) errors.push_back(*mission.error);
                    if (drone.error) errors.push_back(*drone.error);
                    if (lidar.error) errors.push_back(*lidar.error);
                    if (errors.empty()) {
                        result.valid_run_config_paths.push_back(
                            {group.simulation.file, mission.file, drone.file, lidar.file});
                    } else {
                        result.rejected_runs.push_back(
                            {group.simulation.file, mission.file, drone.file,
                             lidar.file, std::move(errors), -1.0});
                    }
                }
            }
        }
    }
    return result;
}

} // namespace

ConfigurationLoader::ConfigurationLoader(const ErrorLogger& error_logger)
    : error_logger_(error_logger)
{
}

CompositionLoadResult ConfigurationLoader::loadSimulationComposition(
    const std::filesystem::path& composition_file) const
{
    try {
        return loadComposition(composition_file, error_logger_);
    } catch (const ConfigurationError& error) {
        error_logger_.fatal(ErrorCode::ConfigurationError, error.what());
        throw;
    } catch (const YAML::Exception& error) {
        const ConfigurationError converted{
            std::string{"Invalid YAML value: "} + error.what()};
        error_logger_.fatal(ErrorCode::ConfigurationError, converted.what());
        throw converted;
    }
}

} // namespace simulator
