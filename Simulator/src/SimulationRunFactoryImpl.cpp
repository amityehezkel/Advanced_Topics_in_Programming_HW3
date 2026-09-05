#include <Simulator/SimulationRunFactoryImpl.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunImpl.h>

#include <TinyNPY.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <typeinfo>
#include <utility>

namespace simulator
{
namespace
{

constexpr double kTolerance = 1e-9;

[[nodiscard]] double centimeters(common::PhysicalLength value)
{
    return value.force_numerical_value_in(common::cm);
}

[[nodiscard]] common::types::MappingBounds hiddenBounds(
    const types::SimulationConfigData& simulation,
    const NpyArray::shape_t& shape)
{
    const double resolution = centimeters(simulation.map_resolution);
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

[[nodiscard]] std::size_t alignedAxisSize(double minimum,
                                          double maximum,
                                          double resolution)
{
    const double intervals = (maximum - minimum) / resolution;
    if (intervals < -kTolerance ||
        std::abs(intervals - std::round(intervals)) > kTolerance) {
        throw std::invalid_argument(
            "Mission boundaries must align with the input-map resolution.");
    }
    const double cells = std::round(intervals);
    if (cells <= 0.0 ||
        cells > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
        throw std::overflow_error("Output map dimension is not representable.");
    }
    return static_cast<std::size_t>(cells);
}

[[nodiscard]] bool nearlyEqual(common::PhysicalLength lhs,
                               common::PhysicalLength rhs)
{
    return std::abs(centimeters(lhs) - centimeters(rhs)) <= kTolerance;
}

} // namespace

SimulationRunFactoryImpl::SimulationRunFactoryImpl(
    common::MappingAlgorithmFactory algorithm_factory,
    common::MissionControlFactory mission_control_factory,
    bool verbose,
    const ErrorLogger* error_logger)
    : algorithm_factory_(std::move(algorithm_factory)),
      mission_control_factory_(std::move(mission_control_factory)),
      verbose_(verbose), error_logger_(error_logger)
{
    if (!algorithm_factory_ || !mission_control_factory_) {
        throw std::invalid_argument(
            "SimulationRunFactoryImpl requires both plugin factories.");
    }
}

std::unique_ptr<ISimulationRun> SimulationRunFactoryImpl::create(
    const types::SimulationConfigData& simulation,
    const common::types::MissionConfigData& mission,
    const common::types::DroneConfigData& drone,
    const common::types::LidarConfigData& lidar,
    const std::filesystem::path& output_path)
{
    static_cast<void>(error_logger_);
    auto hidden_array = std::make_shared<NpyArray>();
    const char* load_error =
        hidden_array->LoadNPY(simulation.map_filename.string());
    if (load_error != nullptr) {
        throw std::runtime_error("Failed to load input NPY map '" +
                                 simulation.map_filename.string() + "': " +
                                 load_error);
    }
    if (hidden_array->Shape().size() != 3) {
        throw std::runtime_error("Input NPY map must have shape [X,Y,Z].");
    }
    auto hidden_map = std::make_unique<Map3DImpl>(
        hidden_array,
        common::types::MapConfig{hiddenBounds(simulation, hidden_array->Shape()),
                                 simulation.map_offset,
                                 simulation.map_resolution});

    const double resolution = centimeters(simulation.map_resolution);
    const auto& bounds = mission.mission_bounds;
    const NpyArray::shape_t output_shape{
        alignedAxisSize(bounds.min_x.force_numerical_value_in(common::cm),
                        bounds.max_x.force_numerical_value_in(common::cm),
                        resolution),
        alignedAxisSize(bounds.min_y.force_numerical_value_in(common::cm),
                        bounds.max_y.force_numerical_value_in(common::cm),
                        resolution),
        alignedAxisSize(
            bounds.min_height.force_numerical_value_in(common::cm),
            bounds.max_height.force_numerical_value_in(common::cm), resolution)};
    auto output_array =
        std::make_shared<NpyArray>(output_shape, sizeof(int), 'i', false);
    output_array->Allocate();
    std::fill_n(output_array->Data<int>(), output_array->NumValue(),
                static_cast<int>(common::types::VoxelOccupancy::Unmapped));
    auto output_map = std::make_unique<Map3DImpl>(
        output_array,
        common::types::MapConfig{
            bounds,
            common::Position3D{-bounds.min_x, -bounds.min_y, -bounds.min_height},
            simulation.map_resolution});

    types::ResolutionRequestStatus resolution_status =
        types::ResolutionRequestStatus::Ignored;
    if (mission.output_mapping_resolution_factor < 1.0) {
        resolution_status = types::ResolutionRequestStatus::IgnoredTooSmall;
    } else {
        const common::PhysicalLength requested =
            mission.gps_resolution * mission.output_mapping_resolution_factor;
        resolution_status = nearlyEqual(requested, simulation.map_resolution)
                                ? types::ResolutionRequestStatus::Accepted
                                : types::ResolutionRequestStatus::Ignored;
    }

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        common::Orientation{simulation.initial_angle,
                            0.0 * common::altitude_angle[common::deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(
        *gps, *hidden_map, mission.mission_bounds, drone.radius);
    const common::types::MovementResult initial_clearance =
        movement->advance(0.0 * common::cm);
    if (!initial_clearance) {
        throw std::runtime_error("Invalid initial drone position: " +
                                 initial_clearance.message);
    }
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = algorithm_factory_(common::MappingAlgorithmDependencies{
        mission, lidar, drone, *output_map});
    if (!mapping_algorithm) {
        throw std::runtime_error("MappingAlgorithm factory returned null.");
    }

    const std::string map_filename =
        "map_output_" + output_path.parent_path().filename().string() + "_" +
        output_path.filename().string() + ".npy";
    const std::filesystem::path output_map_file = output_path / map_filename;
    auto mission_control = mission_control_factory_(common::MissionControlDependencies{
        mission, drone, *lidar_impl, *gps, *movement, *output_map,
        *mapping_algorithm, output_map_file, verbose_});
    if (!mission_control) {
        throw std::runtime_error("MissionControl factory returned null.");
    }

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map), std::move(output_map), std::move(gps),
        std::move(movement), std::move(lidar_impl),
        std::move(mapping_algorithm), std::move(mission_control), simulation,
        mission, output_map_file, resolution_status);
}

} // namespace simulator
