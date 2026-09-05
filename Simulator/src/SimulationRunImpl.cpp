#include <Simulator/SimulationRunImpl.h>

#include <Simulator/MapsComparison.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace simulator
{

SimulationRunImpl::SimulationRunImpl(
    std::unique_ptr<const common::IMap3D> hidden_map,
    std::unique_ptr<common::IMutableMap3D> output_map,
    std::unique_ptr<common::IGPS> gps,
    std::unique_ptr<common::IDroneMovement> movement,
    std::unique_ptr<common::ILidar> lidar,
    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
    std::unique_ptr<common::IMissionControl> mission_control,
    types::SimulationConfigData simulation_config,
    common::types::MissionConfigData mission_config,
    std::filesystem::path output_map_file,
    types::ResolutionRequestStatus resolution_status)
    : hidden_map_(std::move(hidden_map)), output_map_(std::move(output_map)),
      gps_(std::move(gps)), movement_(std::move(movement)),
      lidar_(std::move(lidar)), mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)),
      resolution_status_(resolution_status)
{
    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ || !lidar_ ||
        !mapping_algorithm_ || !mission_control_) {
        throw std::invalid_argument(
            "SimulationRunImpl requires every injected dependency.");
    }
}

types::SimulationResult SimulationRunImpl::run()
{
    const common::types::MissionRunResult mission_result =
        mission_control_->runMission();
    double score = -1.0;
    if (mission_result.status != common::types::MissionRunStatus::Error) {
        const auto scores = MapsComparison::compare(*hidden_map_, {output_map_.get()});
        if (scores.size() != 1) {
            throw std::runtime_error("Map comparison returned an invalid result.");
        }
        score = scores.front();
    }
    return {simulation_config_, mission_config_, resolution_status_,
            {mission_result}, output_map_file_, output_map_->getMapConfig(), score};
}

} // namespace simulator
