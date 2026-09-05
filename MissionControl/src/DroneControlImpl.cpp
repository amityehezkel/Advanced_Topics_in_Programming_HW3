#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <cmath>
#include <exception>
#include <utility>

namespace mission_control_212200943
{

    using common::cm;
    using common::IDroneMovement;
    using common::IGPS;
    using common::ILidar;
    using common::IMappingAlgorithm;
    using common::IMutableMap3D;

    namespace types = common::types;

    DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                       types::MissionConfigData mission,
                                       ILidar &lidar,
                                       IGPS &gps,
                                       IDroneMovement &movement,
                                       IMutableMap3D &output_map,
                                       IMappingAlgorithm &mapping_algorithm)
        : drone_(std::move(drone)),
          mission_(std::move(mission)),
          lidar_(lidar),
          gps_(gps),
          movement_(movement),
          output_map_(output_map),
          mapping_algorithm_(mapping_algorithm) {}

    types::DroneStepResult DroneControlImpl::step()
    {
        try
        {
            const types::LidarScanResult *previous_scan =
                latest_scan_ ? &*latest_scan_ : nullptr;
            const types::MappingStepCommand command =
                mapping_algorithm_.nextStep(state(), previous_scan);

            if (command.movement)
            {
                const types::MovementCommand &movement_command = *command.movement;
                types::MovementResult movement_result;
                switch (movement_command.type)
                {
                case types::MovementCommandType::Hover:
                    movement_result = {true, {}};
                    break;
                case types::MovementCommandType::Rotate:
                    if (movement_command.angle < 0.0 * common::horizontal_angle[common::deg] ||
                        movement_command.angle > drone_.max_rotate)
                    {
                        return {types::DroneStepStatus::Error,
                                "Mapping algorithm requested an invalid rotation."};
                    }
                    movement_result =
                        movement_.rotate(movement_command.rotation, movement_command.angle);
                    break;
                case types::MovementCommandType::Advance:
                    if (std::abs(movement_command.distance.force_numerical_value_in(cm)) >
                        drone_.max_advance.force_numerical_value_in(cm))
                    {
                        return {types::DroneStepStatus::Error,
                                "Mapping algorithm exceeded the advance limit."};
                    }
                    movement_result = movement_.advance(movement_command.distance);
                    break;
                case types::MovementCommandType::Elevate:
                    if (std::abs(movement_command.distance.force_numerical_value_in(cm)) >
                        drone_.max_elevate.force_numerical_value_in(cm))
                    {
                        return {types::DroneStepStatus::Error,
                                "Mapping algorithm exceeded the elevation limit."};
                    }
                    movement_result = movement_.elevate(movement_command.distance);
                    break;
                }
                if (!movement_result)
                {
                    return {types::DroneStepStatus::Error, movement_result.message};
                }
            }

            if (command.scan_orientation)
            {
                latest_scan_ = lidar_.scan(*command.scan_orientation);
                ScanResultToVoxels::applyToMap(
                    output_map_,
                    gps_.position(),
                    gps_.heading(),
                    *latest_scan_,
                    lidar_.config());
            }

            ++step_index_;
            if (command.status == types::AlgorithmStatus::Working)
            {
                return {types::DroneStepStatus::Continue, {}};
            }
            if (command.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels)
            {
                return {types::DroneStepStatus::Completed,
                        "Mapping completed with unmappable voxels."};
            }
            return {types::DroneStepStatus::Completed, {}};
        }
        catch (const std::exception &error)
        {
            return {types::DroneStepStatus::Error, error.what()};
        }
    }

    types::DroneState DroneControlImpl::state() const
    {
        return types::DroneState{gps_.position(), gps_.heading(), step_index_};
    }

} // namespace mission_control_212200943
