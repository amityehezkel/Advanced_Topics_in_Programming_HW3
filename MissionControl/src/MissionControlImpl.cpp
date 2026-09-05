#include <MissionControl/MissionControlImpl.h>

#include <MissionControl/DroneControlImpl.h>

#include <Common/MissionControlRegistration.h>

#include <exception>
#include <memory>
#include <string>
#include <utility>

namespace mission_control_212200943
{

MissionControlImpl::MissionControlImpl(common::MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      output_map_(dependencies.output_map),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_logger_(output_map_file_, dependencies.verbose),
      drone_control_(std::make_unique<DroneControlImpl>(
          dependencies.drone_config,
          dependencies.mission_config,
          dependencies.lidar,
          dependencies.gps,
          dependencies.movement,
          dependencies.output_map,
          dependencies.mapping_algorithm))
{
}

common::types::MissionRunResult MissionControlImpl::runMission()
{
    for (std::size_t step = 0; step < mission_.max_steps; ++step)
    {
        common::types::DroneStepResult step_result;
        try
        {
            step_result = drone_control_->step();
            verbose_logger_.logStep(drone_control_->state(), step_result);
        }
        catch (const std::exception& error)
        {
            return saveMap(common::types::MissionRunResult{
                common::types::MissionRunStatus::Error,
                step,
                {common::types::ErrorRef{"MISSION_STEP_FAILED", error.what()}},
            });
        }
        catch (...)
        {
            return saveMap(common::types::MissionRunResult{
                common::types::MissionRunStatus::Error,
                step,
                {common::types::ErrorRef{
                    "MISSION_STEP_FAILED",
                    "Mission step failed with a non-standard exception."}},
            });
        }

        if (step_result.status == common::types::DroneStepStatus::Completed)
        {
            return saveMap(common::types::MissionRunResult{
                common::types::MissionRunStatus::Completed,
                step + 1,
                {},
            });
        }
        if (step_result.status == common::types::DroneStepStatus::Error)
        {
            return saveMap(common::types::MissionRunResult{
                common::types::MissionRunStatus::Error,
                step + 1,
                {common::types::ErrorRef{
                    "DRONE_CONTROL_ERROR",
                    step_result.message}},
            });
        }
    }

    return saveMap(common::types::MissionRunResult{
        common::types::MissionRunStatus::MaxSteps,
        mission_.max_steps,
        {},
    });
}

common::types::MissionRunResult MissionControlImpl::saveMap(
    common::types::MissionRunResult result) const
{
    try
    {
        output_map_.save(output_map_file_);
    }
    catch (const std::exception& error)
    {
        result.status = common::types::MissionRunStatus::Error;
        result.errors.push_back(
            common::types::ErrorRef{"MAP_SAVE_FAILED", error.what()});
    }
    catch (...)
    {
        result.status = common::types::MissionRunStatus::Error;
        result.errors.push_back(common::types::ErrorRef{
            "MAP_SAVE_FAILED",
            "Output map save failed with a non-standard exception."});
    }

    try
    {
        verbose_logger_.logTerminal(result);
    }
    catch (const std::exception& error)
    {
        result.status = common::types::MissionRunStatus::Error;
        result.errors.push_back(
            common::types::ErrorRef{"VERBOSE_LOG_FAILED", error.what()});
    }
    catch (...)
    {
        result.status = common::types::MissionRunStatus::Error;
        result.errors.push_back(common::types::ErrorRef{
            "VERBOSE_LOG_FAILED",
            "Verbose logging failed with a non-standard exception."});
    }

    return result;
}

REGISTER_MISSION_CONTROL(MissionControlImpl);

} // namespace mission_control_212200943
