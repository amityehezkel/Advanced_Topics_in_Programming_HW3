#include <Common/MissionControlRegistration.h>
#include <Simulator/FactoryRegistry.h>

#include <utility>

namespace common
{

MissionControlRegistration::MissionControlRegistration(
    MissionControlFactory factory)
{
    simulator::FactoryRegistry::instance().addMissionControl(std::move(factory));
}

} // namespace common
