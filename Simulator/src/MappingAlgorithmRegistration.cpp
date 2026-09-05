#include <Common/MappingAlgorithmRegistration.h>
#include <Simulator/FactoryRegistry.h>

#include <utility>

namespace common
{

MappingAlgorithmRegistration::MappingAlgorithmRegistration(
    MappingAlgorithmFactory factory)
{
    simulator::FactoryRegistry::instance().addMappingAlgorithm(std::move(factory));
}

} // namespace common
