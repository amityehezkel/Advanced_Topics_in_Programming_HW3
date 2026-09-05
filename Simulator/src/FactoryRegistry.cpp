#include <Simulator/FactoryRegistry.h>

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace simulator
{

FactoryRegistry& FactoryRegistry::instance() noexcept
{
    static FactoryRegistry registry;
    return registry;
}

void FactoryRegistry::addMappingAlgorithm(common::MappingAlgorithmFactory factory)
{
    const std::scoped_lock lock{mutex_};
    mapping_algorithms_.push_back(std::move(factory));
}

void FactoryRegistry::addMissionControl(common::MissionControlFactory factory)
{
    const std::scoped_lock lock{mutex_};
    mission_controls_.push_back(std::move(factory));
}

std::size_t FactoryRegistry::mappingAlgorithmCount() const
{
    const std::scoped_lock lock{mutex_};
    return mapping_algorithms_.size();
}

std::size_t FactoryRegistry::missionControlCount() const
{
    const std::scoped_lock lock{mutex_};
    return mission_controls_.size();
}

std::vector<common::MappingAlgorithmFactory>
FactoryRegistry::extractMappingAlgorithmsFrom(std::size_t checkpoint)
{
    const std::scoped_lock lock{mutex_};
    if (checkpoint > mapping_algorithms_.size()) {
        throw std::out_of_range("Invalid MappingAlgorithm registry checkpoint.");
    }
    std::vector<common::MappingAlgorithmFactory> result;
    result.reserve(mapping_algorithms_.size() - checkpoint);
    std::move(mapping_algorithms_.begin() + static_cast<std::ptrdiff_t>(checkpoint),
              mapping_algorithms_.end(), std::back_inserter(result));
    mapping_algorithms_.erase(
        mapping_algorithms_.begin() + static_cast<std::ptrdiff_t>(checkpoint),
        mapping_algorithms_.end());
    return result;
}

std::vector<common::MissionControlFactory>
FactoryRegistry::extractMissionControlsFrom(std::size_t checkpoint)
{
    const std::scoped_lock lock{mutex_};
    if (checkpoint > mission_controls_.size()) {
        throw std::out_of_range("Invalid MissionControl registry checkpoint.");
    }
    std::vector<common::MissionControlFactory> result;
    result.reserve(mission_controls_.size() - checkpoint);
    std::move(mission_controls_.begin() + static_cast<std::ptrdiff_t>(checkpoint),
              mission_controls_.end(), std::back_inserter(result));
    mission_controls_.erase(
        mission_controls_.begin() + static_cast<std::ptrdiff_t>(checkpoint),
        mission_controls_.end());
    return result;
}

} // namespace simulator
