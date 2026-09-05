/**
 * @file FactoryRegistry.h
 * @brief Declares the Simulator-side target of automatic plugin registration.
 *
 * The immutable Common registration constructors forward factories here while
 * dlopen executes a plugin's static initialization. Loading is sequential;
 * extraction transfers factory ownership out before a library may be closed.
 */
#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <cstddef>
#include <mutex>
#include <vector>

namespace simulator
{

/** @brief Collects factories emitted by currently loading shared libraries. */
class FactoryRegistry final
{
public:
    /**
     * @brief Returns the process-wide registration target.
     * @return Singleton registry used by Common registration constructors.
     */
    [[nodiscard]] static FactoryRegistry& instance() noexcept;

    /**
     * @brief Appends one MappingAlgorithm factory during static registration.
     * @param factory Factory supplied by REGISTER_MAPPING_ALGORITHM.
     */
    void addMappingAlgorithm(common::MappingAlgorithmFactory factory);

    /**
     * @brief Appends one MissionControl factory during static registration.
     * @param factory Factory supplied by REGISTER_MISSION_CONTROL.
     */
    void addMissionControl(common::MissionControlFactory factory);

    /**
     * @brief Returns the current MappingAlgorithm registration checkpoint.
     * @return Number of pending MappingAlgorithm factories.
     */
    [[nodiscard]] std::size_t mappingAlgorithmCount() const;

    /**
     * @brief Returns the current MissionControl registration checkpoint.
     * @return Number of pending MissionControl factories.
     */
    [[nodiscard]] std::size_t missionControlCount() const;

    /**
     * @brief Removes and returns MappingAlgorithm factories added after a checkpoint.
     * @param checkpoint Count captured immediately before dlopen.
     * @return Newly registered factories in registration order.
     * @throws std::out_of_range If checkpoint exceeds the current count.
     */
    [[nodiscard]] std::vector<common::MappingAlgorithmFactory>
    extractMappingAlgorithmsFrom(std::size_t checkpoint);

    /**
     * @brief Removes and returns MissionControl factories added after a checkpoint.
     * @param checkpoint Count captured immediately before dlopen.
     * @return Newly registered factories in registration order.
     * @throws std::out_of_range If checkpoint exceeds the current count.
     */
    [[nodiscard]] std::vector<common::MissionControlFactory>
    extractMissionControlsFrom(std::size_t checkpoint);

    /** @brief Registry lifetime is controlled by instance(). */
    FactoryRegistry(const FactoryRegistry&) = delete;

    /** @brief The singleton registry cannot be copy-assigned. */
    FactoryRegistry& operator=(const FactoryRegistry&) = delete;

private:
    /** @brief Constructs the initially empty singleton registry. */
    FactoryRegistry() = default;

    mutable std::mutex mutex_; ///< Serializes registration and extraction.
    std::vector<common::MappingAlgorithmFactory> mapping_algorithms_; ///< Pending algorithm factories.
    std::vector<common::MissionControlFactory> mission_controls_; ///< Pending control factories.
};

} // namespace simulator
