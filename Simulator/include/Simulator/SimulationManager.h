/**
 * @file SimulationManager.h
 * @brief Declares expansion and parallel execution of one plugin pairing.
 *
 * The manager expands the composition Cartesian product, creates stable result
 * slots and run paths in advance, and applies the Assignment 3 thread-count
 * semantics without sharing run instances between workers.
 */
#pragma once

#include <Simulator/ErrorLogger.h>
#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRun.h>
#include <Simulator/ISimulationRunFactory.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace simulator
{

/** @brief Expands and executes every valid configuration for one plugin pair. */
class SimulationManager final : public ISimulation
{
public:
    /**
     * @brief Constructs a manager for one Algorithm/MissionControl pairing.
     * @param run_factory Factory used sequentially to create independent runs.
     * @param simulation_threads Main-only when 1; otherwise requested worker count.
     * @param error_logger Optional non-owning sink shared safely by workers.
     */
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory,
                      std::size_t simulation_threads,
                      const ErrorLogger* error_logger = nullptr);

    /**
     * @brief Expands, executes, and aggregates all valid configuration combinations.
     * @param composition Fully parsed immutable simulation composition.
     * @param output_path Results directory for this plugin pairing.
     * @return Detailed report in deterministic Cartesian-product order.
     */
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    /** @brief Associates one owned run with its preallocated result slot. */
    struct PreparedRun
    {
        std::size_t result_index = 0; ///< Stable index in the final report.
        std::unique_ptr<ISimulationRun> run; ///< Independent executable object graph.
    };

    /**
     * @brief Executes prepared runs on the main thread or a bounded worker set.
     * @param prepared_runs Fully constructed independent runs.
     * @param results Preallocated result table updated at distinct indices.
     */
    void executePreparedRuns(
        std::vector<PreparedRun>& prepared_runs,
        std::vector<types::SimulationResult>& results) const;

    std::unique_ptr<ISimulationRunFactory> run_factory_; ///< Sequential composition root.
    std::size_t simulation_threads_ = 1; ///< Assignment-defined thread setting.
    const ErrorLogger* error_logger_ = nullptr; ///< Optional thread-safe error sink.
};

} // namespace simulator
