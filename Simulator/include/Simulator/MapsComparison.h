/**
 * @file MapsComparison.h
 * @brief Declares output-map scoring against a hidden ground-truth map.
 *
 * Scoring remains a Simulator concern so neither dynamically loaded plugin can
 * observe the hidden map.
 */
#pragma once

#include <Common/IMap3D.h>

#include <vector>

namespace simulator
{

/** @brief Computes exact-occupancy agreement scores for generated maps. */
class MapsComparison final
{
public:
    /**
     * @brief Compares each non-null target with the origin map.
     * @param origin Hidden ground-truth occupancy map.
     * @param targets Generated maps evaluated over their configured bounds.
     * @return One percentage score in [0,100] for each target.
     * @throws std::invalid_argument If resolutions are incompatible.
     */
    [[nodiscard]] static std::vector<double> compare(
        const common::IMap3D& origin,
        const std::vector<const common::IMap3D*>& targets);
};

} // namespace simulator
