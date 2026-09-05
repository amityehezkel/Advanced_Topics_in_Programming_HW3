/**
 * @file MockGPS.h
 * @brief Declares the Simulator's resolution-aware GPS implementation.
 *
 * The mock stores an exact simulation pose while exposing a quantized position
 * through the course IGPS interface.
 */
#pragma once

#include <Common/IGPS.h>

namespace simulator
{

/** @brief Simulated GPS exposing a resolution-quantized view of an exact pose. */
class MockGPS final : public common::IGPS
{
public:
    /**
     * @brief Initializes the exact position, heading, and GPS resolution.
     * @param position Initial exact world-space position.
     * @param heading Initial orientation, normalized by the implementation.
     * @param resolution Position quantization interval.
     */
    MockGPS(common::Position3D position,
            common::Orientation heading,
            common::PhysicalLength resolution);

    /**
     * @brief Returns the position rounded to the configured GPS resolution.
     * @return Quantized world-space position.
     */
    [[nodiscard]] common::Position3D position() const override;

    /**
     * @brief Returns the current normalized orientation.
     * @return Drone heading.
     */
    [[nodiscard]] common::Orientation heading() const override;

    /**
     * @brief Returns the unquantized position used by movement physics.
     * @return Exact simulated world-space position.
     */
    [[nodiscard]] common::Position3D actualPosition() const;

    /**
     * @brief Updates the exact simulated position.
     * @param position New world-space position.
     */
    void setPosition(common::Position3D position);

    /**
     * @brief Normalizes and stores a new orientation.
     * @param heading New drone heading.
     */
    void setHeading(common::Orientation heading);

private:
    common::Position3D actual_position_{}; ///< Exact pose used by the Simulator.
    common::Orientation heading_{}; ///< Current normalized heading.
    common::PhysicalLength resolution_{}; ///< GPS quantization interval.
};

} // namespace simulator
