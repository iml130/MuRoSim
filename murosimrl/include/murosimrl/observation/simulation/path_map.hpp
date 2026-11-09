// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_SIMULATION_PATH_MAP_HPP_
#define MUROSIMRL_OBSERVATION_SIMULATION_PATH_MAP_HPP_

#include "murosimrl/observation/observation_simulation.hpp"

namespace murosimrl::observation::simulation
{

/// Occupancy grid map of each simulation (as OccupancyMap, embedded centered
/// in a map_size x map_size buffer with the same value convention), with
/// every robot's planned path cells overwritten to -1.0. Batch size is the
/// number of simulations. The map must fit within map_size x map_size
/// (throws otherwise). Reads Simulation::getGridMap() and each robot's
/// getPathPoints(); path points outside the grid are silently skipped.
class PathMap : public ObservationSimulation
{
private:
    size_t map_size_;
    bool with_buffer_;

    /// Fill `array` with one embedded map plus path overlay per simulation.
    void fillObservationPathMap(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const override;

public:
    /// Path points are not part of the robot state, so there is no stacking;
    /// the fixed size-1 dimension is the channel axis for channel-first CNN
    /// input (same reasoning as GoalMap).
    /// @param map_size Side length of the square output buffer, in cells; must be at least the largest grid map's size. Throws std::runtime_error if 0.
    /// @param with_buffer Map BUFFER cells to 0.5 instead of treating them as free.
    PathMap(const size_t map_size, const bool with_buffer);

    /// Feature shape is [1, map_size, map_size] per simulation (channel-first, no stacking).
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const override;

    /// The configured output buffer side length, in cells.
    size_t mapSize() const { return map_size_; }
    /// Whether BUFFER cells map to 0.5 instead of free.
    bool withBuffer() const { return with_buffer_; }
};

} // namespace murosimrl::observation::simulation

#endif // MUROSIMRL_OBSERVATION_SIMULATION_PATH_MAP_HPP_
