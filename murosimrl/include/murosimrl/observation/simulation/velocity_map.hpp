// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_SIMULATION_VELOCITY_MAP_HPP_
#define MUROSIMRL_OBSERVATION_SIMULATION_VELOCITY_MAP_HPP_

#include "murosimrl/observation/observation_simulation.hpp"

namespace murosimrl::observation::simulation
{

/// Rasterized world-frame velocity field of each simulation's robots,
/// embedded centered in a map_size x map_size grid (same centering as
/// OccupancyMap). Channel 0 and 1 are the x and y world-frame velocity
/// components (metres/second; row order is y-flipped like the grid map, so
/// channel 1 stores the negated y velocity) written at each robot's occupied
/// cells; channel 2 (present only if history_size > 0) is a recency weight
/// in (0, 1] that fades older states in the same stacked frame towards 0.
/// Feature shape per stacked state is [2, map_size, map_size] (history_size
/// == 0) or [3, map_size, map_size]. Cells with no robot stay 0. Each robot
/// occupies either just its center cell, or every cell within its collision
/// radius if use_collision_radius is set (using the same rasterization as
/// the grid map). If history_size > 0, up to that many past states per
/// stacked index are blended into the same frame; robots without a recorded
/// state yet are skipped. The map must fit within map_size x map_size
/// (throws otherwise). Reads Simulation::getGridMap()/getConfig() and each
/// robot's pose/velocity/collision radius per stacked state.
class VelocityMap : public ObservationSimulation
{
private:
    size_t map_size_;
    size_t history_size_;
    bool use_collision_radius_;

    /// Append the flattened cell indices a robot at `robot_position` occupies to `indices`.
    /// `indices` is cleared and refilled; callers reuse one vector across the
    /// fill loops so the buffer allocation is amortized without shared state.
    void fillIndicesForRobotCollisionRadius(const murosim::grid::Map &map, const Eigen::Vector2f &robot_position, const float robot_collision_radius, const size_t start_x, const size_t start_y, std::vector<size_t> &indices) const;

    /// Fill `array` with one velocity map (and optional history channel) per stacked state per simulation.
    void fillObservationVelocity(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const override;

public:
    /// @param map_size Side length of the square output buffer, in cells; must be at least the largest grid map's size. Throws std::runtime_error if 0.
    /// @param history_size Number of past states blended per stacked frame via a fading recency channel; 0 disables the history channel.
    /// @param use_collision_radius Rasterize each robot's full collision-radius footprint instead of just its center cell.
    /// @param stack_size Number of stacked past frames per simulation; 0 and 1 are equivalent (no stacking).
    VelocityMap(const size_t map_size, const size_t history_size, const bool use_collision_radius, const size_t stack_size);

    /// Feature shape is [2 or 3, map_size, map_size] per stacked state, depending on history_size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const override;

    /// The configured output buffer side length, in cells.
    size_t mapSize() const noexcept { return map_size_; }
    /// The configured history depth (0 means "no history channel").
    size_t historySize() const noexcept { return history_size_; }
    /// Whether each robot's full collision-radius footprint is rasterized.
    bool useCollisionRadius() const noexcept { return use_collision_radius_; }
};

} // namespace murosimrl::observation::simulation

#endif // MUROSIMRL_OBSERVATION_SIMULATION_VELOCITY_MAP_HPP_
