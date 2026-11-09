// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_GOAL_NORMALIZED_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_GOAL_NORMALIZED_HPP_

#include "murosimrl/observation/observation_robot_simulation.hpp"

namespace murosimrl::observation::robot_simulation
{

/// Per-robot goal position (x, y), measured relative to the simulation
/// world's AABB center and, if normalize_distance is positive, divided by it
/// and shifted by 0.5 (so the world center maps to (0.5, 0.5)); with
/// normalize_distance <= 0 the raw metre offset from the world center is
/// reported instead. Feature size is 2 per stacked state. Batch size is the
/// total robot count across all simulations. Reads Simulation::getWorld()'s
/// AABB and each robot's getStateAtOrOldest(j).getGoal().
class GoalNormalized : public ObservationRobotSimulation
{
private:
    float normalize_distance_;

    /// Fill `array` with one (x, y) pair per robot per stacked state, in simulation then robot order.
    void fillObservationGoalNormalized(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const override;

public:
    /// @param normalize_distance Distance (metres) the offset from world center is divided by; <= 0 disables normalization.
    /// @param stack_size Number of stacked past goals per robot; 0 and 1 are equivalent (no stacking).
    GoalNormalized(const float normalize_distance, const size_t stack_size);

    /// Per-robot feature size is 2 (x, y), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const override;

    /// The configured normalization distance (<= 0 means "no normalization").
    float normalizeDistance() const { return normalize_distance_; }
};

} // namespace murosimrl::observation::robot_simulation

#endif // MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_GOAL_NORMALIZED_HPP_
