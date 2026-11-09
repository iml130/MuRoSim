// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot_simulation/goal_normalized.hpp"
#include "murosimrl/util/simulation.hpp"
#include <algorithm>

namespace murosimrl::observation::robot_simulation
{

GoalNormalized::GoalNormalized(const float normalize_distance, const size_t stack_size)
    : ObservationRobotSimulation("goal_normalized", stack_size)
    , normalize_distance_(std::max(0.0f, normalize_distance))
{
}

std::vector<size_t> GoalNormalized::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const size_t batch_size = util::simulation::getNumberOfRobots(simulations);
    return ObservationBase::shape(batch_size, {2});
}

void GoalNormalized::compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    fillObservationGoalNormalized(output, simulations);
}

void GoalNormalized::fillObservationGoalNormalized(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const float normalize_inv = (0.0f < normalize_distance_) ? 1.0f / normalize_distance_ : 1.0f;
    const Eigen::Vector2f normalize_half = (0.0f < normalize_distance_) ? Eigen::Vector2f(0.5f, 0.5f) : Eigen::Vector2f::Zero();
    const size_t stack_size = effectiveStackSize();
    size_t index = 0;

    for (const auto &sim : simulations)
    {
        const murosim::geometry::World &world = sim->getWorld();
        const Eigen::Vector2f center = (world.getAabbMin() + world.getAabbMax()) * 0.5f;

        for (const auto& robotPtr : sim->getRobots())
        {
            const murosim::robot::Robot &robot = *robotPtr;

            for (size_t stack_index = 0; stack_index < stack_size; stack_index++)
            {
                const Eigen::Vector2f &goal = robot.getStateAtOrOldest(stack_index).getGoal().getPoint();
                const Eigen::Vector2f position = (goal - center) * normalize_inv + normalize_half;
                array[index++] = position.x();
                array[index++] = position.y();
            }
        }
    }
}

} // namespace murosimrl::observation::robot_simulation
