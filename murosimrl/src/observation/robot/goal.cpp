// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/goal.hpp"

#include <algorithm>

namespace murosimrl::observation::robot
{

Goal::Goal(const size_t stack_size)
    : ObservationRobot("goal", stack_size)
{
}

std::vector<size_t> Goal::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    return ObservationBase::shape(robots.size(), {3});
}

void Goal::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const size_t stack_size = effectiveStackSize();
    size_t index = 0;

    for (size_t i = 0; i < robots.size(); i++)
    {
        const murosim::robot::Robot &robot = *robots[i];

        for (size_t j = 0; j < stack_size; j++)
        {
            const murosim::geometry::Circle goal = robot.getStateAtOrOldest(j).getGoal().getCircle();

            output[index++] = goal.getX();
            output[index++] = goal.getY();
            output[index++] = goal.getRadius();
        }
    }
}

} // namespace murosimrl::observation::robot
