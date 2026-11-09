// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/orientation_to_goal.hpp"
#include "murosimrl/util/robot.hpp"

#include <algorithm>

namespace murosimrl::observation::robot
{

OrientationToGoal::OrientationToGoal(const float threshold_radius, const size_t stack_size)
    : ObservationRobot("orientation_to_goal", stack_size)
    , threshold_radius_(std::max(0.0f, threshold_radius))
{
}

std::vector<size_t> OrientationToGoal::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    return ObservationBase::shape(robots.size(), {2});
}

void OrientationToGoal::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const size_t stack_size = effectiveStackSize();
    size_t index = 0;

    for (size_t i = 0; i < robots.size(); i++)
    {
        const murosim::robot::Robot &robot = *robots[i];

        for (size_t j = 0; j < stack_size; j++)
        {
            const murosim::robot::State &state = robot.getStateAtOrOldest(j);

            Eigen::Vector2f orientation_to_goal = util::robot::orientationToGoalFromState(state);
            const float distance_to_goal = util::robot::distanceToGoalFromState(state);

            if (distance_to_goal < threshold_radius_)
            {
                orientation_to_goal *= distance_to_goal / threshold_radius_;
            }

            output[index++] = orientation_to_goal.x();
            output[index++] = orientation_to_goal.y();
        }
    }
}

} // namespace murosimrl::observation::robot
