// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/distance_to_goal.hpp"
#include "murosimrl/util/robot.hpp"

#include <algorithm>

namespace murosimrl::observation::robot
{

DistanceToGoal::DistanceToGoal(const float normalize_distance, const size_t stack_size)
    : ObservationRobot("distance_to_goal", stack_size)
    , normalize_distance_(std::max(0.0f, normalize_distance))
{
}

std::vector<size_t> DistanceToGoal::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    return ObservationBase::shape(robots.size(), {1});
}

void DistanceToGoal::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const float norm = (0.0f < normalize_distance_) ? 1.0f / normalize_distance_ : 1.0f;
    const size_t stack_size = effectiveStackSize();
    size_t index = 0;

    for (size_t i = 0; i < robots.size(); i++)
    {
        const murosim::robot::Robot &robot = *robots[i];

        for (size_t j = 0; j < stack_size; j++)
        {
            const murosim::robot::State &state = robot.getStateAtOrOldest(j);
            output[index++] = norm * util::robot::distanceToGoalFromState(state);
        }
    }
}

} // namespace murosimrl::observation::robot
