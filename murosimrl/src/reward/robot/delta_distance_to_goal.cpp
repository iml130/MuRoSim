// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/delta_distance_to_goal.hpp"
#include "murosim/robot/util.hpp"

namespace murosimrl::reward::robot
{

DeltaDistanceToGoal::DeltaDistanceToGoal(const float positive, const float negative)
    : RewardRobot("delta_distance_to_goal")
    , positive_(positive)
    , negative_(negative)
{
}

void DeltaDistanceToGoal::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        float reward = 0.0f;

        if (1 < robots[i]->getStateSize())
        {
            const murosim::robot::State &current = robots[i]->getStateAt(0);
            const murosim::robot::State &previous = robots[i]->getStateAt(1);

            const float distance_current = murosim::robot::util::distanceToGoal(current.getPose(), current.getGoal());
            const float distance_previous = murosim::robot::util::distanceToGoal(previous.getPose(), previous.getGoal());

            const float distance_difference = distance_previous - distance_current;

            reward = (distance_difference < 0.0f) ? -distance_difference * negative_ : distance_difference * positive_;
        }

        output[i] = reward;
    }
}

} // namespace murosimrl::reward::robot
