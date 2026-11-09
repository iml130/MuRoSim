// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/on_goal.hpp"
#include "murosim/robot/util.hpp"

#include <algorithm>

namespace murosimrl::reward::robot
{

OnGoal::OnGoal(const float threshold_radius, const float positive)
    : RewardRobot("on_goal")
    , threshold_radius_(threshold_radius)
    , positive_(positive)
{
}

void OnGoal::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    if (threshold_radius_ <= 0.0f)
    {
        for (size_t i = 0; i < robots.size(); i++)
        {
            float reward = 0.0f;

            // getGoalRadius: the same radius Simulation uses for its
            // goal-reached check, so reward and termination agree
            const float radius = robots[i]->getGoal().getGoalRadius();

            if (0.0f < radius)
            {
                const float distance = murosim::robot::util::distanceToGoal(robots[i]->getPose(), robots[i]->getGoal());
                const float norm = std::max(0.0f, radius - distance) / radius;
                reward = positive_ * norm;
            }

            output[i] = reward;
        }
    }
    else
    {
        for (size_t i = 0; i < robots.size(); i++)
        {
            const float distance = murosim::robot::util::distanceToGoal(robots[i]->getPose(), robots[i]->getGoal());
            const float norm = std::max(0.0f, threshold_radius_ - distance) / threshold_radius_;
            output[i] = norm * positive_;
        }
    }
}

} // namespace murosimrl::reward::robot
