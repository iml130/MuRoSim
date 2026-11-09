// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/orientation_to_goal.hpp"
#include <murosim/robot/util.hpp>

#include <cmath>

namespace murosimrl::reward::robot
{

OrientationToGoal::OrientationToGoal(const float positive, const float negative)
    : RewardRobot("orientation_to_goal")
    , positive_(positive)
    , negative_(negative)
{
}

void OrientationToGoal::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    fillRewardOrientationToGoal(output, robots);
}

void OrientationToGoal::fillRewardOrientationToGoal(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        const Eigen::Vector2f orientation_to_goal = murosim::robot::util::orientationToGoal(robots[i]->getPose(), robots[i]->getGoal());

        // orientationToGoal returns the zero vector for a robot exactly on its
        // goal (no direction), which scores neutral here without a special case.
        const float angle_norm = orientation_to_goal.dot(Eigen::Vector2f(1.0f, 0.0f));

        const float reward = (angle_norm < 0.0f) ? -angle_norm * negative_ : angle_norm * positive_;

        array[i] = reward;
    }
}

} // namespace murosimrl::reward::robot
