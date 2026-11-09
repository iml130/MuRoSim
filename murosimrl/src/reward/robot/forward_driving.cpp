// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/forward_driving.hpp"
#include <algorithm>

namespace murosimrl::reward::robot
{

ForwardDriving::ForwardDriving(const float positive, const float threshold_y_velocity)
    : RewardRobot("forward_driving")
    , positive_(positive)
    , threshold_y_velocity_(threshold_y_velocity)
{
}

void ForwardDriving::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    fillRewardForwardDriving(output, robots);
}

void ForwardDriving::fillRewardForwardDriving(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        float reward = 0.0f;
        const Eigen::Vector3f velocity_max = robots[i]->getActuatorVelocityMax();
        const Eigen::Vector3f velocity = robots[i]->getActuatorCommandVelocity();

        if (0.0f < threshold_y_velocity_ && 0.0f < velocity_max.x())
        {
            const float scale_velocity_x = std::max(0.0f, velocity.x()) / velocity_max.x();
            const float scale_velocity_y = std::max(0.0f, threshold_y_velocity_ - std::abs(velocity.y())) / threshold_y_velocity_;
            reward = scale_velocity_x * scale_velocity_y * positive_;
        }

        array[i] = reward;
    }
}

} // namespace murosimrl::reward::robot
