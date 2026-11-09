// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/action_rate.hpp"
#include <murosim/robot/util.hpp>
#include <algorithm>

namespace murosimrl::reward::robot
{

ActionRate::ActionRate(const float threshold_x, const float threshold_y, const float threshold_angular, const float negative)
    : RewardRobot("action_rate")
    , threshold_x_(threshold_x)
    , threshold_y_(threshold_y)
    , threshold_angular_(threshold_angular)
    , negative_(negative)
{
}

void ActionRate::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    fillRewardActionRate(output, robots);
}

void ActionRate::fillRewardActionRate(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        float reward = 0.0f;

        if (0 < robots[i]->getStateSize())
        {
            const Eigen::Vector3f diff = (robots[i]->getStateAt(0).getActuatorCommandVelocity() - robots[i]->getStateAt(0).getActuatorVelocity()).cwiseAbs();

            reward += negative_ * std::max(diff.x() - threshold_x_, 0.0f);
            reward += negative_ * std::max(diff.y() - threshold_y_, 0.0f);
            reward += negative_ * std::max(diff.z() - threshold_angular_, 0.0f);
        }

        array[i] = reward;
    }
}

} // namespace murosimrl::reward::robot
