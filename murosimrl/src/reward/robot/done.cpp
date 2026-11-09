// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/done.hpp"

namespace murosimrl::reward::robot
{

Done::Done(const float positive_reached_goal, const float negative_collision_world, const float negative_collision_robot)
    : RewardRobot("done")
    , positive_reached_goal_(positive_reached_goal)
    , negative_collision_world_(negative_collision_world)
    , negative_collision_robot_(negative_collision_robot)
{
}

void Done::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    fillRewardDone(output, robots);
}

void Done::fillRewardDone(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        float reward = 0.0f;

        if (robots[i]->isFinished())
        {
            // Check first for reaching the goal, because another robot can cause a collision
            if (robots[i]->getStatusState() == murosim::robot::Robot::STATUS_STATE::REACHED_GOAL)
            {
                reward = positive_reached_goal_;
            }
            else
            {
                switch (robots[i]->getStatusCollision())
                {
                case murosim::robot::Robot::STATUS_COLLISION::WORLD:
                    reward = negative_collision_world_;
                    break;

                case murosim::robot::Robot::STATUS_COLLISION::ROBOT:
                    reward = negative_collision_robot_;
                    break;

                default:
                    break;
                }
            }
        }
        array[i] = reward;
    }
}

} // namespace murosimrl::reward::robot
