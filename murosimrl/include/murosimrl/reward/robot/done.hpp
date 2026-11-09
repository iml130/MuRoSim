// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_DONE_HPP_
#define MUROSIMRL_REWARD_ROBOT_DONE_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// One-shot terminal reward for a robot's episode outcome. Zero unless the
/// robot has finished (Robot::isFinished()); a finished robot receives
/// positive_reached_goal if it reached its goal (checked before collision,
/// since another robot can cause a collision at goal arrival), otherwise
/// negative_collision_world or negative_collision_robot depending on
/// Robot::getStatusCollision(), or zero if finished for another reason.
/// Reads Robot::isFinished()/getStatusState()/getStatusCollision().
class Done : public RewardRobot
{
private:
    float positive_reached_goal_;
    float negative_collision_world_;
    float negative_collision_robot_;

    /// Write the per-robot terminal reward into `array`, in robot order.
    void fillRewardDone(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param positive_reached_goal Reward given when the robot's episode ended by reaching its goal.
    /// @param negative_collision_world Reward given when the episode ended by colliding with the world (expected to be <= 0).
    /// @param negative_collision_robot Reward given when the episode ended by colliding with another robot (expected to be <= 0).
    Done(const float positive_reached_goal, const float negative_collision_world, const float negative_collision_robot);

    /// The configured goal-reached reward.
    float positiveReachedGoal() const { return positive_reached_goal_; }
    /// The configured world-collision penalty.
    float negativeCollisionWorld() const { return negative_collision_world_; }
    /// The configured robot-collision penalty.
    float negativeCollisionRobot() const { return negative_collision_robot_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_DONE_HPP_
