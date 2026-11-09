// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_ON_GOAL_HPP_
#define MUROSIMRL_REWARD_ROBOT_ON_GOAL_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// Rewards being close to the goal: scales `positive` by how far inside a
/// radius the robot's current distance to goal is, normalized to [0, 1] (1
/// exactly on the goal, 0 at or beyond the radius). If threshold_radius is
/// positive, that radius is used directly; if it is <= 0, the robot's own
/// goal radius (Robot::getGoal().getGoalRadius(), the same radius the
/// simulation uses for its goal-reached check, so reward and termination
/// agree) is used instead, and the reward is zero when that radius is not
/// positive. Reads Robot::getPose()/getGoal().
class OnGoal : public RewardRobot
{
private:
    float threshold_radius_;
    float positive_;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param threshold_radius Radius (metres) the reward ramps up over; <= 0 uses each robot's own goal radius instead.
    /// @param positive Maximum reward, earned exactly on the goal.
    OnGoal(const float threshold_radius, const float positive);

    /// The configured threshold radius (<= 0 means "use the robot's own goal radius").
    float thresholdRadius() const { return threshold_radius_; }
    /// The configured maximum reward.
    float positive() const { return positive_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_ON_GOAL_HPP_
