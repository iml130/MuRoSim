// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_DELTA_DISTANCE_TO_GOAL_HPP_
#define MUROSIMRL_REWARD_ROBOT_DELTA_DISTANCE_TO_GOAL_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// Rewards progress towards the goal: compares, between the two most recent
/// states, the Euclidean distance from the robot's pose to its goal. A
/// decrease (progress) is scaled by `positive`, an increase (regression) by
/// `negative` (expected to be <= 0). Zero if the robot has fewer than two
/// recorded states. Reads the two latest states' getPose()/getGoal().
class DeltaDistanceToGoal : public RewardRobot
{
private:
    float positive_;
    float negative_;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param positive Reward scale applied to a decrease in distance to goal (progress).
    /// @param negative Reward scale applied to an increase in distance to goal (regression); expected to be <= 0.
    DeltaDistanceToGoal(const float positive, const float negative);

    /// The configured progress reward scale.
    float positive() const { return positive_; }
    /// The configured regression penalty scale.
    float negative() const { return negative_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_DELTA_DISTANCE_TO_GOAL_HPP_
