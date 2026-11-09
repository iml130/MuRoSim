// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_ORIENTATION_TO_GOAL_HPP_
#define MUROSIMRL_REWARD_ROBOT_ORIENTATION_TO_GOAL_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// Rewards facing towards the goal: takes the x component (forward-axis
/// alignment, i.e. cosine of the angle between the robot's heading and the
/// direction to its goal) of the local-frame orientation-to-goal vector and
/// scales it by `positive` when facing towards the goal (component >= 0) or
/// by `negative` (expected to be <= 0) when facing away. A robot exactly on
/// its goal has a zero orientation vector and scores neutral (0). Reads
/// Robot::getPose()/getGoal() via murosim::robot::util::orientationToGoal.
class OrientationToGoal : public RewardRobot
{
private:
    float positive_;
    float negative_;

    /// Write the per-robot orientation-to-goal reward into `array`, in robot order.
    void fillRewardOrientationToGoal(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param positive Reward scale applied when facing towards the goal.
    /// @param negative Reward scale applied when facing away from the goal; expected to be <= 0.
    OrientationToGoal(const float positive, const float negative);

    /// The configured "facing towards" reward scale.
    float positive() const { return positive_; }
    /// The configured "facing away" penalty scale.
    float negative() const { return negative_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_ORIENTATION_TO_GOAL_HPP_
