// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_FORWARD_DRIVING_HPP_
#define MUROSIMRL_REWARD_ROBOT_FORWARD_DRIVING_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// Rewards driving forward with low sideways velocity: scales `positive` by
/// the fraction of max forward (x) velocity commanded (clamped to >= 0, so
/// reverse motion earns nothing) times the fraction of threshold_y_velocity
/// NOT used sideways (clamped to >= 0, so exceeding the threshold earns
/// nothing). Zero if threshold_y_velocity is not positive or the robot's max
/// x velocity is 0 (disabling the reward, e.g. for non-forward-driving
/// actuators). Reads Robot::getActuatorVelocityMax()/getActuatorCommandVelocity().
class ForwardDriving : public RewardRobot
{
private:
    float positive_;
    float threshold_y_velocity_;

    /// Write the per-robot forward-driving reward into `array`, in robot order.
    void fillRewardForwardDriving(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param positive Maximum reward, earned at full forward velocity with zero sideways velocity.
    /// @param threshold_y_velocity Sideways velocity magnitude at or above which the reward drops to zero; <= 0 disables the reward.
    ForwardDriving(const float positive, const float threshold_y_velocity);

    /// The configured maximum reward.
    float positive() const { return positive_; }
    /// The configured sideways-velocity threshold.
    float thresholdYVelocity() const { return threshold_y_velocity_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_FORWARD_DRIVING_HPP_
