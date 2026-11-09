// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_ACTION_RATE_HPP_
#define MUROSIMRL_REWARD_ROBOT_ACTION_RATE_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// Penalizes large, sudden changes in commanded velocity: for each axis
/// (x, y, angular), the absolute difference between the latest commanded and
/// actual actuator velocity is compared against that axis's threshold, and
/// any excess beyond the threshold is multiplied by `negative` (expected to
/// be <= 0) and summed across axes. Zero if the robot has no recorded state
/// yet, or if all per-axis differences stay within their thresholds. Reads
/// the latest state's getActuatorCommandVelocity()/getActuatorVelocity().
class ActionRate : public RewardRobot
{
private:
    float threshold_x_;
    float threshold_y_;
    float threshold_angular_;
    float negative_;

    /// Write the per-robot action-rate penalty into `array`, in robot order.
    void fillRewardActionRate(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param threshold_x Tolerated absolute difference on the x axis before the penalty kicks in.
    /// @param threshold_y Tolerated absolute difference on the y axis before the penalty kicks in.
    /// @param threshold_angular Tolerated absolute difference on the angular axis before the penalty kicks in.
    /// @param negative Penalty scale applied to the excess beyond each threshold; expected to be <= 0.
    ActionRate(const float threshold_x, const float threshold_y, const float threshold_angular, const float negative);

    /// The configured x-axis threshold.
    float thresholdX() const { return threshold_x_; }
    /// The configured y-axis threshold.
    float thresholdY() const { return threshold_y_; }
    /// The configured angular-axis threshold.
    float thresholdAngular() const { return threshold_angular_; }
    /// The configured penalty scale.
    float negative() const { return negative_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_ACTION_RATE_HPP_
