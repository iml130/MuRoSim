// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_DELTA_DISTANCE_ALONG_PATH_HPP_
#define MUROSIMRL_REWARD_ROBOT_DELTA_DISTANCE_ALONG_PATH_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// Rewards progress towards a lookahead point on the robot's planned path,
/// using the same walk-and-lerp geometry as SubgoalDirection
/// (util::path::interpolateAlongPath). Compares, between the two most recent
/// states, how the straight-line distance from the robot to the point
/// `lookahead_distance` ahead on the path changed: a decrease (progress) is
/// scaled by `positive`, an increase (regression) by `negative` (expected to
/// be <= 0). Zero if the robot has fewer than two recorded states, has no
/// path, or the lookahead point could not be found for either state. Reads
/// Robot::getPathPoints() and the two latest states' poses.
class DeltaDistanceAlongPath : public RewardRobot
{
private:
    float positive_;
    float negative_;
    float lookahead_distance_;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param positive Reward scale applied to a decrease in lookahead distance (progress).
    /// @param negative Reward scale applied to an increase in lookahead distance (regression); expected to be <= 0.
    /// @param lookahead_distance Arc-length distance (metres) ahead on the path to track. Clamped to >= 0.1.
    DeltaDistanceAlongPath(const float positive, const float negative, const float lookahead_distance);

    /// The configured progress reward scale.
    float positive() const { return positive_; }
    /// The configured regression penalty scale.
    float negative() const { return negative_; }
    /// The configured lookahead distance, in metres.
    float lookaheadDistance() const { return lookahead_distance_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_DELTA_DISTANCE_ALONG_PATH_HPP_
