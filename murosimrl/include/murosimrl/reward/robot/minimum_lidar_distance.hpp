// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_MINIMUM_LIDAR_DISTANCE_HPP_
#define MUROSIMRL_REWARD_ROBOT_MINIMUM_LIDAR_DISTANCE_HPP_

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward::robot
{

/// Penalizes getting close to obstacles: takes each lidar's minimum reading,
/// clamped to its [range_min, range_max] (so -inf, too close, reads
/// range_min and +inf, no return, reads range_max), and if it is below
/// threshold_distance, scales `negative` (expected to be <= 0) by how far
/// below the threshold it is, normalized to [0, 1] (0 exactly at the
/// threshold, 1 at the lidar's range_min); the closest lidar decides. A lidar
/// whose range_min is at or above the threshold does not contribute. Zero
/// (reward disabled) if threshold_distance is not positive, since that would
/// otherwise divide by zero. Reads each of Robot::getLidars()'s current
/// getArrayLaserDistance().
class MinimumLidarDistance : public RewardRobot
{
private:
    float threshold_distance_;
    float negative_;

    /// Write the per-robot minimum-lidar-distance penalty into `array`, in robot order.
    void fillRewardMinimumLidarDistance(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param threshold_distance Distance (metres) below which the penalty kicks in; <= 0 disables the reward.
    /// @param negative Penalty scale applied at the lidar's range_min (or closer); expected to be <= 0.
    MinimumLidarDistance(const float threshold_distance, const float negative);

    /// The configured threshold distance, in metres.
    float thresholdDistance() const { return threshold_distance_; }
    /// The configured penalty scale.
    float negative() const { return negative_; }
};

} // namespace murosimrl::reward::robot

#endif // MUROSIMRL_REWARD_ROBOT_MINIMUM_LIDAR_DISTANCE_HPP_
