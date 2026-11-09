// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/minimum_lidar_distance.hpp"
#include <algorithm>

namespace murosimrl::reward::robot
{

MinimumLidarDistance::MinimumLidarDistance(const float threshold_distance, const float negative)
    : RewardRobot("minimum_lidar_distance")
    , threshold_distance_(threshold_distance)
    , negative_(negative)
{
}

void MinimumLidarDistance::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    fillRewardMinimumLidarDistance(output, robots);
}

void MinimumLidarDistance::fillRewardMinimumLidarDistance(float* array, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    // A non-positive threshold would divide by zero below (0/0 = NaN);
    // treat it as "reward disabled" like ForwardDriving does.
    if (threshold_distance_ <= 0.0f)
    {
        std::fill(array, array + robots.size(), 0.0f);
        return;
    }

    for (size_t i = 0; i < robots.size(); i++)
    {
        const murosim::robot::Robot &robot = *robots[i];

        // Per lidar, the nearest reading is clamped to the sensor range (-inf,
        // too close, reads range_min; +inf, no return, reads range_max) and
        // scaled so the threshold maps to 0 and range_min to 1. The lidar
        // closest to an obstacle decides.
        float norm = 0.0f;
        for (const murosim::sensor::Lidar &lidar : robot.getLidars())
        {
            const float range_min = lidar.getRangeMin();

            // no reading can lie below a threshold at or under range_min
            if (threshold_distance_ <= range_min)
            {
                continue;
            }

            const float distance = lidar.clampDistance(lidar.getArrayLaserDistance().min());
            norm = std::max(norm, std::clamp((threshold_distance_ - distance) / (threshold_distance_ - range_min), 0.0f, 1.0f));
        }

        array[i] = norm * negative_;
    }
}

} // namespace murosimrl::reward::robot
