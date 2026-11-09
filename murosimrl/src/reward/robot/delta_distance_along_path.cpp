// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/robot/delta_distance_along_path.hpp"
#include "murosimrl/util/path.hpp"

#include <algorithm>
#include <cmath>

namespace murosimrl::reward::robot
{

DeltaDistanceAlongPath::DeltaDistanceAlongPath(const float positive, const float negative, const float lookahead_distance)
    : RewardRobot("delta_distance_along_path")
    , positive_(positive)
    , negative_(negative)
    , lookahead_distance_(std::max(0.1f, lookahead_distance))
{
}

void DeltaDistanceAlongPath::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        float reward = 0.0f;
        const auto& path_points = robots[i]->getPathPoints();

        if (!path_points.empty() && 1 < robots[i]->getStateSize())
        {
            const murosim::robot::State &current = robots[i]->getStateAt(0);
            const murosim::robot::State &previous = robots[i]->getStateAt(1);

            const Eigen::Vector2f pos_current = current.getPose().translation();
            const Eigen::Vector2f pos_previous = previous.getPose().translation();

            Eigen::Vector2f lookahead_current, lookahead_previous;
            const bool found_current = murosimrl::util::path::interpolateAlongPath(pos_current, path_points, lookahead_distance_, lookahead_current);
            const bool found_previous = murosimrl::util::path::interpolateAlongPath(pos_previous, path_points, lookahead_distance_, lookahead_previous);

            if (found_current && found_previous)
            {
                const float distance_current = (pos_current - lookahead_current).norm();
                const float distance_previous = (pos_previous - lookahead_previous).norm();

                const float delta = distance_previous - distance_current;

                // -delta is the regression magnitude; negative_ holds a
                // negative value, same convention as DeltaDistanceToGoal.
                reward = (delta < 0.0f) ? -delta * negative_ : delta * positive_;
            }
        }

        output[i] = reward;
    }
}

} // namespace murosimrl::reward::robot
