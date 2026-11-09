// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/util/path.hpp"

namespace murosimrl::util::path
{

bool interpolateAlongPath(const Eigen::Vector2f &robot_pos,
                          const std::vector<Eigen::Vector2f> &path_points,
                          const float target_distance,
                          Eigen::Vector2f &result)
{
    if (path_points.empty())
    {
        return false;
    }

    float accumulated = 0.0f;
    Eigen::Vector2f prev = robot_pos;

    for (size_t p = 0; p < path_points.size(); p++)
    {
        const float segment = (path_points[p] - prev).norm();
        accumulated += segment;

        if (accumulated >= target_distance)
        {
            const float overshoot = accumulated - target_distance;
            const float t = (segment > 1e-6f) ? (1.0f - overshoot / segment) : 1.0f;
            result = prev + t * (path_points[p] - prev);
            return true;
        }

        prev = path_points[p];
    }

    // Path shorter than target - clamp to the goal (last path point)
    result = path_points.back();
    return true;
}

} // namespace murosimrl::util::path
