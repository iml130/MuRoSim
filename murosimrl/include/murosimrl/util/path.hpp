// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_UTIL_PATH_HPP_
#define MUROSIMRL_UTIL_PATH_HPP_

#include <Eigen/Core>
#include <vector>

namespace murosimrl::util::path
{

/// Walks along path_points and interpolates the point at exactly
/// target_distance from robot_pos. If the path is shorter than target_distance,
/// result clamps to the last path point (the goal), so consumers keep tracking
/// the goal near the end of an episode. Returns false only for an empty path.
/// Single owner of the walk-and-lerp logic shared by the subgoal observations
/// and the along-path rewards - their geometry must never diverge.
/// @param robot_pos Starting point of the walk, in world coordinates (metres).
/// @param path_points Ordered path waypoints, in world coordinates (metres).
/// @param target_distance Arc-length distance from robot_pos to interpolate at, in metres.
/// @param result Receives the interpolated point (or the goal, if the path is shorter).
/// @return False only if path_points is empty; true otherwise.
bool interpolateAlongPath(const Eigen::Vector2f &robot_pos,
                          const std::vector<Eigen::Vector2f> &path_points,
                          float target_distance,
                          Eigen::Vector2f &result);

} // namespace murosimrl::util::path

#endif // MUROSIMRL_UTIL_PATH_HPP_
