// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ROBOT_UTIL_HPP_
#define MUROSIM_ROBOT_UTIL_HPP_

#include "murosim/navigation/node.hpp"

/// Free functions for pose/goal geometry shared by Robot, Simulation and the
/// murosimrl observation/reward implementations.
namespace murosim::robot::util
{
    /// Unit vector, in the robot's local frame, pointing from `pose` toward `goal`.
    /// @return Zero vector if the robot is exactly on the goal (avoids a NaN from normalizing a zero-length offset).
    Eigen::Vector2f orientationToGoal(const Eigen::Affine2f &pose, const murosim::navigation::Node &goal);
    /// @copydoc orientationToGoal(const Eigen::Affine2f&,const murosim::navigation::Node&)
    Eigen::Vector2f orientationToGoal(const Eigen::Affine2f &pose, const Eigen::Vector2f &goal);
    /// Euclidean distance (metres) between `pose`'s position and the goal.
    float distanceToGoal(const Eigen::Affine2f &pose, const murosim::navigation::Node &goal);
    /// @copydoc distanceToGoal(const Eigen::Affine2f&,const murosim::navigation::Node&)
    float distanceToGoal(const Eigen::Affine2f &pose, const Eigen::Vector2f &goal);
    /// Relative pose change from `previous` to `current`, expressed in `previous`'s local frame,
    /// as (dx, dy, dtheta) with translation in metres and rotation in radians.
    Eigen::Vector3f deltaPose(const Eigen::Affine2f &current, const Eigen::Affine2f &previous);

} // namespace murosim::robot::util

#endif // MUROSIM_ROBOT_UTIL_HPP_
