// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/robot/util.hpp"
#include "murosim/math/angle.hpp"

#include <cmath>

namespace murosim::robot::util
{

Eigen::Vector2f orientationToGoal(const Eigen::Affine2f &pose, const murosim::navigation::Node &goal)
{
    return orientationToGoal(pose, goal.getPoint());
}

Eigen::Vector2f orientationToGoal(const Eigen::Affine2f &pose, const Eigen::Vector2f &goal)
{
    const Eigen::Vector2f offset = pose.rotation().inverse() * (goal - pose.translation());
    const float norm = offset.norm();
    // A robot exactly on its goal has no direction; normalized() would
    // divide by zero and feed NaN into every consumer (observations, rewards).
    if (norm <= 0.0f)
    {
        return Eigen::Vector2f::Zero();
    }
    return offset / norm;
}

float distanceToGoal(const Eigen::Affine2f &pose, const murosim::navigation::Node &goal)
{
    return (pose.translation() - goal.getPoint()).norm();
}

float distanceToGoal(const Eigen::Affine2f &pose, const Eigen::Vector2f &goal)
{
    return (pose.translation() - goal).norm();
}

Eigen::Vector3f deltaPose(const Eigen::Affine2f &current, const Eigen::Affine2f &previous)
{
    const Eigen::Affine2f delta = previous.inverse() * current;
    const Eigen::Vector2f translation = delta.translation();
    const float angle = murosim::math::rotationAngle(delta);

    return Eigen::Vector3f(translation.x(), translation.y(), angle);
}
} // namespace murosim::robot::util
