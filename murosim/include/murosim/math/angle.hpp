// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_MATH_ANGLE_HPP_
#define MUROSIM_MATH_ANGLE_HPP_

#include "murosim/math/constants.hpp"
#include <cmath>
#include <Eigen/Geometry>

/// Pure 2D angle helpers with no robot or navigation dependency.
namespace murosim::math
{
/// Rotation angle of a 2D pose in (-pi, pi].
inline float rotationAngle(const Eigen::Affine2f &pose)
{
    return Eigen::Rotation2Df(pose.rotation()).angle();
}

/// Unit vector along the x axis of a 2D pose (its heading).
inline Eigen::Vector2f rotationVector(const Eigen::Affine2f &pose)
{
    return pose.rotation() * Eigen::Vector2f(1.0f, 0.0f);
}

/// Signed angle (radians, via atan2) from `v1` to `v2`, in (-pi, pi].
inline float angleBetweenVectorAtan2(const Eigen::Vector2f &v1, const Eigen::Vector2f &v2)
{
    const float dot = v1.x() * v2.x() + v1.y() * v2.y();
    const float det = v1.x() * v2.y() - v1.y() * v2.x();
    return std::atan2(det, dot);
}

/// Signed shortest angular difference `current - previous`, wrapped into (-pi, pi] radians.
inline float deltaAngle(const float current, const float previous)
{
    const float delta = std::fmod(TWO_PI + current - previous, TWO_PI);
    return (delta > PI) ? delta - TWO_PI : delta;
}
} // namespace murosim::math

#endif // MUROSIM_MATH_ANGLE_HPP_
