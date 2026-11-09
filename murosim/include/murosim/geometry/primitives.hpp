// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GEOMETRY_PRIMITIVES_HPP_
#define MUROSIM_GEOMETRY_PRIMITIVES_HPP_

#include "murosim/geometry/shape.hpp"
#include <cmath>
#include <ostream>

namespace murosim::geometry::primitives
{

/// Builds the conventional 2D pose transform: rotate about the origin by
/// angle radians, then translate by (x, y) (i.e. T * R). This is the pose
/// convention used consistently by Circle/Line/Shape::transform(x, y, angle).
inline Eigen::Affine2f pose(const float x, const float y, const float angle)
{
    return Eigen::Translation2f(x, y) * Eigen::Rotation2Df(angle);
}

/// Same as pose(x, y, angle) but taking a pre-built rotation.
inline Eigen::Affine2f pose(const float x, const float y, const Eigen::Rotation2Df angle)
{
    return Eigen::Translation2f(x, y) * angle;
}

/// Same as pose(x, y, angle) but taking the translation as a vector.
inline Eigen::Affine2f pose(const Eigen::Vector2f translation, const float angle)
{
    return Eigen::Translation2f(translation.x(), translation.y()) * Eigen::Rotation2Df(angle);
}

/// Same as pose(x, y, angle) but taking the translation as a vector and the rotation pre-built.
inline Eigen::Affine2f pose(const Eigen::Vector2f translation, const Eigen::Rotation2Df angle)
{
    return Eigen::Translation2f(translation.x(), translation.y()) * angle;
}

/// Same as pose(x, y, angle) but taking the translation as an Eigen::Translation2f.
inline Eigen::Affine2f pose(const Eigen::Translation2f translation, const float angle)
{
    return translation * Eigen::Rotation2Df(angle);
}

/// Same as pose(x, y, angle) but taking both translation and rotation pre-built.
inline Eigen::Affine2f pose(const Eigen::Translation2f translation, const Eigen::Rotation2Df angle)
{
    return translation * angle;
}

/// Builds a square shape of the given side length, centered on the origin,
/// as four axis-aligned lines.
inline Shape square(const float length)
{
    Shape shape;
    const float half = 0.5f * length;
    shape.addLine(half, half, -half, half);
    shape.addLine(-half, half, -half, -half);
    shape.addLine(-half, -half, half, -half);
    shape.addLine(half, -half, half, half);
    return shape;
}

/// Builds a square shape of the given side length, centered on the origin,
/// then applies pose.
inline Shape square(const float length, const Eigen::Affine2f &pose)
{
    Shape shape = square(length);
    shape.transform(pose);
    return shape;
}

/// Builds a rectangle shape of the given width and height, centered on the
/// origin, as four axis-aligned lines.
inline Shape rectangle(const float width, const float height)
{
    Shape shape;
    const float half_width = 0.5f * width;
    const float half_height = 0.5f * height;
    shape.addLine(half_width, half_height, -half_width, half_height);
    shape.addLine(-half_width, half_height, -half_width, -half_height);
    shape.addLine(-half_width, -half_height, half_width, -half_height);
    shape.addLine(half_width, -half_height, half_width, half_height);
    return shape;
}

/// Builds a rectangle shape of the given width and height, centered on the
/// origin, then applies pose.
inline Shape rectangle(const float width, const float height, const Eigen::Affine2f &pose)
{
    Shape shape = rectangle(width, height);
    shape.transform(pose);
    return shape;
}

/// Builds an equilateral triangle shape of the given side length, centered on
/// its centroid, as three lines with one vertex pointing along +y.
inline Shape triangle(const float length)
{
    Shape shape;

    const float height = std::sqrt(3.0f) * 0.5f * length;
    const float base = -1.0f / 3.0f * height;
    const float half = 0.5f * length;
    Eigen::Vector2f p1 = Eigen::Vector2f(0.0f, 2.0f / 3.0f * height);
    Eigen::Vector2f p2 = Eigen::Vector2f(-half, base);
    Eigen::Vector2f p3 = Eigen::Vector2f(half, base);
    shape.addLine(p1, p2);
    shape.addLine(p2, p3);
    shape.addLine(p3, p1);
    return shape;
}

/// Builds an equilateral triangle shape of the given side length, centered on
/// its centroid, then applies pose.
inline Shape triangle(const float length, const Eigen::Affine2f &pose)
{
    Shape shape = triangle(length);
    shape.transform(pose);
    return shape;
}

} // namespace murosim::geometry::primitives

#endif  // MUROSIM_GEOMETRY_PRIMITIVES_HPP_
