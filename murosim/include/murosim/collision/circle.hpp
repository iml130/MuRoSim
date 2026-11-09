// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_COLLISION_CIRCLE_HPP_
#define MUROSIM_COLLISION_CIRCLE_HPP_

#include "murosim/geometry/shape.hpp"
#include "murosim/geometry/world.hpp"
#include "murosim/simd/array_float.hpp"


namespace murosim::collision
{
    /// Tests circle (already in the world frame) against every line and circle in world.
    /// @return true on any overlap.
    bool circleWithWorld(const murosim::geometry::Circle& circle, const murosim::geometry::World &world);
    /** \brief Circle against a shape given in the same frame as the circle. */
    bool circleWithShape(const murosim::geometry::Circle& circle, const murosim::geometry::Shape &shape);
    /// Tests circle (in world frame) against shape, which is expressed relative to pose.
    /// Transforms the circle into the shape's local frame rather than every shape primitive into the world frame.
    bool circleWithShape(const murosim::geometry::Circle& circle, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose);
    /// Tests two circles for overlap (touching counts as overlap).
    bool circleWithCircle(const murosim::geometry::Circle& circle1, const murosim::geometry::Circle& circle2);
    /// Tests two circles, given as center coordinates and radii, for overlap.
    bool circleWithCircle(const float x1, const float y1, const float r1, const float x2, const float y2, const float r2);
    /// Tests circle (x1, y1, r1) against every circle in the parallel arrays (array_x2, array_y2, array_r2).
    /// @return true if any array circle overlaps.
    bool circleWithCircleArray(const float x1, const float y1, const float r1, const murosim::simd::ArrayFloat& array_x2, const murosim::simd::ArrayFloat& array_y2, const murosim::simd::ArrayFloat& array_r2);
    /// Tests a circle against a line segment for overlap (a segment fully inside the circle also counts).
    bool circleWithLine(const murosim::geometry::Circle& circle, const murosim::geometry::Line& line);
    /// Tests a circle against a line segment for overlap, given as raw points/radius.
    bool circleWithLine(const Eigen::Vector2f& circle_point, const float circle_radius, const Eigen::Vector2f& line_point1, const Eigen::Vector2f& line_point2);
    /// Tests a circle against a line segment for overlap, given as raw coordinates.
    bool circleWithLine(const float cx, const float cy, const float cr, const float x1, const float y1, const float x2, const float y2);
    /// Tests circle (cx, cy, cr) against every line segment in the parallel arrays
    /// (array_x1, array_y1)-(array_x2, array_y2). @return true if any array line overlaps.
    bool circleWithLineArray(const float cx, const float cy, const float cr, const murosim::simd::ArrayFloat& array_x1, const murosim::simd::ArrayFloat& array_y1, const murosim::simd::ArrayFloat& array_x2, const murosim::simd::ArrayFloat& array_y2);
}

#endif // MUROSIM_COLLISION_CIRCLE_HPP_
