// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_COLLISION_LINE_HPP_
#define MUROSIM_COLLISION_LINE_HPP_

#include "murosim/geometry/shape.hpp"
#include "murosim/geometry/world.hpp"
#include "murosim/simd/array_float.hpp"

namespace murosim::collision
{
    /// Tests line (already in the world frame) against every line and circle in world.
    /// @return true on any overlap.
    bool lineWithWorld(const murosim::geometry::Line& line, const murosim::geometry::World &world);
    /** \brief Line against a shape given in the same frame as the line. */
    bool lineWithShape(const murosim::geometry::Line& line, const murosim::geometry::Shape &shape);
    /// Tests line (in world frame) against shape, which is expressed relative to pose.
    /// Transforms the line into the shape's local frame rather than every shape primitive into the world frame.
    bool lineWithShape(const murosim::geometry::Line& line, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose);
    /// Tests a line segment against a circle for overlap (treating the circle as a solid disk).
    bool lineWithCircle(const murosim::geometry::Line& line, const murosim::geometry::Circle& circle);
    /// Tests a line segment against a circle for overlap, given as raw points/radius.
    bool lineWithCircle(const Eigen::Vector2f &line_point1, const Eigen::Vector2f& line_point2, const Eigen::Vector2f& circle_point, const float circle_radius);
    /// Tests a line segment against a circle for overlap, given as raw coordinates.
    bool lineWithCircle(const float x1, const float y1, const float x2, const float y2, const float cx, const float cy, const float cr);
    /// Tests line (p1x, p1y)-(p2x, p2y) against every circle in the parallel arrays
    /// (array_cx, array_cy, array_cr). @return true if any array circle overlaps.
    bool lineWithCircleArray(const float p1x, const float p1y, const float p2x, const float p2y, const murosim::simd::ArrayFloat& array_cx, const murosim::simd::ArrayFloat& array_cy, const murosim::simd::ArrayFloat& array_cr);
    /// Tests two line segments for intersection.
    bool lineWithLine(const murosim::geometry::Line& line1, const murosim::geometry::Line& line2);
    /// Tests two line segments, given as raw coordinates, for intersection.
    /// Parallel (including collinear-overlapping) segments are reported as non-intersecting.
    bool lineWithLine(const float x1, const float y1, const float x2, const float y2, const float x3, const float y3, const float x4, const float y4);
    /// Tests line (p1x, p1y)-(p2x, p2y) against every line segment in the parallel arrays
    /// (array_p3x, array_p3y)-(array_p4x, array_p4y). @return true if any array line intersects.
    bool lineWithLineArray(const float p1x, const float p1y, const float p2x, const float p2y, const murosim::simd::ArrayFloat& array_p3x, const murosim::simd::ArrayFloat& array_p3y, const murosim::simd::ArrayFloat& array_p4x, const murosim::simd::ArrayFloat& array_p4y);
}

#endif // MUROSIM_COLLISION_LINE_HPP_
