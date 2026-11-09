// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_UTIL_POLYGON_HPP_
#define MUROSIMPROC_UTIL_POLYGON_HPP_

#include <Eigen/Geometry>
#include <vector>
#include <murosim/geometry/line.hpp>

namespace murosimproc::util::polygon
{

/// Buffers a closed polygon outward (or inward, for negative `offset`) by `offset`, rounding corners, and returns the resulting outline(s) as line segments.
/// Implemented via `boost::geometry::buffer` with round join/end/point strategies; if the buffering splits or produces holes, every outer and inner ring is included.
/// @param polygon Vertices of the input polygon, in order (not required to be explicitly closed).
/// @param offset Signed buffer distance in the same units as `polygon` (metres); positive grows the polygon, negative shrinks it.
/// @param points_per_circle Number of points a full circle of the rounding is approximated with (Boost.Geometry's `join_round`; fewer than 4 are raised to 4); more points give smoother corners at the cost of more segments.
std::vector<murosim::geometry::Line> offsetPolygonRound(const std::vector<Eigen::Vector2f>& polygon, const float offset, const size_t points_per_circle);

} // namespace murosimproc::util::polygon

#endif // MUROSIMPROC_UTIL_POLYGON_HPP_
