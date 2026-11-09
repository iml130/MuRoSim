// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_UTIL_GEOMETRY_HPP_
#define MUROSIMPROC_UTIL_GEOMETRY_HPP_

#include <assert.h>
#include "murosim/geometry/line.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/random/random_generator.hpp"

namespace murosimproc::util::geometry
{

/// Appends the four lines of the world border to `lines`, spanning `[0, world_size]` in both x and y (metres).
void addBorderLines(std::vector<murosim::geometry::Line> &lines, const float world_size);
/// Appends the four lines of the world border to `lines`, spanning `[0, world_size_x]` by `[0, world_size_y]` (metres).
void addBorderLines(std::vector<murosim::geometry::Line> &lines, const float world_size_x, const float world_size_y);
/// Appends the four lines of a rectangle connecting `point1` and `point2` (its long axis) to `lines`, inflated by `margin` on every side.
void addRectangleLinesFromPoints(std::vector<murosim::geometry::Line> &lines,  const Eigen::Vector2f point1, const Eigen::Vector2f point2, const float margin);
/// Returns the angle (radians) from `center` towards `point`.
float getOrientationToCenterPoint(const Eigen::Vector2f &point, const Eigen::Vector2f &center);
/// Randomly places non-overlapping circles inside `circle`, attempting `tries` random placements and keeping collision-free ones.
/// @param ratio_radius Fraction (clamped to [0.1, 1]) of `circle`'s radius that controls the minimum generated radius; generated radii are always at most half of `circle`'s radius.
/// @return The successfully placed circles (may be fewer than `tries` if placements collide).
std::vector<murosim::geometry::Circle> createCirclesInCircle(const murosim::geometry::Circle &circle, const float ratio_radius, const size_t tries, murosim::random::RandomGenerator &random_generator);
/// Same as the `tries`-overload with a default of 5 placement attempts.
std::vector<murosim::geometry::Circle> createCirclesInCircle(const murosim::geometry::Circle &circle, const float ratio_radius, murosim::random::RandomGenerator &random_generator);
/// Generates the outline of a random convex polygon inscribed in `circle`, as a closed loop of lines.
/// @param ratio_length Fraction (clamped to [0, 1]) of `circle`'s radius used as the minimum vertex distance from the center.
/// @param segments_min Minimum number of polygon vertices (clamped to at least 3).
/// @param segments_max Maximum number of polygon vertices (clamped to at least `segments_min`).
std::vector<murosim::geometry::Line> createPolygonLinesInCircle(const murosim::geometry::Circle &circle, float ratio_length, size_t segments_min, size_t segments_max, murosim::random::RandomGenerator &random_generator);
/// Same as the full overload with `segments_min = 3` and `segments_max = 8`.
std::vector<murosim::geometry::Line> createPolygonLinesInCircle(const murosim::geometry::Circle &circle, const float ratio_length, murosim::random::RandomGenerator &random_generator);
/// Randomly places non-overlapping circles inside `circle`, attempting up to `max_attempts_per_circle` placements per circle until `target_count` circles are placed.
/// @return The successfully placed circles (may be fewer than `target_count` if placements keep colliding).
std::vector<murosim::geometry::Circle> createCirclesInCircle(const murosim::geometry::Circle &circle, const float ratio_radius, const size_t target_count, const size_t max_attempts_per_circle, murosim::random::RandomGenerator &random_generator);
/// Generates the outline of a random axis-aligned-in-local-frame rectangle inscribed in `circle`, as a closed loop of lines.
std::vector<murosim::geometry::Line> createRectangleLinesInCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator);
/// Generates the outline of a random L-shaped hexagon inscribed in `circle`, as a closed loop of lines.
std::vector<murosim::geometry::Line> createLShapeLinesInCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator);

} // namespace murosimproc::util::geometry

#endif // MUROSIMPROC_UTIL_GEOMETRY_HPP_
