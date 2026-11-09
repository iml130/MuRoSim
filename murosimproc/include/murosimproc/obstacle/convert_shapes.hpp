// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_OBSTACLE_CONVERT_SHAPES_HPP_
#define MUROSIMPROC_OBSTACLE_CONVERT_SHAPES_HPP_

#include "murosimproc/obstacle/shape_distribution.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/random/random_generator.hpp"

#include <vector>

namespace murosimproc::obstacle
{

/**
 * @brief Convert pre-placed bounding circles into varied shapes.
 *
 * Each circle is converted to either a circle or polygon based on shape weights.
 * Decompose probability controls whether circles are split into sub-shapes first.
 *
 * @param placed Circular bounding regions to convert
 * @param shape Shape selection probabilities
 * @param min_radius Minimum obstacle radius (for decompose eligibility)
 * @param rng Random number generator
 * @param lines Output: appended with line-based shapes
 * @param circles Output: appended with circle shapes
 */
void convertShapes(
    const std::vector<murosim::geometry::Circle> &placed,
    const ShapeDistribution &shape,
    float min_radius,
    murosim::random::RandomGenerator &rng,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles);

} // namespace murosimproc::obstacle

#endif // MUROSIMPROC_OBSTACLE_CONVERT_SHAPES_HPP_
