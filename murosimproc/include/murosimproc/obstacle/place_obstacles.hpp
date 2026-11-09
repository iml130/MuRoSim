// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_OBSTACLE_PLACE_OBSTACLES_HPP_
#define MUROSIMPROC_OBSTACLE_PLACE_OBSTACLES_HPP_

#include "murosimproc/obstacle/shape_distribution.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/random/random_generator.hpp"

#include <Eigen/Core>
#include <vector>

namespace murosimproc::obstacle
{

/**
 * @brief Place varied obstacles within a rectangular area.
 *
 * 1. Generates hex-grid + jitter candidate positions
 * 2. Places circular bounding regions with collision avoidance
 * 3. Converts bounding circles to varied shapes (circles, polygons)
 *
 * @param density Objects per unit area
 * @param scale_min Minimum obstacle radius
 * @param scale_max Maximum obstacle radius
 * @param shape Shape selection probabilities
 * @param area_min Bottom-left corner of the placement area
 * @param area_max Top-right corner of the placement area
 * @param rng Random number generator
 * @param lines Output: appended with line-based shapes
 * @param circles Output: appended with circle shapes
 */
void placeObstacles(
    float density,
    float scale_min,
    float scale_max,
    const ShapeDistribution &shape,
    const Eigen::Vector2f &area_min,
    const Eigen::Vector2f &area_max,
    murosim::random::RandomGenerator &rng,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles);

} // namespace murosimproc::obstacle

#endif // MUROSIMPROC_OBSTACLE_PLACE_OBSTACLES_HPP_
