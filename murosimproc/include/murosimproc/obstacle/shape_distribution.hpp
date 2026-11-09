// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_OBSTACLE_SHAPE_DISTRIBUTION_HPP_
#define MUROSIMPROC_OBSTACLE_SHAPE_DISTRIBUTION_HPP_

#include "murosimproc/generators/random_range.hpp"

namespace murosimproc::obstacle
{

/// Configuration for obstacle shape selection probabilities.
/// circle and polygon are relative weights (normalized internally).
/// Obstacles with radius below min_polygon_radius are always rendered as circles.
struct ShapeDistribution
{
    generators::RandomRange circle = 0.5f;
    generators::RandomRange polygon = 0.5f;
    generators::RandomRange decompose = 0.3f;
    float min_polygon_radius = 0.05f;
};

} // namespace murosimproc::obstacle

#endif // MUROSIMPROC_OBSTACLE_SHAPE_DISTRIBUTION_HPP_
