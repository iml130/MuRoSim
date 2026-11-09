// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/obstacle/convert_shapes.hpp"
#include "murosimproc/util/geometry.hpp"

#include <iterator>
#include <random>

namespace murosimproc::obstacle
{

/// Minimum radius multiplier for decompose eligibility
static constexpr float DECOMPOSE_THRESHOLD = 1.5f;

void convertShapes(
    const std::vector<murosim::geometry::Circle> &placed,
    const ShapeDistribution &shape,
    float min_radius,
    murosim::random::RandomGenerator &rng,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles)
{
    // Sample fixed values from the shape distribution ranges
    const float circle_weight = shape.circle(rng);
    const float polygon_weight = shape.polygon(rng);
    const float decompose_prob = shape.decompose(rng);

    // Normalize to get circle probability
    const float weight_sum = circle_weight + polygon_weight;
    const float circle_prob = (weight_sum > 0.0f) ? (circle_weight / weight_sum) : 0.5f;

    std::uniform_real_distribution<float> unit_dist(0.0f, 1.0f);

    for (const auto &obstacle : placed)
    {
        // Decompose: split bounding circle into sub-circles, then render each as circle or polygon
        const bool decompose = unit_dist(rng) < decompose_prob &&
            obstacle.getRadius() > min_radius * DECOMPOSE_THRESHOLD;

        // Collect bounding circles to convert (1 original or N sub-circles)
        std::vector<murosim::geometry::Circle> bounds;
        if (decompose)
        {
            const size_t target_count = std::uniform_int_distribution<size_t>(2, 5)(rng);
            constexpr size_t max_attempts_per_circle = 10;
            bounds = murosimproc::util::geometry::createCirclesInCircle(
                obstacle,
                unit_dist(rng) * 0.5f + 0.3f,
                target_count,
                max_attempts_per_circle,
                rng);
        }

        // Each sub-shape independently rolls circle vs polygon
        auto convertOne = [&](const murosim::geometry::Circle &b) {
            if (b.getRadius() < shape.min_polygon_radius || unit_dist(rng) < circle_prob)
            {
                circles.push_back(b);
            }
            else
            {
                std::vector<murosim::geometry::Line> polygon_lines =
                    murosimproc::util::geometry::createPolygonLinesInCircle(
                        b, unit_dist(rng) * 0.4f + 0.5f, rng);
                lines.insert(lines.end(),
                    std::make_move_iterator(polygon_lines.begin()),
                    std::make_move_iterator(polygon_lines.end()));
            }
        };

        if (decompose)
        {
            for (const auto &b : bounds)
                convertOne(b);
        }
        else
        {
            convertOne(obstacle);
        }
    }
}

} // namespace murosimproc::obstacle
