// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/obstacle/place_obstacles.hpp"
#include "murosimproc/obstacle/convert_shapes.hpp"
#include "murosimproc/util/points.hpp"
#include "murosimproc/util/spatial_grid.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <random>

namespace murosimproc::obstacle
{

static constexpr size_t MAX_PLACEMENT_ATTEMPTS = 30;
static constexpr float HEX_JITTER_FRACTION = 0.3f;
static constexpr float RADIUS_REDUCTION_FACTOR = 0.7f;
static constexpr size_t MAX_RADIUS_REDUCTION_ROUNDS = 3;
static constexpr float MIN_RADIUS_FRACTION = 0.3f;

static std::vector<murosim::geometry::Circle> placeCircles(
    const Eigen::Vector2f &area_min,
    const Eigen::Vector2f &area_max,
    float density,
    float min_radius,
    float max_radius,
    murosim::random::RandomGenerator &rng)
{
    assert(area_min.x() < area_max.x());
    assert(area_min.y() < area_max.y());

    const Eigen::Vector2f region_size = area_max - area_min;
    const float region_area = region_size.x() * region_size.y();

    // Poisson-like object count
    const float expected_objects = region_area * density;
    std::uniform_real_distribution<float> unit_dist(0.0f, 1.0f);
    const size_t base_count = static_cast<size_t>(expected_objects);
    const float fractional = expected_objects - static_cast<float>(base_count);
    const size_t num_objects = base_count + (unit_dist(rng) < fractional ? 1 : 0);

    if (num_objects == 0)
    {
        return {};
    }

    // Edge case: region too small → single centered obstacle
    if (region_size.x() < 2.0f * min_radius || region_size.y() < 2.0f * min_radius)
    {
        const float min_dimension = std::min(region_size.x(), region_size.y());
        const float centered_radius = std::min(min_radius, min_dimension * 0.3f);
        if (centered_radius > 0.0f)
        {
            const Eigen::Vector2f center = (area_min + area_max) * 0.5f;
            return {murosim::geometry::Circle(center.x(), center.y(), centered_radius)};
        }
        return {};
    }

    std::uniform_real_distribution<float> radius_dist(min_radius, max_radius);

    // Spatial grid for O(1) collision checking
    murosimproc::util::SpatialGrid spatial_grid(2.0f * max_radius);

    // Hex grid + jitter candidate positions
    const float spacing = 1.0f / std::sqrt(std::max(density, 0.01f));
    const float jitter = spacing * HEX_JITTER_FRACTION;

    std::vector<Eigen::Vector2f> candidates;
    murosimproc::util::points::createRandomPointsRectangleHexagon(
        candidates, rng, area_min, area_max, spacing, jitter);

    std::shuffle(candidates.begin(), candidates.end(), rng);

    std::vector<murosim::geometry::Circle> placed;
    size_t candidate_idx = 0;

    for (size_t obj = 0; obj < num_objects; ++obj)
    {
        float radius = radius_dist(rng);
        bool success = false;

        // Try hex grid candidates first
        for (size_t attempt = 0;
             attempt < MAX_PLACEMENT_ATTEMPTS && !success && candidate_idx < candidates.size();
             ++attempt)
        {
            const Eigen::Vector2f &pos = candidates[candidate_idx++];

            // Bounds check — obstacle must fit within the area
            if (pos.x() - radius < area_min.x() ||
                pos.x() + radius > area_max.x() ||
                pos.y() - radius < area_min.y() ||
                pos.y() + radius > area_max.y())
            {
                continue;
            }

            const murosim::geometry::Circle candidate(pos.x(), pos.y(), radius);
            if (!spatial_grid.hasCollision(candidate))
            {
                spatial_grid.insert(candidate);
                placed.emplace_back(pos.x(), pos.y(), radius);
                success = true;
            }
        }

        // Adaptive radius reduction with uniform random fallback
        if (!success)
        {
            float reduced_radius = radius;
            for (size_t round = 0; round < MAX_RADIUS_REDUCTION_ROUNDS && !success; ++round)
            {
                reduced_radius *= RADIUS_REDUCTION_FACTOR;
                if (reduced_radius < MIN_RADIUS_FRACTION * min_radius)
                {
                    break;
                }

                const float x_min = area_min.x() + reduced_radius;
                const float x_max = area_max.x() - reduced_radius;
                const float y_min = area_min.y() + reduced_radius;
                const float y_max = area_max.y() - reduced_radius;

                if (x_min >= x_max || y_min >= y_max)
                {
                    break;
                }

                std::uniform_real_distribution<float> x_dist(x_min, x_max);
                std::uniform_real_distribution<float> y_dist(y_min, y_max);

                for (size_t attempt = 0; attempt < MAX_PLACEMENT_ATTEMPTS && !success; ++attempt)
                {
                    const float x = x_dist(rng);
                    const float y = y_dist(rng);
                    const murosim::geometry::Circle candidate(x, y, reduced_radius);

                    if (!spatial_grid.hasCollision(candidate))
                    {
                        spatial_grid.insert(candidate);
                        placed.emplace_back(x, y, reduced_radius);
                        success = true;
                    }
                }
            }
        }
    }

    return placed;
}

void placeObstacles(
    float density,
    float scale_min,
    float scale_max,
    const ShapeDistribution &shape,
    const Eigen::Vector2f &area_min,
    const Eigen::Vector2f &area_max,
    murosim::random::RandomGenerator &rng,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles)
{
    const Eigen::Vector2f region_size = area_max - area_min;
    const float min_dimension = std::min(region_size.x(), region_size.y());

    // Clamp radii to fit within the region
    const float min_radius = std::min(scale_min * 0.5f, min_dimension * 0.4f);
    const float max_radius = std::min(scale_max * 0.5f, min_dimension * 0.45f);

    if (min_radius <= 0.0f || max_radius <= 0.0f || max_radius < min_radius)
    {
        return;
    }

    // Place circular bounding regions
    std::vector<murosim::geometry::Circle> placed = placeCircles(
        area_min, area_max, density, min_radius, max_radius, rng);

    if (placed.empty())
    {
        return;
    }

    // Convert to varied shapes
    convertShapes(placed, shape, min_radius, rng, lines, circles);
}

} // namespace murosimproc::obstacle
