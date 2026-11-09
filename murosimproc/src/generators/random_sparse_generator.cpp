// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/generators/random_sparse_generator.hpp"
#include "murosimproc/obstacle/convert_shapes.hpp"
#include "murosimproc/util/spatial_grid.hpp"
#include "murosimproc/util/geometry.hpp"
#include "murosim/math/constants.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace murosimproc::generators
{

murosimproc::obstacle::ShapeDistribution RandomSparseGenerator::entropyToShapeDistribution(const RandomRange& entropy)
{
    // ShapeDistribution{circle, polygon, decompose}; min_polygon_radius keeps its default
    return {
        RandomRange(entropy.min(), entropy.max()),
        RandomRange((1.0f - entropy.max()) * 0.5f, (1.0f - entropy.min()) * 0.5f),
        RandomRange(entropy.min() * 0.5f, entropy.max() * 0.5f),
    };
}

RandomSparseGenerator::RandomSparseGenerator(
    size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    RandomRange obstacle_size,
    RandomRange placement_density_rate,
    murosimproc::obstacle::ShapeDistribution shape_distribution)
    : murosimproc::generators::Generator("random_sparse", min_node_count, node_radius, world_size, spacing)
    , obstacle_size_(obstacle_size)
    , shape_distribution_(shape_distribution)
    , placement_density_rate_(placement_density_rate)
{
    validateRandomSparseConfig();
}

RandomSparseGenerator::RandomSparseGenerator(
    size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    RandomRange obstacle_size,
    RandomRange placement_density_rate,
    RandomRange shape_entropy)
    : RandomSparseGenerator(min_node_count, node_radius, world_size, spacing, obstacle_size,
                            placement_density_rate, entropyToShapeDistribution(shape_entropy))
{
}

void RandomSparseGenerator::validateRandomSparseConfig() const
{
    // Validate obstacle scale
    if (obstacle_size_.min() <= 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "min_obstacle_size must be > 0 (got " +
            std::to_string(obstacle_size_.min()) + ")");
    }

    // Validate shape distribution (all fields must be >= 0)
    if (shape_distribution_.circle.min() < 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "shape_distribution.circle.min must be >= 0 (got " +
            std::to_string(shape_distribution_.circle.min()) + ")");
    }
    if (shape_distribution_.polygon.min() < 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "shape_distribution.polygon.min must be >= 0 (got " +
            std::to_string(shape_distribution_.polygon.min()) + ")");
    }
    if (shape_distribution_.decompose.min() < 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "shape_distribution.decompose.min must be >= 0 (got " +
            std::to_string(shape_distribution_.decompose.min()) + ")");
    }

    // Validate placement density rate
    if (placement_density_rate_.min() <= 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "min_placement_density_rate must be > 0 (got " +
            std::to_string(placement_density_rate_.min()) + ")");
    }

    // Validate world size constraint
    const float max_spacing = getSpacing().max();
    if (getWorldSize().min() <= max_spacing * 3.0f) [[unlikely]] {
        throw std::invalid_argument(
            "min_world_size (" + std::to_string(getWorldSize().min()) +
            ") must be > 3 * max_spacing (" +
            std::to_string(max_spacing) + ")");
    }
}

const RandomRange& RandomSparseGenerator::getObstacleSize() const
{
    return obstacle_size_;
}

const murosimproc::obstacle::ShapeDistribution& RandomSparseGenerator::getShapeDistribution() const
{
    return shape_distribution_;
}

const RandomRange& RandomSparseGenerator::getPlacementDensityRate() const
{
    return placement_density_rate_;
}

size_t RandomSparseGenerator::calculateMaxPlacementAttempts(float world_size, float placement_density_rate) const
{
    // Calculate attempts based on world size and density rate
    // Formula: attempts = world_size * density_rate * base_multiplier
    // This ensures larger worlds get proportionally more attempts
    constexpr float base_multiplier = 10.0f;  // Base attempts per unit

    const float attempts_float = world_size * placement_density_rate * base_multiplier;
    const size_t attempts = static_cast<size_t>(std::max(1.0f, attempts_float));

    return attempts;
}

void RandomSparseGenerator::generate(murosim::random::RandomGenerator::result_type seed, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, std::vector<murosim::navigation::Node> &nodes) const
{
    murosim::random::RandomGenerator random_generator(seed);

    const float world_size = getWorldSize()(random_generator);
    const float node_radius = getNodeRadius();
    const size_t min_nodes = getMinNodeCount();

    // Placement density
    const float placement_density_rate = placement_density_rate_(random_generator);
    const size_t max_placement_attempts = calculateMaxPlacementAttempts(world_size, placement_density_rate);

    // Obstacle scale range (relative to node_radius)
    const float min_obstacle_radius = obstacle_size_.min() * node_radius;
    const float max_obstacle_radius = obstacle_size_.max() * node_radius;

    // Grid cell size based on worst case: max obstacle radius + max half-spacing
    const float max_half_spacing = getSpacing().max() * 0.5f;
    const float max_placement_radius = max_obstacle_radius + max_half_spacing;
    murosimproc::util::SpatialGrid grid(2.0f * max_placement_radius);

    // Phase 1: Place obstacles first (consecutive-failure exit)
    // Each obstacle gets its own spacing sampled from the spacing RandomRange,
    // giving natural density variation while guaranteeing robot passability.
    std::uniform_real_distribution<float> distribution_obstacle_radius(min_obstacle_radius, max_obstacle_radius);

    std::vector<murosim::geometry::Circle> circle_obstacles;
    size_t consecutive_failures = 0;

    while (consecutive_failures < max_placement_attempts)
    {
        const float radius = distribution_obstacle_radius(random_generator);
        const float half_spacing = getSpacing()(random_generator) * 0.5f;
        const float obs_min_pos = radius;
        const float obs_max_pos = world_size - radius;

        if (obs_min_pos >= obs_max_pos) {
            consecutive_failures++;
            continue;
        }

        std::uniform_real_distribution<float> distribution_position(obs_min_pos, obs_max_pos);

        const murosim::geometry::Circle candidate = murosim::geometry::Circle(
            distribution_position(random_generator),
            distribution_position(random_generator),
            radius);

        // Inflate by per-obstacle half_spacing — guarantees robot can pass
        const murosim::geometry::Circle candidate_inflated = murosim::geometry::Circle(
            candidate.getPoint(),
            radius + half_spacing);

        if (grid.hasCollision(candidate_inflated))
        {
            consecutive_failures++;
        }
        else
        {
            circle_obstacles.push_back(candidate);
            grid.insert(candidate_inflated);
            consecutive_failures = 0;
        }
    }

    // Phase 2: Build a pool of candidate node positions, then select min_node_count
    // using farthest-point sampling for maximum spread.
    // Nodes are robot start/goal points — they only need to physically not overlap obstacles.
    const float node_border_margin = node_radius + max_half_spacing;
    const float node_min_pos = node_border_margin;
    const float node_max_pos = world_size - node_border_margin;

    if (node_min_pos >= node_max_pos) {
        return;  // World too small for any nodes
    }

    // Separate grid for candidate placement — inflate obstacles by max_half_spacing
    // so that nodes keep enough clearance from obstacle surfaces.
    // Without this, nodes can land right at an obstacle edge and end up inside
    // the grid map's buffer zone, preventing any FREE region from being created.
    murosimproc::util::SpatialGrid node_grid(2.0f * (max_obstacle_radius + max_half_spacing));
    for (const auto &obs : circle_obstacles)
    {
        node_grid.insert(murosim::geometry::Circle(
            obs.getPoint(), obs.getRadius() + max_half_spacing));
    }

    std::uniform_real_distribution<float> distribution_node(node_min_pos, node_max_pos);

    std::vector<murosim::geometry::Circle> candidate_pool;
    consecutive_failures = 0;

    while (consecutive_failures < max_placement_attempts)
    {
        const murosim::geometry::Circle candidate = murosim::geometry::Circle(
            distribution_node(random_generator),
            distribution_node(random_generator),
            node_radius);

        if (node_grid.hasCollision(candidate))
        {
            consecutive_failures++;
        }
        else
        {
            candidate_pool.push_back(candidate);
            node_grid.insert(candidate);
            consecutive_failures = 0;
        }
    }

    // Select min_node_count nodes from the pool using farthest-point sampling
    const size_t select_count = std::min(min_nodes, candidate_pool.size());
    std::vector<murosim::geometry::Circle> circle_nodes;
    circle_nodes.reserve(select_count);

    if (!candidate_pool.empty() && select_count > 0)
    {
        // Track which candidates are still available
        std::vector<bool> selected(candidate_pool.size(), false);
        // min-distance from each candidate to the nearest selected node
        std::vector<float> min_dist(candidate_pool.size(), std::numeric_limits<float>::max());

        // Pick first node randomly
        const size_t first_idx = std::uniform_int_distribution<size_t>(0, candidate_pool.size() - 1)(random_generator);
        selected[first_idx] = true;
        circle_nodes.push_back(candidate_pool[first_idx]);

        // Update distances from first selected node
        const auto &first_pt = candidate_pool[first_idx].getPoint();
        for (size_t i = 0; i < candidate_pool.size(); ++i)
        {
            if (!selected[i])
            {
                const auto &pt = candidate_pool[i].getPoint();
                const float dx = pt.x() - first_pt.x();
                const float dy = pt.y() - first_pt.y();
                min_dist[i] = dx * dx + dy * dy;
            }
        }

        // Greedily select the farthest candidate from all selected nodes
        for (size_t n = 1; n < select_count; ++n)
        {
            size_t best_idx = 0;
            float best_dist = -1.0f;

            for (size_t i = 0; i < candidate_pool.size(); ++i)
            {
                if (!selected[i] && min_dist[i] > best_dist)
                {
                    best_dist = min_dist[i];
                    best_idx = i;
                }
            }

            selected[best_idx] = true;
            circle_nodes.push_back(candidate_pool[best_idx]);

            // Update min distances with the newly selected node
            const auto &new_pt = candidate_pool[best_idx].getPoint();
            for (size_t i = 0; i < candidate_pool.size(); ++i)
            {
                if (!selected[i])
                {
                    const auto &pt = candidate_pool[i].getPoint();
                    const float dx = pt.x() - new_pt.x();
                    const float dy = pt.y() - new_pt.y();
                    const float d2 = dx * dx + dy * dy;
                    if (d2 < min_dist[i])
                    {
                        min_dist[i] = d2;
                    }
                }
            }
        }
    }

    // Phase 3: Convert to shapes (only if we have enough nodes)
    if (circle_nodes.size() >= 2)
    {
        murosimproc::util::geometry::addBorderLines(lines, world_size);

        for (const murosim::geometry::Circle &c : circle_nodes)
        {
            nodes.emplace_back(c, 0.0f, murosim::math::PI);
        }

        // Convert obstacle circles to varied shapes
        murosimproc::obstacle::convertShapes(
            circle_obstacles, shape_distribution_, min_obstacle_radius,
            random_generator, lines, circles);
    }
}

} // namespace murosimproc::generators
