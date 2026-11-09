// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/generators/graph_generator.hpp"
#include "murosimproc/graph/algorithm.hpp"
#include "murosimproc/graph/converter.hpp"
#include "murosimproc/graph/filter.hpp"
#include "murosimproc/graph/graph.hpp"
#include "murosimproc/util/geometry.hpp"
#include "murosimproc/util/points.hpp"
#include "murosim/math/constants.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

namespace murosimproc::generators
{

GraphGenerator::GraphGenerator(
    size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    RandomRange edge_length,
    RandomRange point_removal_rate,
    RandomRange edge_removal_rate,
    size_t tube_num_buckets,
    size_t tube_arc_segments,
    float tube_simplify_tolerance)
    : murosimproc::generators::Generator("graph", min_node_count, node_radius, world_size, spacing)
    , point_removal_rate_(point_removal_rate)
    , edge_removal_rate_(edge_removal_rate)
    , edge_length_(edge_length)
    , tube_num_buckets_(tube_num_buckets)
    , tube_arc_segments_(tube_arc_segments)
    , tube_simplify_tolerance_(tube_simplify_tolerance)
{
    validateGraphConfig();
}

void GraphGenerator::validateGraphConfig() const
{
    if (point_removal_rate_.min() < 0.0f || point_removal_rate_.max() > 1.0f) [[unlikely]] {
        throw std::invalid_argument(
            "point_removal_rate must be in [0, 1] (got [" +
            std::to_string(point_removal_rate_.min()) + ", " +
            std::to_string(point_removal_rate_.max()) + "])");
    }

    if (edge_removal_rate_.min() < 0.0f || edge_removal_rate_.max() > 1.0f) [[unlikely]] {
        throw std::invalid_argument(
            "edge_removal_rate must be in [0, 1] (got [" +
            std::to_string(edge_removal_rate_.min()) + ", " +
            std::to_string(edge_removal_rate_.max()) + "])");
    }

    if (edge_length_.min() <= 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "edge_length min must be > 0 (got " +
            std::to_string(edge_length_.min()) + ")");
    }

    const float tube_half_width = getSpacing().max() * 0.5f;
    if (edge_length_.min() < tube_half_width) [[unlikely]] {
        throw std::invalid_argument(
            "edge_length min (" + std::to_string(edge_length_.min()) +
            ") must be >= tube half-width (" + std::to_string(tube_half_width) +
            ") so corridors are not wider than they are long");
    }

    if (tube_num_buckets_ == 0) [[unlikely]] {
        throw std::invalid_argument("tube_num_buckets must be > 0");
    }

    if (tube_arc_segments_ == 0) [[unlikely]] {
        throw std::invalid_argument("tube_arc_segments must be > 0");
    }

    if (tube_simplify_tolerance_ < 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "tube_simplify_tolerance must be >= 0 (got " +
            std::to_string(tube_simplify_tolerance_) + ")");
    }

    // Margin is tube_max_spacing, inset on each side. The point generation area
    // is (world_size - 2 * margin). Must leave room for points with edge_length spacing.
    const float margin = std::max(getSpacing().min() * 0.5f * 1.05f, tube_half_width);
    const float min_available = getWorldSize().min() - 2.0f * margin;
    if (min_available <= 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "min_world_size (" + std::to_string(getWorldSize().min()) +
            ") must be > 2 * tube_max_spacing (" +
            std::to_string(margin) +
            ") so the point area is not empty");
    }

    if (min_available <= edge_length_.max()) [[unlikely]] {
        throw std::invalid_argument(
            "min_world_size - 2 * margin (" +
            std::to_string(min_available) +
            ") must be > max_edge_length (" +
            std::to_string(edge_length_.max()) +
            ") to fit enough points for a graph");
    }
}

void GraphGenerator::generate(
    murosim::random::RandomGenerator::result_type seed,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> & /*circles*/,
    std::vector<murosim::navigation::Node> &nodes
) const
{
    murosim::random::RandomGenerator rng(seed);

    const float world_size = getWorldSize()(rng);
    const float node_radius = getNodeRadius();
    const float tube_min_spacing = getSpacing().min() * 0.5f;
    const float tube_max_spacing = getSpacing().max() * 0.5f;
    const float point_keep_probability = 1.0f - point_removal_rate_(rng);
    const float edge_keep_probability = 1.0f - edge_removal_rate_(rng);

    // 1. Generate Poisson disk sampled points with margin to keep tubes inside the border
    //    Uses edge_length min/max as Poisson disk radii to control point distances
    const float margin = tube_max_spacing;
    std::vector<Eigen::Vector2f> points;
    murosimproc::util::points::createRandomPointsRectanglePoissonDisk(
        points, rng,
        Eigen::Vector2f(margin, margin),
        Eigen::Vector2f(world_size - margin, world_size - margin),
        edge_length_.min(),
        edge_length_.max());

    if (points.size() < 2) {
        return;
    }

    // 1b. Randomly thin points; keep fraction is point_keep_probability with
    //     a floor of min_node_count * 2 to ensure enough survive edge pruning
    const size_t min_keep = getMinNodeCount() * 2;
    const size_t desired_keep = std::max(
        min_keep,
        static_cast<size_t>(std::ceil(static_cast<float>(points.size()) * point_keep_probability)));
    murosimproc::util::points::choiceRandomPointsMinMax(
        points, rng,
        min_keep,
        desired_keep);

    // 2. Build Delaunay triangulation graph
    murosimproc::graph::Graph graph = murosimproc::graph::algorithm::makeDelaunayGraph(points);

    // 3. Random edge pruning for variety
    murosimproc::graph::filter::byEdgeChoiceRandom(graph, rng, edge_keep_probability);

    // 5. Convert graph to tube walls (tube half-width slightly larger than node radius)
    std::vector<murosim::geometry::Line> tube_lines =
        murosimproc::graph::converter::convertToTubes(
            graph,
            tube_min_spacing,
            tube_max_spacing,
            rng(),
            tube_num_buckets_,
            tube_arc_segments_,
            tube_simplify_tolerance_);

    lines.insert(lines.end(), std::make_move_iterator(tube_lines.begin()), std::make_move_iterator(tube_lines.end()));

    // 6. Add border
    murosimproc::util::geometry::addBorderLines(lines, world_size);

    // 7. Place navigation nodes at graph vertices. Vertices of degree 1 and 2
    //    are preferred — a robot parked on a junction (degree >= 3) blocks
    //    several tubes at once, so junctions only fill up to min_node_count.
    const std::vector<murosimproc::graph::Vertex> node_vertices =
        murosimproc::graph::filter::selectNodeVertices(graph, getMinNodeCount(), rng);

    for (const murosimproc::graph::Vertex v : node_vertices) {
        const auto &neighbors = murosimproc::graph::getNodeNeighbors(graph, v);
        const size_t degree = neighbors.size();

        const Eigen::Vector2f &pos = murosimproc::graph::getNodePosition(graph, v);
        float start_angle = 0.0f;
        float start_bias = murosim::math::PI;

        if (degree == 1) {
            // Dead-end: angle toward neighbor
            const Eigen::Vector2f &nbr = murosimproc::graph::getNodePosition(graph, *neighbors.begin());
            const Eigen::Vector2f dir = nbr - pos;
            start_angle = std::atan2(dir.y(), dir.x());
            start_bias = murosim::math::PI_4;
        } else if (degree == 2) {
            // Corridor/bend: bisector of the two neighbor directions
            const Eigen::Vector2f dir0 =
                (murosimproc::graph::getNodePosition(graph, *neighbors.begin()) - pos).normalized();
            const Eigen::Vector2f dir1 =
                (murosimproc::graph::getNodePosition(graph, *std::next(neighbors.begin())) - pos).normalized();
            const Eigen::Vector2f bisector = dir0 + dir1;
            const Eigen::Vector2f through =
                (bisector.squaredNorm() < 1e-6f) ? dir0 : bisector;
            start_angle = std::atan2(through.y(), through.x());
            start_bias = murosim::math::PI_4;
        }

        nodes.emplace_back(pos.x(), pos.y(), node_radius, start_angle, start_bias);
    }
}

} // namespace murosimproc::generators
