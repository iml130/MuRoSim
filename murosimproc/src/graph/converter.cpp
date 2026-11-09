// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/graph/converter.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <cmath>
#include <random>

using namespace murosimproc::graph;

namespace murosimproc::graph::converter
{

namespace bg = boost::geometry;

using BoostPoint = bg::model::d2::point_xy<double>;
using BoostLinestring = bg::model::linestring<BoostPoint>;
using BoostMultiLinestring = bg::model::multi_linestring<BoostLinestring>;
using BoostPolygon = bg::model::polygon<BoostPoint>;
using BoostMultiPolygon = bg::model::multi_polygon<BoostPolygon>;

// Remove small fragment polygons produced by bg::union_ precision issues.
static void removeFragments(BoostMultiPolygon& mp, double min_area)
{
    mp.erase(
        std::remove_if(mp.begin(), mp.end(),
            [min_area](const BoostPolygon& p) {
                return std::abs(bg::area(p)) < min_area;
            }),
        mp.end());
}

// Correct and validate polygons before union.
static void sanitize(BoostMultiPolygon& mp)
{
    bg::correct(mp);
}

std::vector<murosim::geometry::Line> convertToTubes(
    const Graph& graph,
    float min_spacing,
    float max_spacing,
    murosim::random::RandomGenerator::result_type seed,
    size_t num_buckets,
    size_t arc_segments,
    float simplify_tolerance)
{
    std::vector<murosim::geometry::Line> result;

    if (getNodeSize(graph) == 0 || num_buckets == 0)
    {
        return result;
    }

    murosim::random::RandomGenerator rng(seed);
    std::uniform_int_distribution<size_t> bucket_dist(0, num_buckets - 1);

    std::vector<BoostMultiLinestring> buckets(num_buckets);
    std::vector<double> bucket_spacing(num_buckets);

    for (size_t i = 0; i < num_buckets; ++i)
    {
        // A single bucket uses the midpoint so max_spacing still has effect.
        double t = (num_buckets == 1) ? 0.5 : static_cast<double>(i) / static_cast<double>(num_buckets - 1);
        bucket_spacing[i] = static_cast<double>(min_spacing) + t * static_cast<double>(max_spacing - min_spacing);
    }

    // Assign each edge to a random bucket
    for (const auto& edge : getEdges(graph))
    {
        const Eigen::Vector2f& pos_u = getNodePosition(graph, edge.source);
        const Eigen::Vector2f& pos_v = getNodePosition(graph, edge.target);

        BoostLinestring line;
        bg::append(line, BoostPoint(static_cast<double>(pos_u.x()), static_cast<double>(pos_u.y())));
        bg::append(line, BoostPoint(static_cast<double>(pos_v.x()), static_cast<double>(pos_v.y())));

        size_t bucket_idx = bucket_dist(rng);
        buckets[bucket_idx].push_back(std::move(line));
    }

    // Buffer strategies
    bg::strategy::buffer::side_straight side_strategy;
    bg::strategy::buffer::join_round join_strategy(arc_segments);
    bg::strategy::buffer::end_round end_strategy(arc_segments);
    bg::strategy::buffer::point_circle point_strategy(arc_segments);

    BoostMultiPolygon all_buffers;

    const double fragment_area = static_cast<double>(min_spacing) * static_cast<double>(min_spacing);

    for (size_t i = 0; i < num_buckets; ++i)
    {
        if (buckets[i].empty())
        {
            continue;
        }

        bg::strategy::buffer::distance_symmetric<double> distance_strategy(bucket_spacing[i]);

        BoostMultiPolygon bucket_buffer;
        bg::buffer(buckets[i], bucket_buffer,
                   distance_strategy, side_strategy,
                   join_strategy, end_strategy, point_strategy);

        sanitize(bucket_buffer);

        // Union with accumulated buffers
        BoostMultiPolygon temp;
        bg::union_(all_buffers, bucket_buffer, temp);
        sanitize(temp);
        removeFragments(temp, fragment_area);
        all_buffers = std::move(temp);
    }

    // Final cleanup
    removeFragments(all_buffers, fragment_area);

    // Simplify if tolerance > 0
    if (simplify_tolerance > 0.0f)
    {
        BoostMultiPolygon simplified;
        bg::simplify(all_buffers, simplified, simplify_tolerance);
        all_buffers = std::move(simplified);
    }

    // Convert buffered polygons to lines
    // Boost's buffer output repeats a point where an arc meets a straight
    // side; a segment between two such points would be a zero-length line,
    // which the collision code cannot normalise, so those are skipped.
    const auto append_ring = [&result](const auto& ring)
    {
        for (size_t i = 0; i + 1 < ring.size(); ++i)
        {
            const Eigen::Vector2f p1(static_cast<float>(bg::get<0>(ring[i])), static_cast<float>(bg::get<1>(ring[i])));
            const Eigen::Vector2f p2(static_cast<float>(bg::get<0>(ring[i + 1])),
                                     static_cast<float>(bg::get<1>(ring[i + 1])));
            if (p1 != p2)
            {
                result.emplace_back(p1, p2);
            }
        }
    };
    for (const auto& poly : all_buffers)
    {
        append_ring(poly.outer());
        for (const auto& inner : poly.inners())
        {
            append_ring(inner);
        }
    }

    return result;
}


} // namespace murosimproc::graph::converter
