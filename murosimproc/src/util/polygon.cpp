// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/util/polygon.hpp"

#include <boost/geometry.hpp>

namespace murosimproc::util::polygon
{

std::vector<murosim::geometry::Line> offsetPolygonRound(const std::vector<Eigen::Vector2f>& polygon, const float offset, const size_t points_per_circle)
{
    namespace bg = boost::geometry;

    using BoostPoint        = bg::model::d2::point_xy<float>;
    using BoostPolygon      = bg::model::polygon<BoostPoint>;
    using BoostMultiPolygon = bg::model::multi_polygon<BoostPolygon>;

    // Convert input polygon to Boost geometry format
    BoostPolygon boost_poly;
    for (const auto& p : polygon) {
        boost::geometry::append(boost_poly.outer(), BoostPoint(p.x(), p.y()));
    }
    boost::geometry::correct(boost_poly);

    // Perform offsetting using Boost geometry
    BoostMultiPolygon offset_polys;
    bg::strategy::buffer::distance_symmetric<float> distance_strategy(offset);
    bg::strategy::buffer::join_round join_strategy(points_per_circle);
    bg::strategy::buffer::end_round end_strategy(points_per_circle);
    bg::strategy::buffer::point_circle circle_strategy(points_per_circle);
    bg::strategy::buffer::side_straight side_strategy;

    bg::buffer(boost_poly, offset_polys,
               distance_strategy, side_strategy,
               join_strategy, end_strategy, circle_strategy);

    // Convert every ring (outer and holes) into line segments. Boost rings
    // repeat their first point at the end; a segment from that last point
    // back to the first would be a zero-length line, so it is skipped.
    const auto append_ring = [](std::vector<murosim::geometry::Line>& lines, const auto& ring) {
        const size_t count = (ring.size() > 1 && bg::equals(ring.front(), ring.back())) ? ring.size() - 1 : ring.size();
        for (size_t i = 0; i < count; ++i) {
            const Eigen::Vector2f p1(ring[i].x(), ring[i].y());
            const Eigen::Vector2f p2(ring[(i + 1) % count].x(), ring[(i + 1) % count].y());
            lines.emplace_back(p1, p2);
        }
    };
    std::vector<murosim::geometry::Line> result_lines;
    for (const auto& poly : offset_polys) {
        append_ring(result_lines, poly.outer());
        for (const auto& inner : poly.inners()) {
            append_ring(result_lines, inner);
        }
    }

    return result_lines;
}

} // namespace murosimproc::util::polygon
