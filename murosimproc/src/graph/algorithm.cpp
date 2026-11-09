// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/graph/algorithm.hpp"

#include <boost/polygon/voronoi.hpp>
#include <unordered_set>
#include <limits>
#include <algorithm>

namespace
{
    // Boost.Polygon requires integer coordinates
    // We scale floating-point coordinates to integers for precision
    constexpr double SCALE_FACTOR = 1e6;

    struct Point
    {
        int64_t x;
        int64_t y;

        Point(const int64_t point_x = 0, const int64_t point_y = 0) : x(point_x), y(point_y) {}
    };

    // Hash function for point deduplication
    struct PointHash
    {
        size_t operator()(const Point& p) const noexcept
        {
            return std::hash<int64_t>{}(p.x) ^ (std::hash<int64_t>{}(p.y) << 32);
        }
    };

    struct PointEqual
    {
        bool operator()(const Point& a, const Point& b) const noexcept
        {
            return a.x == b.x && a.y == b.y;
        }
    };

    // Hash function for edge deduplication
    struct EdgeHash
    {
        size_t operator()(const std::pair<size_t, size_t>& edge) const noexcept
        {
            return std::hash<size_t>{}(edge.first) ^ (std::hash<size_t>{}(edge.second) << 1);
        }
    };

    // Create canonical edge (smaller index first)
    std::pair<size_t, size_t> makeEdge(size_t a, size_t b)
    {
        return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    }
} // anonymous namespace

// Register Point with Boost.Polygon geometry concept
namespace boost::polygon
{
    template <>
    struct geometry_concept<Point>
    {
        typedef point_concept type;
    };

    template <>
    struct point_traits<Point>
    {
        typedef int64_t coordinate_type;

        static inline coordinate_type get(const Point& point, orientation_2d orient)
        {
            return (orient == HORIZONTAL) ? point.x : point.y;
        }
    };
} // namespace boost::polygon

namespace murosimproc::graph::algorithm
{

Graph makeDelaunayGraph(const std::vector<Eigen::Vector2f> &points)
{
    Graph graph;

    if (points.size() < 2)
    {
        return graph;
    }

    // Find bounding box for normalization
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& point : points)
    {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
    }

    const float extent = std::max(max_x - min_x, max_y - min_y);
    if (extent < 1e-10f)
    {
        // All points are nearly coincident
        return graph;
    }

    // Convert to scaled integer coordinates and filter duplicates
    std::vector<Point> scaled_points;
    std::vector<Eigen::Vector2f> filtered_positions;
    scaled_points.reserve(points.size());
    filtered_positions.reserve(points.size());

    std::unordered_set<Point, PointHash, PointEqual> seen_coords;
    seen_coords.reserve(points.size());

    for (const auto& point : points)
    {
        // Normalize to [0, 1] range then scale to integer
        const double nx = static_cast<double>((point.x() - min_x) / extent);
        const double ny = static_cast<double>((point.y() - min_y) / extent);
        const Point scaled(static_cast<int64_t>(nx * SCALE_FACTOR), static_cast<int64_t>(ny * SCALE_FACTOR));

        if (seen_coords.insert(scaled).second)
        {
            scaled_points.push_back(scaled);
            filtered_positions.push_back(point);
        }
    }

    if (filtered_positions.size() < 2)
    {
        return graph;
    }

    // Add all nodes first
    for (const auto& pos : filtered_positions)
    {
        addNode(graph, pos);
    }

    // Special case: exactly 2 points - just connect them
    if (filtered_positions.size() == 2)
    {
        addBidirectionalEdge(graph, 0, 1);
        return graph;
    }

    // Construct Voronoi diagram
    boost::polygon::voronoi_diagram<double> vd;
    boost::polygon::construct_voronoi(scaled_points.begin(), scaled_points.end(), &vd);

    // Extract Delaunay edges from Voronoi diagram
    // Each Voronoi edge separates two cells; the cell indices are the Delaunay edge endpoints
    std::unordered_set<std::pair<size_t, size_t>, EdgeHash> edges;

    for (const auto& edge : vd.edges())
    {
        // Each edge has a cell and a twin edge with another cell
        // We only process primary edges (avoid duplicates from twin)
        if (edge.is_primary())
        {
            const auto* cell1 = edge.cell();
            const auto* cell2 = edge.twin()->cell();

            if (cell1 && cell2)
            {
                const size_t idx1 = cell1->source_index();
                const size_t idx2 = cell2->source_index();

                if (idx1 < filtered_positions.size() && idx2 < filtered_positions.size() && idx1 != idx2)
                {
                    edges.insert(makeEdge(idx1, idx2));
                }
            }
        }
    }

    // Add edges to graph
    for (const auto& [a, b] : edges)
    {
        addBidirectionalEdge(graph, static_cast<Vertex>(a), static_cast<Vertex>(b));
    }

    return graph;
}


} // namespace murosimproc::graph::algorithm
