// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_TESTS_FIXTURES_HPP_
#define MUROSIMPROC_TESTS_FIXTURES_HPP_

#include <gtest/gtest.h>
#include "murosimproc/graph/graph.hpp"
#include "murosimproc/util/array2d.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/io/data_container.hpp"
#include "murosim/random/random_generator.hpp"
#include "murosim/simulation/simulation.hpp"
#include <cmath>
#include <cstdint>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

/// Shared helpers for the murosimproc tests: ASCII grids, a fixed seed, and
/// the chaining of line segments back into closed loops that the geometry
/// tests need to check outlines.
namespace fixtures
{
constexpr float EPSILON = 1e-5f;
constexpr uint64_t SEED = 42;

/// Grid image from ASCII rows: '#' is a wall (1), '.' an obstacle zone (2),
/// anything else free space (0). All rows must have the same length.
inline murosimproc::util::Array2d<uint8_t> makeImage(const std::vector<std::string> &rows)
{
    const size_t size_x = rows.front().size();
    const size_t size_y = rows.size();
    std::vector<uint8_t> data;
    data.reserve(size_x * size_y);
    for (const std::string &row : rows)
    {
        for (const char cell : row)
        {
            data.push_back(cell == '#' ? 1 : (cell == '.' ? 2 : 0));
        }
    }
    return murosimproc::util::Array2d<uint8_t>(size_x, size_y, data);
}

/// Quantised point used to match the end points of neighbouring segments
/// (they agree to well below the resolution).
inline std::pair<long, long> pointKey(const Eigen::Vector2f &point)
{
    constexpr float RESOLUTION = 1e-4f;
    return std::make_pair(std::lround(point.x() / RESOLUTION), std::lround(point.y() / RESOLUTION));
}

/// Closed loops formed by chaining `lines` head to tail (point 2 of one
/// segment is point 1 of the next). Every loop is returned as its ordered
/// vertices without the repeated closing point. Segments that do not close
/// a loop are ignored.
inline std::vector<std::vector<Eigen::Vector2f>> closedLoops(const std::vector<murosim::geometry::Line> &lines)
{
    std::map<std::pair<long, long>, std::vector<size_t>> next_by_start;
    for (size_t i = 0; i < lines.size(); i++)
    {
        next_by_start[pointKey(lines[i].getPoint1())].push_back(i);
    }
    std::vector<bool> used(lines.size(), false);
    std::vector<std::vector<Eigen::Vector2f>> loops;
    for (size_t start = 0; start < lines.size(); start++)
    {
        if (used[start])
        {
            continue;
        }
        std::vector<Eigen::Vector2f> loop;
        size_t current = start;
        bool closed = false;
        while (!used[current])
        {
            used[current] = true;
            loop.push_back(lines[current].getPoint1());
            const auto candidates = next_by_start.find(pointKey(lines[current].getPoint2()));
            if (candidates == next_by_start.end())
            {
                break;
            }
            size_t next = lines.size();
            for (const size_t candidate : candidates->second)
            {
                if (candidate == start)
                {
                    closed = true;
                }
                else if (!used[candidate])
                {
                    next = candidate;
                }
            }
            if (closed || next == lines.size())
            {
                break;
            }
            current = next;
        }
        if (closed)
        {
            loops.push_back(loop);
        }
    }
    return loops;
}

/// Like `closedLoops` but ignores the direction of the segments: from every
/// end point the walk continues along any unused segment sharing it, so it
/// also chains outlines whose segments were emitted in arbitrary order and
/// orientation. Assumes every corner joins exactly two segments.
inline std::vector<std::vector<Eigen::Vector2f>> undirectedLoops(const std::vector<murosim::geometry::Line> &lines)
{
    std::map<std::pair<long, long>, std::vector<size_t>> lines_at;
    for (size_t i = 0; i < lines.size(); i++)
    {
        lines_at[pointKey(lines[i].getPoint1())].push_back(i);
        lines_at[pointKey(lines[i].getPoint2())].push_back(i);
    }
    std::vector<bool> used(lines.size(), false);
    std::vector<std::vector<Eigen::Vector2f>> loops;
    for (size_t start = 0; start < lines.size(); start++)
    {
        if (used[start])
        {
            continue;
        }
        used[start] = true;
        const std::pair<long, long> origin = pointKey(lines[start].getPoint1());
        std::vector<Eigen::Vector2f> loop = {lines[start].getPoint1()};
        Eigen::Vector2f head = lines[start].getPoint2();
        bool closed = false;
        while (!closed)
        {
            if (pointKey(head) == origin)
            {
                closed = true;
                break;
            }
            loop.push_back(head);
            size_t next = lines.size();
            for (const size_t candidate : lines_at[pointKey(head)])
            {
                if (!used[candidate])
                {
                    next = candidate;
                    break;
                }
            }
            if (next == lines.size())
            {
                break;
            }
            used[next] = true;
            const bool entered_at_point1 = pointKey(lines[next].getPoint1()) == pointKey(head);
            head = entered_at_point1 ? lines[next].getPoint2() : lines[next].getPoint1();
        }
        if (closed)
        {
            loops.push_back(loop);
        }
    }
    return loops;
}

/// n x n lattice graph with `step` between neighbours, vertex (i, j) at (i * step, j * step).
inline murosimproc::graph::Graph gridGraph(const size_t n, const float step)
{
    murosimproc::graph::Graph graph;
    std::vector<std::vector<murosimproc::graph::Vertex>> ids(n, std::vector<murosimproc::graph::Vertex>(n));
    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = 0; j < n; j++)
        {
            ids[i][j] = murosimproc::graph::addNode(graph, Eigen::Vector2f(static_cast<float>(i) * step,
                                                                          static_cast<float>(j) * step));
        }
    }
    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = 0; j < n; j++)
        {
            if (i + 1 < n)
            {
                murosimproc::graph::addEdge(graph, ids[i][j], ids[i + 1][j]);
            }
            if (j + 1 < n)
            {
                murosimproc::graph::addEdge(graph, ids[i][j], ids[i][j + 1]);
            }
        }
    }
    return graph;
}

/// Plus-shaped graph: vertex 0 in the centre (degree 4), vertices 1 to 4 one
/// unit out on the four arms (degree 2), vertices 5 to 8 at the arm tips
/// (degree 1).
inline murosimproc::graph::Graph plusGraph()
{
    murosimproc::graph::Graph graph;
    const murosimproc::graph::Vertex center = murosimproc::graph::addNode(graph, Eigen::Vector2f(0.0f, 0.0f));
    const Eigen::Vector2f directions[4] = {{1.0f, 0.0f}, {-1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, -1.0f}};
    std::vector<murosimproc::graph::Vertex> inner;
    for (const Eigen::Vector2f &direction : directions)
    {
        inner.push_back(murosimproc::graph::addNode(graph, direction));
        murosimproc::graph::addEdge(graph, center, inner.back());
    }
    for (size_t i = 0; i < 4; i++)
    {
        const murosimproc::graph::Vertex tip = murosimproc::graph::addNode(graph, directions[i] * 2.0f);
        murosimproc::graph::addEdge(graph, inner[i], tip);
    }
    return graph;
}

/// Whether every vertex can be reached from vertex 0 (true for an empty graph).
inline bool isConnected(const murosimproc::graph::Graph &graph)
{
    if (murosimproc::graph::empty(graph))
    {
        return true;
    }
    std::vector<bool> visited(graph.numVertices(), false);
    std::queue<murosimproc::graph::Vertex> frontier;
    frontier.push(0);
    visited[0] = true;
    size_t reached = 1;
    while (!frontier.empty())
    {
        const murosimproc::graph::Vertex current = frontier.front();
        frontier.pop();
        for (const murosimproc::graph::Vertex next : graph.adjacentVertices(current))
        {
            if (!visited[next])
            {
                visited[next] = true;
                reached++;
                frontier.push(next);
            }
        }
    }
    return reached == graph.numVertices();
}

/// Signed area of a polygon by the shoelace formula (positive when counter-clockwise).
inline float signedArea(const std::vector<Eigen::Vector2f> &polygon)
{
    float area = 0.0f;
    for (size_t i = 0; i < polygon.size(); i++)
    {
        const Eigen::Vector2f &a = polygon[i];
        const Eigen::Vector2f &b = polygon[(i + 1) % polygon.size()];
        area += a.x() * b.y() - b.x() * a.y();
    }
    return area / 2.0f;
}
/// Bitwise equality of two generated worlds: the same seed must replay the
/// exact same random draws, so any float difference is hidden nondeterminism.
inline testing::AssertionResult containersEqual(const murosim::io::DataContainer &a,
                                                const murosim::io::DataContainer &b)
{
    if (a.getName() != b.getName())
    {
        return testing::AssertionFailure() << "names differ: " << a.getName() << " vs " << b.getName();
    }
    if (a.getLines().size() != b.getLines().size() || a.getCircles().size() != b.getCircles().size() ||
        a.getNodes().size() != b.getNodes().size())
    {
        return testing::AssertionFailure() << "element counts differ";
    }
    for (size_t i = 0; i < a.getLines().size(); i++)
    {
        if (a.getLines()[i].getPoint1() != b.getLines()[i].getPoint1() ||
            a.getLines()[i].getPoint2() != b.getLines()[i].getPoint2())
        {
            return testing::AssertionFailure() << "line " << i << " differs";
        }
    }
    for (size_t i = 0; i < a.getCircles().size(); i++)
    {
        if (a.getCircles()[i].getPoint() != b.getCircles()[i].getPoint() ||
            a.getCircles()[i].getRadius() != b.getCircles()[i].getRadius())
        {
            return testing::AssertionFailure() << "circle " << i << " differs";
        }
    }
    for (size_t i = 0; i < a.getNodes().size(); i++)
    {
        if (a.getNodes()[i].getX() != b.getNodes()[i].getX() || a.getNodes()[i].getY() != b.getNodes()[i].getY() ||
            a.getNodes()[i].getRadius() != b.getNodes()[i].getRadius())
        {
            return testing::AssertionFailure() << "node " << i << " differs";
        }
    }
    return testing::AssertionSuccess();
}

/// The output contract every generator shares: the container has geometry,
/// at least `min_node_count` nodes of radius `node_radius`, all inside the
/// world's bounding box, and a Simulation can be built from it.
inline void assertUsableWorld(const murosim::io::DataContainer &container, const size_t min_node_count,
                              const float node_radius)
{
    ASSERT_TRUE(container.isValid());
    ASSERT_GE(container.getNodes().size(), min_node_count);
    const murosim::geometry::World world = container.createWorld();
    const Eigen::Vector2f aabb_min = world.getAabbMin();
    const Eigen::Vector2f aabb_max = world.getAabbMax();
    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        const murosim::navigation::Node &node = container.getNodes()[i];
        ASSERT_FLOAT_EQ(node.getRadius(), node_radius) << "node " << i;
        ASSERT_GE(node.getX(), aabb_min.x()) << "node " << i;
        ASSERT_LE(node.getX(), aabb_max.x()) << "node " << i;
        ASSERT_GE(node.getY(), aabb_min.y()) << "node " << i;
        ASSERT_LE(node.getY(), aabb_max.y()) << "node " << i;
    }
    const murosim::simulation::Simulation simulation(container, 0.1f, 0.2f,
                                                     murosim::simulation::Simulation::INIT_MODE::MANUAL);
    EXPECT_EQ(simulation.getName(), container.getName());
    EXPECT_EQ(simulation.getNodes().getNodes().size(), container.getNodes().size());
}

/// Whether every line end point lies inside the square [0, world_size]².
inline bool linesInsideSquare(const std::vector<murosim::geometry::Line> &lines, const float world_size)
{
    for (const murosim::geometry::Line &line : lines)
    {
        for (const Eigen::Vector2f &point : {line.getPoint1(), line.getPoint2()})
        {
            if (point.x() < -EPSILON || point.y() < -EPSILON || point.x() > world_size + EPSILON ||
                point.y() > world_size + EPSILON)
            {
                return false;
            }
        }
    }
    return true;
}

/// Whether the given lines contain the closed square border of the world.
inline bool hasSquareBorder(const std::vector<murosim::geometry::Line> &lines, const float world_size)
{
    for (const std::vector<Eigen::Vector2f> &loop : undirectedLoops(lines))
    {
        if (std::abs(std::abs(signedArea(loop)) - world_size * world_size) < 1e-3f * world_size * world_size)
        {
            return true;
        }
    }
    return false;
}
} // namespace fixtures

#endif // MUROSIMPROC_TESTS_FIXTURES_HPP_
