// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/graph/algorithm.hpp"
#include "murosimproc/graph/graph.hpp"
#include "murosimproc/util/points.hpp"
#include "fixtures.hpp"
#include <vector>

namespace
{
using murosimproc::graph::Graph;
using murosimproc::graph::algorithm::makeDelaunayGraph;
using fixtures::SEED;

bool hasEdge(const Graph &graph, const size_t u, const size_t v)
{
    return graph.adjacentVertices(u).count(v) == 1;
}
} // namespace

// ============================================================================
// Degenerate Input Tests
// ============================================================================

TEST(GraphDelaunay, FewerThanTwoPointsGiveAnEmptyGraph)
{
    EXPECT_TRUE(murosimproc::graph::empty(makeDelaunayGraph({})));
    EXPECT_TRUE(murosimproc::graph::empty(makeDelaunayGraph({Eigen::Vector2f(1.0f, 1.0f)})));
}

TEST(GraphDelaunay, TwoPointsGiveOneEdge)
{
    const Graph graph = makeDelaunayGraph({Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(3.0f, 4.0f)});

    ASSERT_EQ(graph.numVertices(), static_cast<size_t>(2));
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(1));
    EXPECT_TRUE(hasEdge(graph, 0, 1));
}

TEST(GraphDelaunay, CoincidentPointsAreMergedAndKeepTheFirstPosition)
{
    const Graph graph = makeDelaunayGraph({Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(0.0f, 0.0f),
                                           Eigen::Vector2f(2.0f, 0.0f)});

    ASSERT_EQ(graph.numVertices(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(graph[1].position.x(), 2.0f);
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(1));
}

TEST(GraphDelaunay, IdenticalPointsGiveAnEmptyGraph)
{
    const Graph graph = makeDelaunayGraph({Eigen::Vector2f(1.0f, 1.0f), Eigen::Vector2f(1.0f, 1.0f)});

    EXPECT_TRUE(murosimproc::graph::empty(graph)); // no extent to triangulate
}

// ============================================================================
// Triangulation Tests
// ============================================================================

// The four corners of a square lie on one circle, so no diagonal is
// Delaunay: the Voronoi construction yields the hull only.
TEST(GraphDelaunay, CocircularSquareGivesOnlyTheHull)
{
    const Graph graph = makeDelaunayGraph({Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f),
                                           Eigen::Vector2f(1.0f, 1.0f), Eigen::Vector2f(0.0f, 1.0f)});

    ASSERT_EQ(graph.numVertices(), static_cast<size_t>(4));
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(4));
    EXPECT_TRUE(hasEdge(graph, 0, 1));
    EXPECT_TRUE(hasEdge(graph, 1, 2));
    EXPECT_TRUE(hasEdge(graph, 2, 3));
    EXPECT_TRUE(hasEdge(graph, 3, 0));
}

TEST(GraphDelaunay, QuadrilateralGivesHullAndOneDiagonal)
{
    // corner 3 pulled up: the circle through 0, 1, 2 no longer contains it
    const Graph graph = makeDelaunayGraph({Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(2.0f, 0.0f),
                                           Eigen::Vector2f(2.0f, 1.0f), Eigen::Vector2f(0.0f, 1.5f)});

    ASSERT_EQ(graph.numVertices(), static_cast<size_t>(4));
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(5));
    EXPECT_TRUE(hasEdge(graph, 0, 1));
    EXPECT_TRUE(hasEdge(graph, 1, 2));
    EXPECT_TRUE(hasEdge(graph, 2, 3));
    EXPECT_TRUE(hasEdge(graph, 3, 0));
    EXPECT_TRUE(hasEdge(graph, 0, 2) != hasEdge(graph, 1, 3)); // exactly one diagonal
}

TEST(GraphDelaunay, TriangleWithCentrePointFansOut)
{
    const Graph graph = makeDelaunayGraph({Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(4.0f, 0.0f),
                                           Eigen::Vector2f(2.0f, 4.0f), Eigen::Vector2f(2.0f, 1.5f)});

    // the centre point connects to every corner: 3 hull edges plus 3 spokes
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(6));
    EXPECT_EQ(graph.adjacentVertices(3).size(), static_cast<size_t>(3));
}

TEST(GraphDelaunay, CollinearPointsFormAChain)
{
    const Graph graph = makeDelaunayGraph({Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f),
                                           Eigen::Vector2f(2.0f, 0.0f)});

    EXPECT_TRUE(hasEdge(graph, 0, 1));
    EXPECT_TRUE(hasEdge(graph, 1, 2));
    EXPECT_FALSE(hasEdge(graph, 0, 2)); // the middle point separates the outer two
}

TEST(GraphDelaunay, RandomCloudIsConnectedAndPlanar)
{
    std::vector<Eigen::Vector2f> points;
    murosimproc::util::points::createRandomPointsRectangleUniform(points, SEED, Eigen::Vector2f(0.0f, 0.0f),
                                                                  Eigen::Vector2f(10.0f, 10.0f), 60);

    const Graph graph = makeDelaunayGraph(points);

    ASSERT_EQ(graph.numVertices(), static_cast<size_t>(60));
    EXPECT_TRUE(fixtures::isConnected(graph));
    EXPECT_LE(graph.edges().size(), static_cast<size_t>(3 * 60 - 6)); // Euler bound for planar graphs
    EXPECT_GE(graph.edges().size(), static_cast<size_t>(60 - 1));
}

TEST(GraphDelaunay, VertexOrderFollowsTheInput)
{
    const std::vector<Eigen::Vector2f> points = {Eigen::Vector2f(5.0f, 5.0f), Eigen::Vector2f(0.0f, 0.0f),
                                                 Eigen::Vector2f(5.0f, 0.0f)};

    const Graph graph = makeDelaunayGraph(points);

    for (size_t i = 0; i < points.size(); i++)
    {
        EXPECT_FLOAT_EQ(graph[i].position.x(), points[i].x()) << "vertex " << i;
        EXPECT_FLOAT_EQ(graph[i].position.y(), points[i].y()) << "vertex " << i;
    }
}
