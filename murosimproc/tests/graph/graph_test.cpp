// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/graph/graph.hpp"
#include <set>
#include <vector>

namespace
{
using murosimproc::graph::Edge;
using murosimproc::graph::Graph;
using murosimproc::graph::Vertex;
using murosimproc::graph::VertexProperty;
} // namespace

// ============================================================================
// Vertex Property Tests
// ============================================================================

TEST(GraphVertexProperty, ConstructorsStoreThePosition)
{
    EXPECT_TRUE(VertexProperty().position.isZero());
    EXPECT_FLOAT_EQ(VertexProperty(Eigen::Vector2f(1.0f, 2.0f)).position.y(), 2.0f);
    EXPECT_FLOAT_EQ(VertexProperty(3.0f, 4.0f).position.x(), 3.0f);
}

// ============================================================================
// Test Fixture
// ============================================================================

// A triangle 0-1-2 with a pendant vertex 3 attached to vertex 2.
class GraphTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        for (int i = 0; i < 4; i++)
        {
            graph.addVertex(VertexProperty(static_cast<float>(i), 0.0f));
        }
        graph.addEdge(0, 1);
        graph.addEdge(1, 2);
        graph.addEdge(2, 0);
        graph.addEdge(2, 3);
    }

    Graph graph;
};

// ============================================================================
// Vertex Tests
// ============================================================================

TEST(GraphVertices, EmptyGraphHasNoVertices)
{
    const Graph graph;

    EXPECT_EQ(graph.numVertices(), static_cast<size_t>(0));
    EXPECT_TRUE(murosimproc::graph::empty(graph));
    EXPECT_TRUE(graph.edges().empty());
}

TEST(GraphVertices, AddVertexReturnsConsecutiveDescriptors)
{
    Graph graph;

    EXPECT_EQ(graph.addVertex(VertexProperty(0.0f, 0.0f)), static_cast<Vertex>(0));
    EXPECT_EQ(graph.addVertex(VertexProperty(1.0f, 0.0f)), static_cast<Vertex>(1));
    EXPECT_EQ(murosimproc::graph::addNode(graph, Eigen::Vector2f(2.0f, 0.0f)), static_cast<Vertex>(2));
    EXPECT_EQ(graph.numVertices(), static_cast<size_t>(3));
    EXPECT_EQ(murosimproc::graph::getNodeSize(graph), static_cast<size_t>(3));
    EXPECT_FALSE(murosimproc::graph::empty(graph));
}

TEST_F(GraphTest, PositionsAreStoredAndWritable)
{
    EXPECT_FLOAT_EQ(graph[3].position.x(), 3.0f);
    EXPECT_FLOAT_EQ(murosimproc::graph::getNodePosition(graph, 3).x(), 3.0f);

    graph[3].position = Eigen::Vector2f(7.0f, 8.0f);

    EXPECT_FLOAT_EQ(murosimproc::graph::getNodePosition(graph, 3).y(), 8.0f);
}

// ============================================================================
// Edge Tests
// ============================================================================

TEST_F(GraphTest, EdgesAreUndirected)
{
    EXPECT_EQ(graph.adjacentVertices(0), (std::set<size_t>{1, 2}));
    EXPECT_EQ(graph.adjacentVertices(3), (std::set<size_t>{2}));
    EXPECT_EQ(murosimproc::graph::getNodeNeighbors(graph, 2), (std::set<size_t>{0, 1, 3}));
}

TEST_F(GraphTest, EdgeListHasEachEdgeOnceWithSourceBelowTarget)
{
    const std::vector<Edge> edges = murosimproc::graph::getEdges(graph);

    ASSERT_EQ(edges.size(), static_cast<size_t>(4));
    for (size_t i = 0; i < edges.size(); i++)
    {
        EXPECT_LT(edges[i].source, edges[i].target) << "edge " << i;
    }
    EXPECT_EQ(edges[0].source, static_cast<Vertex>(0));
    EXPECT_EQ(edges[0].target, static_cast<Vertex>(1));
    EXPECT_EQ(edges[3].source, static_cast<Vertex>(2));
    EXPECT_EQ(edges[3].target, static_cast<Vertex>(3));
}

TEST_F(GraphTest, AddEdgeReportsNewAndDuplicate)
{
    const std::pair<Edge, bool> fresh = graph.addEdge(0, 3);
    const std::pair<Edge, bool> duplicate = graph.addEdge(3, 0); // the same undirected edge

    EXPECT_TRUE(fresh.second);
    EXPECT_FALSE(duplicate.second);
    EXPECT_EQ(fresh.first.source, static_cast<Vertex>(0));
    EXPECT_EQ(fresh.first.target, static_cast<Vertex>(3));
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(5));
}

TEST_F(GraphTest, BidirectionalEdgeIsTheSameAsAnEdge)
{
    EXPECT_FALSE(murosimproc::graph::addBidirectionalEdge(graph, 0, 1).second); // exists already
    EXPECT_TRUE(murosimproc::graph::addBidirectionalEdge(graph, 1, 3).second);
    EXPECT_EQ(graph.adjacentVertices(3), (std::set<size_t>{1, 2}));
}

TEST_F(GraphTest, RemoveEdgeDropsBothDirections)
{
    graph.removeEdge(1, 2);

    EXPECT_EQ(graph.adjacentVertices(1), (std::set<size_t>{0}));
    EXPECT_EQ(graph.adjacentVertices(2), (std::set<size_t>{0, 3}));
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(3));
}

TEST_F(GraphTest, RemovingAMissingEdgeIsANoOp)
{
    murosimproc::graph::removeEdge(graph, 0, 3);

    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(4));
}
