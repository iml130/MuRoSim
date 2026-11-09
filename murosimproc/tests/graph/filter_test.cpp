// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/graph/filter.hpp"
#include "murosimproc/graph/graph.hpp"
#include "murosim/random/random_generator.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::random::RandomGenerator;
using murosimproc::graph::Graph;
using murosimproc::graph::Vertex;
using murosimproc::graph::filter::byEdgeChoiceRandom;
using murosimproc::graph::filter::selectNodeVertices;
using fixtures::SEED;

constexpr size_t GRID = 4; // 16 vertices, 24 edges, 15 in a spanning tree
constexpr uint32_t ROUNDS = 10;

bool contains(const std::vector<Vertex> &vertices, const Vertex v)
{
    return std::find(vertices.begin(), vertices.end(), v) != vertices.end();
}
} // namespace

// ============================================================================
// Edge Choice Tests
// ============================================================================

TEST(GraphFilterEdgeChoice, KeepProbabilityOutsideUnitRangeThrows)
{
    Graph graph = fixtures::gridGraph(GRID, 1.0f);
    RandomGenerator rng(SEED);

    EXPECT_THROW(byEdgeChoiceRandom(graph, rng, -0.1f), std::invalid_argument);
    EXPECT_THROW(byEdgeChoiceRandom(graph, rng, 1.1f), std::invalid_argument);
}

TEST(GraphFilterEdgeChoice, EmptyGraphIsANoOp)
{
    Graph graph;
    RandomGenerator rng(SEED);

    EXPECT_NO_THROW(byEdgeChoiceRandom(graph, rng, 0.5f));
    EXPECT_TRUE(murosimproc::graph::empty(graph));
}

TEST(GraphFilterEdgeChoice, KeepProbabilityZeroLeavesASpanningTree)
{
    Graph graph = fixtures::gridGraph(GRID, 1.0f);
    RandomGenerator rng(SEED);

    byEdgeChoiceRandom(graph, rng, 0.0f);

    EXPECT_EQ(graph.edges().size(), GRID * GRID - 1);
    EXPECT_TRUE(fixtures::isConnected(graph));
}

TEST(GraphFilterEdgeChoice, KeepProbabilityOneKeepsEveryEdge)
{
    Graph graph = fixtures::gridGraph(GRID, 1.0f);
    RandomGenerator rng(SEED);

    byEdgeChoiceRandom(graph, rng, 1.0f);

    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(24));
}

TEST(GraphFilterEdgeChoice, IntermediateProbabilityKeepsThatShareOfTheExtraEdges)
{
    Graph graph = fixtures::gridGraph(GRID, 1.0f);
    RandomGenerator rng(SEED);

    byEdgeChoiceRandom(graph, rng, 0.5f);

    // 15 tree edges plus half of the 9 others, rounded down
    EXPECT_EQ(graph.edges().size(), static_cast<size_t>(15 + 4));
    EXPECT_TRUE(fixtures::isConnected(graph));
}

TEST(GraphFilterEdgeChoice, GraphStaysConnectedForEverySeed)
{
    for (uint32_t seed = 0; seed < ROUNDS; seed++)
    {
        Graph graph = fixtures::gridGraph(GRID, 1.0f);
        RandomGenerator rng(seed);

        byEdgeChoiceRandom(graph, rng, 0.2f);

        EXPECT_TRUE(fixtures::isConnected(graph)) << "seed " << seed;
        EXPECT_GE(graph.edges().size(), GRID * GRID - 1) << "seed " << seed;
    }
}

TEST(GraphFilterEdgeChoice, SameSeedGivesTheSameGraph)
{
    Graph first = fixtures::gridGraph(GRID, 1.0f);
    Graph second = fixtures::gridGraph(GRID, 1.0f);
    RandomGenerator rng_first(SEED);
    RandomGenerator rng_second(SEED);

    byEdgeChoiceRandom(first, rng_first, 0.3f);
    byEdgeChoiceRandom(second, rng_second, 0.3f);

    const std::vector<murosimproc::graph::Edge> edges_first = first.edges();
    const std::vector<murosimproc::graph::Edge> edges_second = second.edges();
    ASSERT_EQ(edges_first.size(), edges_second.size());
    for (size_t i = 0; i < edges_first.size(); i++)
    {
        EXPECT_EQ(edges_first[i].source, edges_second[i].source) << "edge " << i;
        EXPECT_EQ(edges_first[i].target, edges_second[i].target) << "edge " << i;
    }
}

// ============================================================================
// Node Vertex Selection Tests
// ============================================================================

// A robot parked on a junction blocks several tubes, so junctions are only
// used to fill up to the minimum node count; dead ends and bends always
// qualify, isolated vertices never.

TEST(GraphFilterNodeVertices, JunctionExcludedWhenEnoughLowDegreeVertices)
{
    const Graph graph = fixtures::plusGraph();

    for (uint32_t seed = 0; seed < ROUNDS; seed++)
    {
        RandomGenerator rng(seed);
        const std::vector<Vertex> selected = selectNodeVertices(graph, 8, rng); // 8 vertices of degree <= 2 suffice
        EXPECT_EQ(selected.size(), static_cast<size_t>(8)) << "seed " << seed;
        EXPECT_FALSE(contains(selected, 0)) << "seed " << seed;
    }
}

TEST(GraphFilterNodeVertices, JunctionFillsUpToTheMinimumNodeCount)
{
    const Graph graph = fixtures::plusGraph();
    RandomGenerator rng(SEED);

    const std::vector<Vertex> selected = selectNodeVertices(graph, 9, rng);

    EXPECT_EQ(selected.size(), static_cast<size_t>(9));
    EXPECT_TRUE(contains(selected, 0));
}

TEST(GraphFilterNodeVertices, LowDegreeVerticesAreNeverDropped)
{
    const Graph graph = fixtures::plusGraph();
    RandomGenerator rng(SEED);

    const std::vector<Vertex> selected = selectNodeVertices(graph, 2, rng); // far below the 8 available

    EXPECT_EQ(selected.size(), static_cast<size_t>(8));
    EXPECT_FALSE(contains(selected, 0));
}

TEST(GraphFilterNodeVertices, IsolatedVertexIsNeverSelected)
{
    Graph graph;
    const Vertex isolated = murosimproc::graph::addNode(graph, Eigen::Vector2f(5.0f, 5.0f));
    const Vertex a = murosimproc::graph::addNode(graph, Eigen::Vector2f(0.0f, 0.0f));
    const Vertex b = murosimproc::graph::addNode(graph, Eigen::Vector2f(1.0f, 0.0f));
    murosimproc::graph::addEdge(graph, a, b);
    RandomGenerator rng(SEED);

    const std::vector<Vertex> selected = selectNodeVertices(graph, 10, rng);

    EXPECT_EQ(selected.size(), static_cast<size_t>(2));
    EXPECT_FALSE(contains(selected, isolated));
}

TEST(GraphFilterNodeVertices, LowestDegreeJunctionsAreUsedFirst)
{
    // two junctions: j3 of degree 3 and j4 of degree 4; with one fallback
    // slot the degree-3 junction must win
    Graph graph;
    const Vertex j3 = murosimproc::graph::addNode(graph, Eigen::Vector2f(0.0f, 0.0f));
    const Vertex j4 = murosimproc::graph::addNode(graph, Eigen::Vector2f(10.0f, 0.0f));
    for (int i = 0; i < 3; i++)
    {
        murosimproc::graph::addEdge(graph, j3, murosimproc::graph::addNode(graph, Eigen::Vector2f(-1.0f, i)));
    }
    for (int i = 0; i < 4; i++)
    {
        murosimproc::graph::addEdge(graph, j4, murosimproc::graph::addNode(graph, Eigen::Vector2f(11.0f, i)));
    }

    for (uint32_t seed = 0; seed < ROUNDS; seed++)
    {
        RandomGenerator rng(seed);
        const std::vector<Vertex> selected = selectNodeVertices(graph, 8, rng); // 7 leaves plus one junction
        EXPECT_EQ(selected.size(), static_cast<size_t>(8)) << "seed " << seed;
        EXPECT_TRUE(contains(selected, j3)) << "seed " << seed;
        EXPECT_FALSE(contains(selected, j4)) << "seed " << seed;
    }
}

TEST(GraphFilterNodeVertices, EmptyGraphSelectsNothing)
{
    RandomGenerator rng(SEED);

    EXPECT_TRUE(selectNodeVertices(Graph(), 3, rng).empty());
}
