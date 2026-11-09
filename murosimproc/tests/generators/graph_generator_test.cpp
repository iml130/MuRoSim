// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/graph_generator.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <stdexcept>

namespace
{
using murosim::math::PI;
using murosim::math::PI_4;
using murosimproc::generators::GraphGenerator;
using murosimproc::generators::RandomRange;
using murosim::io::DataContainer;

constexpr size_t MIN_NODE_COUNT = 5;
constexpr float NODE_RADIUS = 0.25f;
constexpr float WORLD_SIZE = 10.0f;
const RandomRange SPACING(0.7f, 1.25f);
const RandomRange EDGE_LENGTH(1.5f, 4.0f);
const RandomRange POINT_REMOVAL(0.05f, 0.75f);
const RandomRange EDGE_REMOVAL(0.05f, 0.75f);
constexpr size_t BUCKETS = 5;
constexpr size_t ARC_SEGMENTS = 8;
constexpr float SIMPLIFY = 0.0f;

GraphGenerator graphGenerator(const RandomRange world_size = WORLD_SIZE, const RandomRange spacing = SPACING,
                              const RandomRange edge_length = EDGE_LENGTH,
                              const RandomRange point_removal = POINT_REMOVAL,
                              const RandomRange edge_removal = EDGE_REMOVAL, const size_t buckets = BUCKETS,
                              const size_t arc_segments = ARC_SEGMENTS, const float simplify = SIMPLIFY)
{
    return GraphGenerator(MIN_NODE_COUNT, NODE_RADIUS, world_size, spacing, edge_length, point_removal,
                          edge_removal, buckets, arc_segments, simplify);
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(GraphGeneratorConstructor, NameAndDefaults)
{
    const GraphGenerator generator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, SPACING, EDGE_LENGTH, POINT_REMOVAL,
                                   EDGE_REMOVAL);

    EXPECT_EQ(generator.getName(), "graph");
    EXPECT_EQ(generator.getMinNodeCount(), MIN_NODE_COUNT);
}

TEST(GraphGeneratorConstructor, RemovalRatesMustBeProbabilities)
{
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, RandomRange(-0.1f, 0.5f)), std::invalid_argument);
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, RandomRange(0.5f, 1.1f)), std::invalid_argument);
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, POINT_REMOVAL, RandomRange(-0.1f, 0.5f)),
                 std::invalid_argument);
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, POINT_REMOVAL, RandomRange(0.5f, 1.1f)),
                 std::invalid_argument);
    EXPECT_NO_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, RandomRange(0.0f, 1.0f), RandomRange(0.0f, 1.0f)));
}

TEST(GraphGeneratorConstructor, EdgeLengthMustCoverTheTubeWidth)
{
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, RandomRange(0.0f, 4.0f)), std::invalid_argument);
    // half the maximum spacing is 0.625
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, RandomRange(0.6f, 4.0f)), std::invalid_argument);
    EXPECT_NO_THROW(graphGenerator(WORLD_SIZE, SPACING, RandomRange(0.65f, 4.0f)));
}

TEST(GraphGeneratorConstructor, TubeParametersAreValidated)
{
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, POINT_REMOVAL, EDGE_REMOVAL, 0),
                 std::invalid_argument);
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, POINT_REMOVAL, EDGE_REMOVAL, BUCKETS, 0),
                 std::invalid_argument);
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, EDGE_LENGTH, POINT_REMOVAL, EDGE_REMOVAL, BUCKETS, ARC_SEGMENTS,
                                -0.1f),
                 std::invalid_argument);
}

TEST(GraphGeneratorConstructor, WorldMustFitTheLongestEdgeInsideTheMargin)
{
    // 10 m minus twice the 0.625 m margin leaves 8.75 m for the points
    EXPECT_THROW(graphGenerator(WORLD_SIZE, SPACING, RandomRange(1.5f, 9.0f)), std::invalid_argument);
    EXPECT_NO_THROW(graphGenerator(WORLD_SIZE, SPACING, RandomRange(1.5f, 8.0f)));
    EXPECT_THROW(graphGenerator(1.2f, SPACING, RandomRange(0.7f, 1.0f)), std::invalid_argument);
}

// ============================================================================
// Generation Tests
// ============================================================================

TEST(GraphGeneratorGenerate, WorldIsUsable)
{
    const DataContainer container = graphGenerator()(fixtures::SEED);

    EXPECT_TRUE(container.getCircles().empty());
    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
}

TEST(GraphGeneratorGenerate, TubesStayInsideTheBorder)
{
    const DataContainer container = graphGenerator()(fixtures::SEED);

    EXPECT_GT(container.getLines().size(), static_cast<size_t>(4));
    EXPECT_TRUE(fixtures::hasSquareBorder(container.getLines(), WORLD_SIZE));
    EXPECT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE));
}

TEST(GraphGeneratorGenerate, NodesKeepTheTubeMarginToTheBorder)
{
    const DataContainer container = graphGenerator()(fixtures::SEED);
    const float margin = SPACING.max() / 2.0f;

    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        const murosim::navigation::Node &node = container.getNodes()[i];
        EXPECT_GE(node.getX(), margin - fixtures::EPSILON) << "node " << i;
        EXPECT_LE(node.getX(), WORLD_SIZE - margin + fixtures::EPSILON) << "node " << i;
        EXPECT_GE(node.getY(), margin - fixtures::EPSILON) << "node " << i;
        EXPECT_LE(node.getY(), WORLD_SIZE - margin + fixtures::EPSILON) << "node " << i;
    }
}

TEST(GraphGeneratorGenerate, NodesKeepTheMinimumEdgeLengthApart)
{
    const DataContainer container = graphGenerator()(fixtures::SEED);

    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        for (size_t j = 0; j < i; j++)
        {
            const float dx = container.getNodes()[i].getX() - container.getNodes()[j].getX();
            const float dy = container.getNodes()[i].getY() - container.getNodes()[j].getY();
            EXPECT_GE(std::hypot(dx, dy), EDGE_LENGTH.min() - fixtures::EPSILON) << "nodes " << i << " and " << j;
        }
    }
}

TEST(GraphGeneratorGenerate, NodeBiasFollowsTheDegree)
{
    const DataContainer container = graphGenerator()(fixtures::SEED);

    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        const float bias = container.getNodes()[i].getStartBias();
        EXPECT_TRUE(std::abs(bias - PI_4) < fixtures::EPSILON || std::abs(bias - PI) < fixtures::EPSILON)
            << "node " << i << " bias " << bias;
    }
}

TEST(GraphGeneratorGenerate, SameSeedGivesTheSameWorld)
{
    const GraphGenerator generator = graphGenerator(RandomRange(10.0f, 12.0f));

    EXPECT_TRUE(fixtures::containersEqual(generator(fixtures::SEED), generator(fixtures::SEED)));
    EXPECT_FALSE(fixtures::containersEqual(generator(1), generator(2)));
}
