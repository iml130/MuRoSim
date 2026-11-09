// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/line_generator.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <stdexcept>

namespace
{
using murosim::math::PI;
using murosim::math::PI_4;
using murosimproc::generators::LineGenerator;
using murosimproc::generators::RandomRange;
using murosim::io::DataContainer;

constexpr size_t MIN_NODE_COUNT = 2;
constexpr float NODE_RADIUS = 0.25f;
constexpr float WORLD_SIZE = 10.0f; // fixed, so the border and the centre are known
const RandomRange SPACING(0.6f, 1.0f);
constexpr float LENGTH_OFFSET = 0.5f;
constexpr size_t BORDER_LINES = 4;
constexpr size_t RECTANGLE_LINES = 4;

LineGenerator lineGenerator(const bool is_horizontal, const RandomRange world_size = WORLD_SIZE,
                            const RandomRange spacing = SPACING, const size_t min_node_count = MIN_NODE_COUNT)
{
    return LineGenerator(min_node_count, NODE_RADIUS, world_size, spacing, LENGTH_OFFSET, is_horizontal);
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(LineGeneratorConstructor, NameAndConfiguration)
{
    const LineGenerator generator = lineGenerator(true);

    EXPECT_EQ(generator.getName(), "line");
    EXPECT_EQ(generator.getMinNodeCount(), MIN_NODE_COUNT);
    EXPECT_FLOAT_EQ(generator.getNodeRadius(), NODE_RADIUS);
}

TEST(LineGeneratorConstructor, NodeCountIsRaisedToTwo)
{
    EXPECT_EQ(lineGenerator(true, WORLD_SIZE, SPACING, 0).getMinNodeCount(), static_cast<size_t>(2));
    EXPECT_EQ(lineGenerator(true, WORLD_SIZE, SPACING, 3).getMinNodeCount(), static_cast<size_t>(3));
}

TEST(LineGeneratorConstructor, WorldMustBeTwiceTheSpacing)
{
    EXPECT_THROW(lineGenerator(true, 2.0f, 1.0f), std::invalid_argument);
    EXPECT_NO_THROW(lineGenerator(true, 5.0f, 1.0f));
}

TEST(LineGeneratorConstructor, WorldMustHoldTheNodeSpace)
{
    // 3 nodes x 1 m spacing x 2 = 6 m
    EXPECT_THROW(lineGenerator(true, 6.0f, 1.0f, 3), std::invalid_argument);
    EXPECT_NO_THROW(lineGenerator(true, 7.0f, 1.0f, 3));
}

// ============================================================================
// Horizontal Tests
// ============================================================================

TEST(LineGeneratorHorizontal, BorderPlusOneRectangle)
{
    const DataContainer container = lineGenerator(true)(fixtures::SEED);

    EXPECT_EQ(container.getLines().size(), BORDER_LINES + RECTANGLE_LINES);
    EXPECT_TRUE(container.getCircles().empty());
    EXPECT_TRUE(fixtures::hasSquareBorder(container.getLines(), WORLD_SIZE));
    EXPECT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE));
    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
}

TEST(LineGeneratorHorizontal, TwoNodesOnTheCentreLineFacingOutwards)
{
    const DataContainer container = lineGenerator(true)(fixtures::SEED);

    ASSERT_EQ(container.getNodes().size(), static_cast<size_t>(2));
    const murosim::navigation::Node &right = container.getNodes()[0];
    const murosim::navigation::Node &left = container.getNodes()[1];
    EXPECT_FLOAT_EQ(right.getY(), WORLD_SIZE / 2.0f);
    EXPECT_FLOAT_EQ(left.getY(), WORLD_SIZE / 2.0f);
    EXPECT_GT(right.getX(), WORLD_SIZE / 2.0f);
    EXPECT_LT(left.getX(), WORLD_SIZE / 2.0f);
    EXPECT_NEAR(right.getX() - WORLD_SIZE / 2.0f, WORLD_SIZE / 2.0f - left.getX(), fixtures::EPSILON);
    EXPECT_NEAR(right.getStartAngle(), 0.0f, fixtures::EPSILON); // the start angle points away from the centre
    EXPECT_NEAR(std::abs(left.getStartAngle()), PI, fixtures::EPSILON);
    EXPECT_FLOAT_EQ(right.getStartBias(), PI_4);
    EXPECT_FLOAT_EQ(left.getStartBias(), PI_4);
}

TEST(LineGeneratorHorizontal, NodesLieInsideTheRectangle)
{
    const DataContainer container = lineGenerator(true)(fixtures::SEED);

    ASSERT_EQ(container.getLines().size(), BORDER_LINES + RECTANGLE_LINES);
    const std::vector<murosim::geometry::Line> rectangle(container.getLines().begin() + BORDER_LINES,
                                                         container.getLines().end());
    Eigen::Vector2f rectangle_min(WORLD_SIZE, WORLD_SIZE);
    Eigen::Vector2f rectangle_max(0.0f, 0.0f);
    for (const murosim::geometry::Line &line : rectangle)
    {
        rectangle_min = rectangle_min.cwiseMin(line.getPoint1()).cwiseMin(line.getPoint2());
        rectangle_max = rectangle_max.cwiseMax(line.getPoint1()).cwiseMax(line.getPoint2());
    }
    EXPECT_EQ(fixtures::undirectedLoops(rectangle).size(), static_cast<size_t>(1));
    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        const murosim::navigation::Node &node = container.getNodes()[i];
        EXPECT_GT(node.getX() - NODE_RADIUS, rectangle_min.x()) << "node " << i;
        EXPECT_LT(node.getX() + NODE_RADIUS, rectangle_max.x()) << "node " << i;
        EXPECT_GT(node.getY() - NODE_RADIUS, rectangle_min.y()) << "node " << i;
        EXPECT_LT(node.getY() + NODE_RADIUS, rectangle_max.y()) << "node " << i;
    }
}

// ============================================================================
// Rotated Tests
// ============================================================================

TEST(LineGeneratorRotated, NodesAreFarApartInsideTheWorld)
{
    const DataContainer container = lineGenerator(false)(fixtures::SEED);

    ASSERT_EQ(container.getNodes().size(), static_cast<size_t>(2));
    const Eigen::Vector2f a(container.getNodes()[0].getX(), container.getNodes()[0].getY());
    const Eigen::Vector2f b(container.getNodes()[1].getX(), container.getNodes()[1].getY());
    // the end points stay a node-space diagonal away from the border
    const float node_space_min = SPACING.min() * static_cast<float>(MIN_NODE_COUNT);
    const float reach = WORLD_SIZE / 2.0f - std::sqrt(2.0f) * node_space_min;
    EXPECT_GT((a - b).norm(), 2.0f * std::sqrt(2.0f) * node_space_min);
    for (const Eigen::Vector2f &point : {a, b})
    {
        EXPECT_LE(std::abs(point.x() - WORLD_SIZE / 2.0f), reach + fixtures::EPSILON);
        EXPECT_LE(std::abs(point.y() - WORLD_SIZE / 2.0f), reach + fixtures::EPSILON);
    }
    EXPECT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE));
    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
}

TEST(LineGeneratorRotated, NodesFaceAwayFromEachOther)
{
    const DataContainer container = lineGenerator(false)(fixtures::SEED);

    ASSERT_EQ(container.getNodes().size(), static_cast<size_t>(2));
    const murosim::navigation::Node &a = container.getNodes()[0];
    const murosim::navigation::Node &b = container.getNodes()[1];
    const float away_from_b = std::atan2(a.getY() - b.getY(), a.getX() - b.getX());
    const float away_from_a = std::atan2(b.getY() - a.getY(), b.getX() - a.getX());
    EXPECT_NEAR(a.getStartAngle(), away_from_b, fixtures::EPSILON);
    EXPECT_NEAR(b.getStartAngle(), away_from_a, fixtures::EPSILON);
}

TEST(LineGeneratorRotated, SameSeedGivesTheSameWorld)
{
    const LineGenerator generator = lineGenerator(false, RandomRange(10.0f, 12.0f));

    EXPECT_TRUE(fixtures::containersEqual(generator(fixtures::SEED), generator(fixtures::SEED)));
    EXPECT_FALSE(fixtures::containersEqual(generator(1), generator(2)));
}
