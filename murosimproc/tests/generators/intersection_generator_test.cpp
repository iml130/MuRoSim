// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/intersection_generator.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <stdexcept>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosimproc::generators::IntersectionGenerator;
using murosimproc::generators::RandomRange;
using murosim::io::DataContainer;

constexpr size_t MIN_NODE_COUNT = 4;
constexpr float NODE_RADIUS = 0.25f;
constexpr float WORLD_SIZE = 10.0f;
constexpr float SPACING = 2.0f; // fixed corridor width, so every corner is known
constexpr size_t LINES = 4 + 8; // border plus two lines per corner block

IntersectionGenerator intersectionGenerator(const float center_offset, const RandomRange world_size = WORLD_SIZE,
                                            const RandomRange spacing = SPACING)
{
    return IntersectionGenerator(MIN_NODE_COUNT, NODE_RADIUS, world_size, spacing, center_offset);
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(IntersectionGeneratorConstructor, NameAndConfiguration)
{
    const IntersectionGenerator generator = intersectionGenerator(0.0f);

    EXPECT_EQ(generator.getName(), "intersection");
    EXPECT_EQ(generator.getMinNodeCount(), MIN_NODE_COUNT);
}

TEST(IntersectionGeneratorConstructor, WorldMustBeThreeTimesTheSpacing)
{
    EXPECT_THROW(intersectionGenerator(0.0f, 6.0f, 2.0f), std::invalid_argument);
    EXPECT_NO_THROW(intersectionGenerator(0.0f, 7.0f, 2.0f));
}

// ============================================================================
// Geometry Tests
// ============================================================================

TEST(IntersectionGeometry, CentredCrossHasFourCornerBlocks)
{
    const DataContainer container = intersectionGenerator(0.0f)(fixtures::SEED);

    ASSERT_EQ(container.getLines().size(), LINES);
    EXPECT_TRUE(container.getCircles().empty());
    EXPECT_TRUE(fixtures::hasSquareBorder(container.getLines(), WORLD_SIZE));
    const float inner = WORLD_SIZE / 2.0f - SPACING / 2.0f;
    const float outer = WORLD_SIZE / 2.0f + SPACING / 2.0f;
    for (size_t i = 4; i < LINES; i++)
    {
        const murosim::geometry::Line &line = container.getLines()[i];
        // one end on a corner of the crossing, the other on the border
        EXPECT_TRUE(std::abs(line.getPoint1X() - inner) < fixtures::EPSILON ||
                    std::abs(line.getPoint1X() - outer) < fixtures::EPSILON)
            << "line " << i;
        EXPECT_TRUE(std::abs(line.getPoint1Y() - inner) < fixtures::EPSILON ||
                    std::abs(line.getPoint1Y() - outer) < fixtures::EPSILON)
            << "line " << i;
        EXPECT_TRUE(line.getPoint2X() == 0.0f || line.getPoint2X() == WORLD_SIZE || line.getPoint2Y() == 0.0f ||
                    line.getPoint2Y() == WORLD_SIZE)
            << "line " << i;
        EXPECT_TRUE(line.getPoint1X() == line.getPoint2X() || line.getPoint1Y() == line.getPoint2Y())
            << "line " << i;
    }
}

TEST(IntersectionGeometry, FourNodesAtTheCorridorEndsFacingTheCentre)
{
    const DataContainer container = intersectionGenerator(0.0f)(fixtures::SEED);

    ASSERT_EQ(container.getNodes().size(), static_cast<size_t>(4));
    const float half_spacing = SPACING / 2.0f;
    const float centre = WORLD_SIZE / 2.0f;
    const murosim::navigation::Node &west = container.getNodes()[0];
    const murosim::navigation::Node &south = container.getNodes()[1];
    const murosim::navigation::Node &east = container.getNodes()[2];
    const murosim::navigation::Node &north = container.getNodes()[3];
    EXPECT_FLOAT_EQ(west.getX(), half_spacing);
    EXPECT_FLOAT_EQ(west.getY(), centre);
    EXPECT_FLOAT_EQ(west.getStartAngle(), 0.0f);
    EXPECT_FLOAT_EQ(south.getX(), centre);
    EXPECT_FLOAT_EQ(south.getY(), half_spacing);
    EXPECT_FLOAT_EQ(south.getStartAngle(), PI_2);
    EXPECT_FLOAT_EQ(east.getX(), WORLD_SIZE - half_spacing);
    EXPECT_FLOAT_EQ(east.getY(), centre);
    EXPECT_FLOAT_EQ(east.getStartAngle(), PI);
    EXPECT_FLOAT_EQ(north.getX(), centre);
    EXPECT_FLOAT_EQ(north.getY(), WORLD_SIZE - half_spacing);
    EXPECT_FLOAT_EQ(north.getStartAngle(), -PI_2);
    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        EXPECT_FLOAT_EQ(container.getNodes()[i].getStartBias(), PI_4) << "node " << i;
    }
    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
}

TEST(IntersectionGeometry, CentreOffsetMovesTheCrossingWithinTheWorld)
{
    const IntersectionGenerator generator = intersectionGenerator(1.0f);

    bool moved = false;
    for (uint32_t seed = 0; seed < 10; seed++)
    {
        const DataContainer container = generator(seed);
        ASSERT_EQ(container.getNodes().size(), static_cast<size_t>(4)) << "seed " << seed;
        const float centre_x = container.getNodes()[1].getX();
        const float centre_y = container.getNodes()[0].getY();
        moved = moved || centre_x != WORLD_SIZE / 2.0f || centre_y != WORLD_SIZE / 2.0f;
        // the corner blocks keep a full spacing to the border on every side
        ASSERT_GE(centre_x, SPACING * 1.5f - fixtures::EPSILON) << "seed " << seed;
        ASSERT_LE(centre_x, WORLD_SIZE - SPACING * 1.5f + fixtures::EPSILON) << "seed " << seed;
        ASSERT_GE(centre_y, SPACING * 1.5f - fixtures::EPSILON) << "seed " << seed;
        ASSERT_LE(centre_y, WORLD_SIZE - SPACING * 1.5f + fixtures::EPSILON) << "seed " << seed;
        ASSERT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE)) << "seed " << seed;
    }
    EXPECT_TRUE(moved);
}

TEST(IntersectionGeometry, SameSeedGivesTheSameWorld)
{
    const IntersectionGenerator generator = intersectionGenerator(0.5f, RandomRange(10.0f, 12.0f),
                                                                  RandomRange(1.0f, 2.0f));

    EXPECT_TRUE(fixtures::containersEqual(generator(fixtures::SEED), generator(fixtures::SEED)));
    EXPECT_FALSE(fixtures::containersEqual(generator(1), generator(2)));
}
