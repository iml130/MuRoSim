// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/maze_generator.hpp"
#include "fixtures.hpp"
#include <stdexcept>

namespace
{
using murosimproc::generators::MazeGenerator;
using murosimproc::generators::RandomRange;
using murosim::io::DataContainer;

constexpr size_t MIN_NODE_COUNT = 5;
constexpr float NODE_RADIUS = 0.25f;
constexpr float WORLD_SIZE = 10.0f;
const RandomRange SPACING(0.7f, 1.25f);
const RandomRange WALL_THICKNESS(0.1f, 0.3f);
const RandomRange WALL_REMOVAL_RATE(0.0f, 0.3f);

MazeGenerator mazeGenerator(const RandomRange world_size = WORLD_SIZE, const RandomRange spacing = SPACING,
                            const RandomRange wall_thickness = WALL_THICKNESS,
                            const RandomRange wall_removal_rate = WALL_REMOVAL_RATE)
{
    return MazeGenerator(MIN_NODE_COUNT, NODE_RADIUS, world_size, spacing, wall_thickness, wall_removal_rate);
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(MazeGeneratorConstructor, NameAndConfiguration)
{
    const MazeGenerator generator = mazeGenerator();

    EXPECT_EQ(generator.getName(), "maze");
    EXPECT_FLOAT_EQ(generator.getWallThickness().min(), WALL_THICKNESS.min());
    EXPECT_FLOAT_EQ(generator.getWallThickness().max(), WALL_THICKNESS.max());
    EXPECT_FLOAT_EQ(generator.getWallRemovalRate().min(), WALL_REMOVAL_RATE.min());
    EXPECT_FLOAT_EQ(generator.getWallRemovalRate().max(), WALL_REMOVAL_RATE.max());
}

TEST(MazeGeneratorConstructor, WallRemovalDefaultsToZero)
{
    const MazeGenerator generator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, SPACING, WALL_THICKNESS);

    EXPECT_FLOAT_EQ(generator.getWallRemovalRate().min(), 0.0f);
    EXPECT_FLOAT_EQ(generator.getWallRemovalRate().max(), 0.0f);
}

// The maze needs at least 3 cells per side: floor(world) / (spacing + wall)
TEST(MazeGeneratorConstructor, WorldTooSmallForThreeCellsThrows)
{
    EXPECT_THROW(mazeGenerator(8.0f, RandomRange(2.0f, 3.0f), 1.0f), std::runtime_error); // 8 / 4 = 2
    EXPECT_NO_THROW(mazeGenerator(12.0f, RandomRange(2.0f, 3.0f), 1.0f));                 // 12 / 4 = 3
}

// ============================================================================
// Generation Tests
// ============================================================================

TEST(MazeGeneratorGenerate, WorldIsUsable)
{
    const DataContainer container = mazeGenerator()(fixtures::SEED);

    EXPECT_TRUE(container.getCircles().empty());
    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
}

TEST(MazeGeneratorGenerate, GeometryIsScaledToTheWorldSize)
{
    const DataContainer container = mazeGenerator()(fixtures::SEED);

    EXPECT_TRUE(fixtures::hasSquareBorder(container.getLines(), WORLD_SIZE));
    EXPECT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE));
    EXPECT_GT(container.getLines().size(), static_cast<size_t>(4));
}

TEST(MazeGeneratorGenerate, WallLinesAreAxisAligned)
{
    const DataContainer container = mazeGenerator()(fixtures::SEED);

    for (size_t i = 0; i < container.getLines().size(); i++)
    {
        const murosim::geometry::Line &line = container.getLines()[i];
        ASSERT_TRUE(line.getPoint1X() == line.getPoint2X() || line.getPoint1Y() == line.getPoint2Y()) << "line " << i;
    }
}

TEST(MazeGeneratorGenerate, NodesKeepTheirRadiusAfterScaling)
{
    const DataContainer container = mazeGenerator()(fixtures::SEED);

    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        EXPECT_NEAR(container.getNodes()[i].getRadius(), NODE_RADIUS, fixtures::EPSILON) << "node " << i;
    }
}

// Without inner walls the free area is one rectangle: its outline plus the border
TEST(MazeGeneratorGenerate, RemovingAllWallsLeavesOneOpenRectangle)
{
    const DataContainer container = mazeGenerator(WORLD_SIZE, SPACING, WALL_THICKNESS, 1.0f)(fixtures::SEED);

    EXPECT_EQ(container.getLines().size(), static_cast<size_t>(4 + 4));
    EXPECT_EQ(fixtures::undirectedLoops(container.getLines()).size(), static_cast<size_t>(2));
    EXPECT_GE(container.getNodes().size(), MIN_NODE_COUNT);
}

TEST(MazeGeneratorGenerate, SameSeedGivesTheSameWorld)
{
    const MazeGenerator generator = mazeGenerator(RandomRange(10.0f, 12.0f));

    EXPECT_TRUE(fixtures::containersEqual(generator(fixtures::SEED), generator(fixtures::SEED)));
    EXPECT_FALSE(fixtures::containersEqual(generator(1), generator(2)));
}
