// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/wfc_generator.hpp"
#include "murosimproc/wfc/preset.hpp"
#include "murosimproc/wfc/tiles.hpp"
#include "fixtures.hpp"
#include <stdexcept>

namespace
{
using murosimproc::generators::RandomRange;
using murosimproc::generators::WfcGenerator;
using murosimproc::obstacle::ShapeDistribution;
using murosimproc::wfc::Tiles;
using murosimproc::wfc::preset::PresetType;
using murosim::io::DataContainer;

constexpr size_t MIN_NODE_COUNT = 5;
constexpr float NODE_RADIUS = 0.25f;
constexpr float WORLD_SIZE = 12.0f; // >= spacing.max * pattern size 3 * minimum grid 3
const RandomRange SPACING(0.7f, 1.25f);
const RandomRange OBSTACLE_SIZE(0.3f, 1.5f);
const RandomRange OBSTACLE_DENSITY(0.3f, 0.7f);
const ShapeDistribution ALL_CIRCLES = {1.0f, 0.0f, 0.0f, 0.05f};

WfcGenerator wfcGenerator(const PresetType preset = PresetType::BLOCK, const RandomRange world_size = WORLD_SIZE,
                          const RandomRange spacing = SPACING)
{
    return WfcGenerator(MIN_NODE_COUNT, NODE_RADIUS, world_size, spacing, preset, OBSTACLE_SIZE, OBSTACLE_DENSITY,
                        ALL_CIRCLES);
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(WfcGeneratorConstructor, NameAndConfiguration)
{
    const WfcGenerator generator = wfcGenerator();

    EXPECT_EQ(generator.getName(), "wfc");
    EXPECT_FLOAT_EQ(generator.getObstacleDensity().min(), OBSTACLE_DENSITY.min());
    EXPECT_FLOAT_EQ(generator.getObstacleDensity().max(), OBSTACLE_DENSITY.max());
    EXPECT_FLOAT_EQ(generator.getObstacleSize().min(), OBSTACLE_SIZE.min());
    EXPECT_FLOAT_EQ(generator.getObstacleSize().max(), OBSTACLE_SIZE.max());
    EXPECT_FLOAT_EQ(generator.getShapeDistribution().circle.min(), 1.0f);
    EXPECT_EQ(generator.getTiles().getPatternSize(), static_cast<size_t>(3));
}

TEST(WfcGeneratorConstructor, TilesOverloadKeepsTheGivenTiles)
{
    const Tiles tiles(fixtures::makeImage({"    ", "    ", "    ", "    "}), 2);
    const WfcGenerator generator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, SPACING, tiles, OBSTACLE_SIZE,
                                 OBSTACLE_DENSITY, ALL_CIRCLES);

    EXPECT_EQ(generator.getTiles().getPatternSize(), static_cast<size_t>(2));
    EXPECT_EQ(generator.getTiles().getPatterns().size(), static_cast<size_t>(1));
}

// The grid needs at least 3 x 3 pattern cells: floor(world / (spacing.max * 3))
TEST(WfcGeneratorConstructor, WorldTooSmallForTheGridThrows)
{
    EXPECT_THROW(wfcGenerator(PresetType::BLOCK, 8.0f, 1.0f), std::invalid_argument); // 8 / 3 = 2
    EXPECT_NO_THROW(wfcGenerator(PresetType::BLOCK, 9.0f, 1.0f));                    // 9 / 3 = 3
}

TEST(WfcGeneratorConstructor, NegativeObstacleDensityThrows)
{
    EXPECT_THROW(WfcGenerator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, SPACING, PresetType::BLOCK, OBSTACLE_SIZE,
                              RandomRange(-0.1f, 0.5f), ALL_CIRCLES),
                 std::invalid_argument);
    EXPECT_NO_THROW(WfcGenerator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, SPACING, PresetType::BLOCK, OBSTACLE_SIZE,
                                 0.0f, ALL_CIRCLES));
}

// ============================================================================
// Generation Tests
// ============================================================================

TEST(WfcGeneratorGenerate, WorldIsUsable)
{
    const DataContainer container = wfcGenerator()(fixtures::SEED);

    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
    EXPECT_TRUE(fixtures::hasSquareBorder(container.getLines(), WORLD_SIZE));
    EXPECT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE));
}

TEST(WfcGeneratorGenerate, ExactlyTheRequestedNodeCount)
{
    const DataContainer container = wfcGenerator()(fixtures::SEED);

    EXPECT_EQ(container.getNodes().size(), MIN_NODE_COUNT);
}

TEST(WfcGeneratorGenerate, ObstacleZonesBecomeCirclesInsideTheWorld)
{
    const DataContainer container = wfcGenerator(PresetType::OBSTACLE)(fixtures::SEED);

    ASSERT_FALSE(container.getCircles().empty());
    for (size_t i = 0; i < container.getCircles().size(); i++)
    {
        const murosim::geometry::Circle &circle = container.getCircles()[i];
        EXPECT_GE(circle.getX() - circle.getRadius(), -fixtures::EPSILON) << "circle " << i;
        EXPECT_GE(circle.getY() - circle.getRadius(), -fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getX() + circle.getRadius(), WORLD_SIZE + fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getY() + circle.getRadius(), WORLD_SIZE + fixtures::EPSILON) << "circle " << i;
    }
}

TEST(WfcGeneratorGenerate, EveryPresetGeneratesAUsableWorld)
{
    for (size_t i = 0; i < murosimproc::wfc::preset::PRESET_COUNT; i++)
    {
        const DataContainer container = wfcGenerator(static_cast<PresetType>(i))(fixtures::SEED);
        ASSERT_TRUE(container.isValid()) << "preset " << i;
        ASSERT_GE(container.getNodes().size(), MIN_NODE_COUNT) << "preset " << i;
        ASSERT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE)) << "preset " << i;
    }
}

TEST(WfcGeneratorGenerate, SameSeedGivesTheSameWorld)
{
    const WfcGenerator generator = wfcGenerator(PresetType::BLOCK, RandomRange(12.0f, 14.0f));

    EXPECT_TRUE(fixtures::containersEqual(generator(fixtures::SEED), generator(fixtures::SEED)));
    EXPECT_FALSE(fixtures::containersEqual(generator(1), generator(2)));
}
