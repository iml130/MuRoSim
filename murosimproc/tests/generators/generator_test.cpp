// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/generator.hpp"
#include "murosimproc/generators/maze_generator.hpp"
#include "murosimproc/generators/wfc_generator.hpp"
#include "murosimproc/wfc/tiles.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <stdexcept>
#include <string>

namespace
{
using murosimproc::generators::Generator;
using murosimproc::generators::MazeGenerator;
using murosimproc::generators::RandomRange;
using murosimproc::generators::WfcGenerator;
using murosimproc::obstacle::ShapeDistribution;
using murosimproc::wfc::Tiles;
using murosim::io::DataContainer;
using murosim::random::RandomGenerator;

// The base class is abstract; the maze generator passes every argument
// through to it unchanged, so it stands in for the shared behaviour.
constexpr size_t MIN_NODE_COUNT = 5;
constexpr float NODE_RADIUS = 0.25f;
const RandomRange WORLD_SIZE(10.0f, 12.0f);
const RandomRange SPACING(0.7f, 1.25f);
const RandomRange WALL_THICKNESS(0.1f, 0.3f);
constexpr size_t SEED_DIGITS = 18; // 2 hex-digit slots per byte of the 64-bit seed plus 2

MazeGenerator mazeGenerator(const size_t min_node_count = MIN_NODE_COUNT, const float node_radius = NODE_RADIUS,
                            const RandomRange world_size = WORLD_SIZE, const RandomRange spacing = SPACING)
{
    return MazeGenerator(min_node_count, node_radius, world_size, spacing, WALL_THICKNESS);
}
} // namespace

// ============================================================================
// Validation Tests
// ============================================================================

TEST(GeneratorValidation, MinNodeCountBelowTwoThrows)
{
    EXPECT_THROW(mazeGenerator(1), std::invalid_argument);
    EXPECT_NO_THROW(mazeGenerator(2));
}

TEST(GeneratorValidation, NonPositiveNodeRadiusThrows)
{
    EXPECT_THROW(mazeGenerator(MIN_NODE_COUNT, 0.0f), std::invalid_argument);
    EXPECT_THROW(mazeGenerator(MIN_NODE_COUNT, -NODE_RADIUS), std::invalid_argument);
}

TEST(GeneratorValidation, NonPositiveWorldSizeThrows)
{
    EXPECT_THROW(mazeGenerator(MIN_NODE_COUNT, NODE_RADIUS, RandomRange(0.0f, 12.0f)), std::invalid_argument);
}

// Nodes must fit within corridors: spacing.min >= 2 * node_radius
TEST(GeneratorValidation, SpacingBelowTheNodeDiameterThrows)
{
    EXPECT_THROW(mazeGenerator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, RandomRange(0.4f, 1.0f)),
                 std::invalid_argument);
    EXPECT_NO_THROW(mazeGenerator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, RandomRange(0.5f, 1.0f)));
}

TEST(GeneratorValidation, SpacingLargerThanTheWorldThrows)
{
    EXPECT_THROW(mazeGenerator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, RandomRange(11.0f, 12.0f)),
                 std::invalid_argument);
}

// A 10 m world with 5 m spacing holds about 4 nodes; asking for 5 fails at
// construction instead of after 100 generation retries.
TEST(GeneratorValidation, InfeasibleNodeCountThrows)
{
    EXPECT_THROW(mazeGenerator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, RandomRange(2.0f, 5.0f)),
                 std::invalid_argument);
    EXPECT_NO_THROW(mazeGenerator(4, NODE_RADIUS, WORLD_SIZE, RandomRange(2.0f, 3.0f)));
}

// ============================================================================
// Getter Tests
// ============================================================================

TEST(GeneratorGetters, ConfigurationIsStored)
{
    const MazeGenerator generator = mazeGenerator();

    EXPECT_EQ(generator.getName(), "maze");
    EXPECT_EQ(generator.getMinNodeCount(), MIN_NODE_COUNT);
    EXPECT_FLOAT_EQ(generator.getNodeRadius(), NODE_RADIUS);
    EXPECT_FLOAT_EQ(generator.getWorldSize().min(), WORLD_SIZE.min());
    EXPECT_FLOAT_EQ(generator.getWorldSize().max(), WORLD_SIZE.max());
    EXPECT_FLOAT_EQ(generator.getSpacing().min(), SPACING.min());
    EXPECT_FLOAT_EQ(generator.getSpacing().max(), SPACING.max());
}

TEST(GeneratorGetters, NameWithSeedIsZeroPadded)
{
    const MazeGenerator generator = mazeGenerator();

    EXPECT_EQ(generator.getNameWithSeed(42), "maze_" + std::string(SEED_DIGITS - 2, '0') + "42");
    EXPECT_EQ(generator.getNameWithSeed(0), "maze_" + std::string(SEED_DIGITS, '0'));
}

// ============================================================================
// Generation Tests
// ============================================================================

TEST(GeneratorGenerate, SeedOverloadNamesTheWorldAfterTheSeed)
{
    const MazeGenerator generator = mazeGenerator();

    const DataContainer container = generator(3);

    EXPECT_EQ(container.getName(), generator.getNameWithSeed(3));
    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
}

TEST(GeneratorGenerate, SameSeedGivesTheSameWorld)
{
    const MazeGenerator generator = mazeGenerator();

    EXPECT_TRUE(fixtures::containersEqual(generator(fixtures::SEED), generator(fixtures::SEED)));
}

TEST(GeneratorGenerate, DifferentSeedsGiveDifferentWorlds)
{
    const MazeGenerator generator = mazeGenerator();

    const DataContainer first = generator(1);
    const DataContainer second = generator(2);

    EXPECT_NE(first.getName(), second.getName());
    EXPECT_FALSE(fixtures::containersEqual(first, second));
}

TEST(GeneratorGenerate, GeneratorOverloadDrawsTheSeed)
{
    const MazeGenerator generator = mazeGenerator();
    RandomGenerator random_generator(7);
    RandomGenerator reference(7);
    const uint64_t expected_seed = reference();

    const DataContainer container = generator(random_generator);

    EXPECT_EQ(container.getName(), generator.getNameWithSeed(expected_seed));
    EXPECT_TRUE(fixtures::containersEqual(container, generator(expected_seed)));
    EXPECT_EQ(random_generator(), reference()); // exactly one draw was consumed
}

TEST(GeneratorGenerate, SameSeededGeneratorsGiveTheSameWorld)
{
    const MazeGenerator generator = mazeGenerator();
    RandomGenerator first(7);
    RandomGenerator second(7);

    EXPECT_TRUE(fixtures::containersEqual(generator(first), generator(second)));
}

// A WFC tile set of nothing but walls never yields a free cell, so no
// attempt reaches the minimum node count.
TEST(GeneratorGenerate, ThrowsWhenNoAttemptReachesTheNodeCount)
{
    const Tiles walls(fixtures::makeImage({"###", "###", "###"}), 3);
    const WfcGenerator generator(2, NODE_RADIUS, 10.0f, 1.0f, walls, RandomRange(0.3f, 0.5f), 0.0f,
                                 ShapeDistribution());

    EXPECT_THROW(generator(fixtures::SEED), std::runtime_error);
    RandomGenerator random_generator(fixtures::SEED);
    EXPECT_THROW(generator(random_generator), std::runtime_error);
}
