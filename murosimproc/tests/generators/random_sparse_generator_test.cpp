// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/random_sparse_generator.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <stdexcept>

namespace
{
using murosim::math::PI;
using murosimproc::generators::RandomRange;
using murosimproc::generators::RandomSparseGenerator;
using murosimproc::obstacle::ShapeDistribution;
using murosim::io::DataContainer;

constexpr size_t MIN_NODE_COUNT = 5;
constexpr float NODE_RADIUS = 0.25f;
constexpr float WORLD_SIZE = 10.0f;
const RandomRange SPACING(0.7f, 1.25f);
const RandomRange OBSTACLE_SIZE(0.3f, 2.0f); // in node radii
constexpr float DENSITY_RATE = 1.0f;
const ShapeDistribution ALL_CIRCLES = {1.0f, 0.0f, 0.0f, 0.05f};

RandomSparseGenerator sparseGenerator(const ShapeDistribution &shape = ALL_CIRCLES,
                                      const RandomRange world_size = WORLD_SIZE, const RandomRange spacing = SPACING,
                                      const RandomRange obstacle_size = OBSTACLE_SIZE,
                                      const RandomRange density_rate = DENSITY_RATE)
{
    return RandomSparseGenerator(MIN_NODE_COUNT, NODE_RADIUS, world_size, spacing, obstacle_size, density_rate,
                                 shape);
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(RandomSparseGeneratorConstructor, NameAndConfiguration)
{
    const RandomSparseGenerator generator = sparseGenerator();

    EXPECT_EQ(generator.getName(), "random_sparse");
    EXPECT_FLOAT_EQ(generator.getObstacleSize().min(), OBSTACLE_SIZE.min());
    EXPECT_FLOAT_EQ(generator.getObstacleSize().max(), OBSTACLE_SIZE.max());
    EXPECT_FLOAT_EQ(generator.getPlacementDensityRate().min(), DENSITY_RATE);
    EXPECT_FLOAT_EQ(generator.getShapeDistribution().circle.min(), 1.0f);
    EXPECT_FLOAT_EQ(generator.getShapeDistribution().polygon.max(), 0.0f);
}

TEST(RandomSparseGeneratorConstructor, EntropyMapsToAShapeDistribution)
{
    const ShapeDistribution shape = RandomSparseGenerator::entropyToShapeDistribution(RandomRange(0.4f, 0.6f));

    EXPECT_FLOAT_EQ(shape.circle.min(), 0.4f);
    EXPECT_FLOAT_EQ(shape.circle.max(), 0.6f);
    EXPECT_FLOAT_EQ(shape.polygon.min(), 0.2f); // (1 - 0.6) / 2
    EXPECT_FLOAT_EQ(shape.polygon.max(), 0.3f); // (1 - 0.4) / 2
    EXPECT_FLOAT_EQ(shape.decompose.min(), 0.2f);
    EXPECT_FLOAT_EQ(shape.decompose.max(), 0.3f);
}

TEST(RandomSparseGeneratorConstructor, EntropyConstructorStoresTheMappedDistribution)
{
    const RandomSparseGenerator generator(MIN_NODE_COUNT, NODE_RADIUS, WORLD_SIZE, SPACING, OBSTACLE_SIZE,
                                          DENSITY_RATE, RandomRange(0.4f, 0.6f));

    EXPECT_FLOAT_EQ(generator.getShapeDistribution().circle.max(), 0.6f);
    EXPECT_FLOAT_EQ(generator.getShapeDistribution().polygon.min(), 0.2f);
}

TEST(RandomSparseGeneratorConstructor, InvalidConfigurationThrows)
{
    EXPECT_THROW(sparseGenerator(ALL_CIRCLES, WORLD_SIZE, SPACING, RandomRange(0.0f, 2.0f)), std::invalid_argument);
    EXPECT_THROW(sparseGenerator(ALL_CIRCLES, WORLD_SIZE, SPACING, OBSTACLE_SIZE, 0.0f), std::invalid_argument);
    EXPECT_THROW(sparseGenerator(ALL_CIRCLES, 3.0f, 1.0f), std::invalid_argument); // world <= 3 * spacing
    const ShapeDistribution negative = {-0.1f, 0.5f, 0.3f, 0.05f};
    EXPECT_THROW(sparseGenerator(negative), std::invalid_argument);
}

// ============================================================================
// Generation Tests
// ============================================================================

TEST(RandomSparseGeneratorGenerate, WorldIsUsable)
{
    const DataContainer container = sparseGenerator()(fixtures::SEED);

    fixtures::assertUsableWorld(container, MIN_NODE_COUNT, NODE_RADIUS);
    EXPECT_TRUE(fixtures::hasSquareBorder(container.getLines(), WORLD_SIZE));
}

TEST(RandomSparseGeneratorGenerate, ExactlyTheRequestedNodeCount)
{
    const DataContainer container = sparseGenerator()(fixtures::SEED);

    EXPECT_EQ(container.getNodes().size(), MIN_NODE_COUNT);
    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        EXPECT_FLOAT_EQ(container.getNodes()[i].getStartAngle(), 0.0f) << "node " << i;
        EXPECT_FLOAT_EQ(container.getNodes()[i].getStartBias(), PI) << "node " << i;
    }
}

TEST(RandomSparseGeneratorGenerate, ObstaclesAreCirclesInsideTheWorld)
{
    const DataContainer container = sparseGenerator()(fixtures::SEED);

    ASSERT_FALSE(container.getCircles().empty());
    EXPECT_EQ(container.getLines().size(), static_cast<size_t>(4)); // border only, no polygons
    for (size_t i = 0; i < container.getCircles().size(); i++)
    {
        const murosim::geometry::Circle &circle = container.getCircles()[i];
        EXPECT_GE(circle.getRadius(), OBSTACLE_SIZE.min() * NODE_RADIUS - fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getRadius(), OBSTACLE_SIZE.max() * NODE_RADIUS + fixtures::EPSILON) << "circle " << i;
        EXPECT_GE(circle.getX() - circle.getRadius(), -fixtures::EPSILON) << "circle " << i;
        EXPECT_GE(circle.getY() - circle.getRadius(), -fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getX() + circle.getRadius(), WORLD_SIZE + fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getY() + circle.getRadius(), WORLD_SIZE + fixtures::EPSILON) << "circle " << i;
    }
}

TEST(RandomSparseGeneratorGenerate, ObstaclesKeepTheSpacingApart)
{
    const DataContainer container = sparseGenerator()(fixtures::SEED);

    for (size_t i = 0; i < container.getCircles().size(); i++)
    {
        for (size_t j = 0; j < i; j++)
        {
            const murosim::geometry::Circle &a = container.getCircles()[i];
            const murosim::geometry::Circle &b = container.getCircles()[j];
            const float distance = (a.getPoint() - b.getPoint()).norm();
            ASSERT_GE(distance, a.getRadius() + b.getRadius() + SPACING.min() - fixtures::EPSILON)
                << "circles " << i << " and " << j;
        }
    }
}

TEST(RandomSparseGeneratorGenerate, NodesKeepClearOfObstaclesAndTheBorder)
{
    const DataContainer container = sparseGenerator()(fixtures::SEED);
    const float margin = NODE_RADIUS + SPACING.max() / 2.0f;

    for (size_t i = 0; i < container.getNodes().size(); i++)
    {
        const murosim::navigation::Node &node = container.getNodes()[i];
        EXPECT_GE(node.getX(), margin - fixtures::EPSILON) << "node " << i;
        EXPECT_LE(node.getX(), WORLD_SIZE - margin + fixtures::EPSILON) << "node " << i;
        EXPECT_GE(node.getY(), margin - fixtures::EPSILON) << "node " << i;
        EXPECT_LE(node.getY(), WORLD_SIZE - margin + fixtures::EPSILON) << "node " << i;
        for (size_t j = 0; j < container.getCircles().size(); j++)
        {
            const murosim::geometry::Circle &circle = container.getCircles()[j];
            const float distance = std::hypot(node.getX() - circle.getX(), node.getY() - circle.getY());
            ASSERT_GE(distance, circle.getRadius() + SPACING.max() / 2.0f + NODE_RADIUS - fixtures::EPSILON)
                << "node " << i << " circle " << j;
        }
    }
}

TEST(RandomSparseGeneratorGenerate, PolygonWeightProducesLines)
{
    const ShapeDistribution all_polygons = {0.0f, 1.0f, 0.0f, 0.05f};

    const DataContainer container = sparseGenerator(all_polygons)(fixtures::SEED);

    EXPECT_TRUE(container.getCircles().empty());
    EXPECT_GT(container.getLines().size(), static_cast<size_t>(4));
    EXPECT_TRUE(fixtures::linesInsideSquare(container.getLines(), WORLD_SIZE));
}

TEST(RandomSparseGeneratorGenerate, SameSeedGivesTheSameWorld)
{
    const RandomSparseGenerator generator = sparseGenerator(ShapeDistribution(), RandomRange(10.0f, 12.0f));

    EXPECT_TRUE(fixtures::containersEqual(generator(fixtures::SEED), generator(fixtures::SEED)));
    EXPECT_FALSE(fixtures::containersEqual(generator(1), generator(2)));
}
