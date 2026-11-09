// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/obstacle/convert_shapes.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <vector>

namespace
{
using murosimproc::generators::RandomRange;
using murosimproc::obstacle::convertShapes;
using murosimproc::obstacle::ShapeDistribution;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::random::RandomGenerator;

constexpr size_t OBSTACLES = 40;
constexpr float RADIUS = 1.0f;
constexpr float SPACING = 3.0f;
constexpr float MIN_RADIUS = 0.1f; // decompose needs radius > 1.5 * MIN_RADIUS
constexpr float MIN_POLYGON_RADIUS_LARGE = 2.0f * RADIUS;
constexpr float DECOMPOSE_THRESHOLD = 1.5f; // from convert_shapes.cpp
const ShapeDistribution ALL_CIRCLES = {1.0f, 0.0f, 0.0f, 0.05f};
const ShapeDistribution ALL_POLYGONS = {0.0f, 1.0f, 0.0f, 0.05f};
const ShapeDistribution ALWAYS_DECOMPOSE = {1.0f, 0.0f, 1.0f, 0.05f};

/// Bounding circles of radius `radius` in a row, SPACING apart.
std::vector<Circle> placedRow(const size_t count, const float radius = RADIUS)
{
    std::vector<Circle> placed;
    for (size_t i = 0; i < count; i++)
    {
        placed.emplace_back(static_cast<float>(i) * SPACING, 0.0f, radius);
    }
    return placed;
}

struct Shapes
{
    std::vector<Line> lines;
    std::vector<Circle> circles;
};

Shapes convert(const std::vector<Circle> &placed, const ShapeDistribution &shape, const float min_radius = MIN_RADIUS,
               const uint64_t seed = fixtures::SEED)
{
    Shapes shapes;
    RandomGenerator random_generator(seed);
    convertShapes(placed, shape, min_radius, random_generator, shapes.lines, shapes.circles);
    return shapes;
}

/// Index of the placed circle that contains `point`, or placed.size().
size_t containing(const std::vector<Circle> &placed, const Eigen::Vector2f &point)
{
    for (size_t i = 0; i < placed.size(); i++)
    {
        if ((point - placed[i].getPoint()).norm() <= placed[i].getRadius() + fixtures::EPSILON)
        {
            return i;
        }
    }
    return placed.size();
}

void expectSameCircles(const std::vector<Circle> &actual, const std::vector<Circle> &expected)
{
    ASSERT_EQ(actual.size(), expected.size());
    for (size_t i = 0; i < actual.size(); i++)
    {
        EXPECT_FLOAT_EQ(actual[i].getX(), expected[i].getX()) << "circle " << i;
        EXPECT_FLOAT_EQ(actual[i].getY(), expected[i].getY()) << "circle " << i;
        EXPECT_FLOAT_EQ(actual[i].getRadius(), expected[i].getRadius()) << "circle " << i;
    }
}
} // namespace

// ============================================================================
// Shape Weight Tests
// ============================================================================

TEST(ConvertShapesWeights, AllCircleWeightKeepsEveryBoundingCircle)
{
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, ALL_CIRCLES);

    EXPECT_TRUE(shapes.lines.empty());
    expectSameCircles(shapes.circles, placed);
}

TEST(ConvertShapesWeights, AllPolygonWeightTurnsEveryObstacleIntoAClosedPolygon)
{
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, ALL_POLYGONS);

    EXPECT_TRUE(shapes.circles.empty());
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::closedLoops(shapes.lines);
    ASSERT_EQ(loops.size(), OBSTACLES);
    std::vector<bool> covered(placed.size(), false);
    size_t loop_lines = 0;
    for (size_t i = 0; i < loops.size(); i++)
    {
        loop_lines += loops[i].size();
        ASSERT_GE(loops[i].size(), static_cast<size_t>(3)) << "loop " << i;
        ASSERT_LE(loops[i].size(), static_cast<size_t>(8)) << "loop " << i;
        const size_t owner = containing(placed, loops[i][0]);
        ASSERT_LT(owner, placed.size()) << "loop " << i;
        for (size_t j = 0; j < loops[i].size(); j++)
        {
            ASSERT_EQ(containing(placed, loops[i][j]), owner) << "loop " << i << " vertex " << j;
        }
        covered[owner] = true;
    }
    EXPECT_EQ(loop_lines, shapes.lines.size()); // every line belongs to a loop
    for (size_t i = 0; i < covered.size(); i++)
    {
        ASSERT_TRUE(covered[i]) << "obstacle " << i;
    }
}

TEST(ConvertShapesWeights, EqualWeightsProduceBothKinds)
{
    const ShapeDistribution shape = {0.5f, 0.5f, 0.0f, 0.05f};

    const Shapes shapes = convert(placedRow(OBSTACLES), shape);

    EXPECT_FALSE(shapes.circles.empty());
    EXPECT_FALSE(shapes.lines.empty());
    EXPECT_EQ(shapes.circles.size() + fixtures::closedLoops(shapes.lines).size(), OBSTACLES);
}

TEST(ConvertShapesWeights, ZeroWeightsFallBackToEqualProbability)
{
    const ShapeDistribution shape = {0.0f, 0.0f, 0.0f, 0.05f};

    const Shapes shapes = convert(placedRow(OBSTACLES), shape);

    EXPECT_FALSE(shapes.circles.empty());
    EXPECT_FALSE(shapes.lines.empty());
    EXPECT_EQ(shapes.circles.size() + fixtures::closedLoops(shapes.lines).size(), OBSTACLES);
}

TEST(ConvertShapesWeights, ObstaclesBelowThePolygonRadiusStayCircles)
{
    ShapeDistribution shape = ALL_POLYGONS;
    shape.min_polygon_radius = MIN_POLYGON_RADIUS_LARGE;
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, shape);

    EXPECT_TRUE(shapes.lines.empty());
    expectSameCircles(shapes.circles, placed);
}

TEST(ConvertShapesWeights, WeightsAreDrawnFromTheirRanges)
{
    // Circle weight anywhere in [0, 1] against a polygon weight of 0: always circles
    const ShapeDistribution shape = {RandomRange(0.0f, 1.0f), 0.0f, 0.0f, 0.05f};
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, shape);

    EXPECT_TRUE(shapes.lines.empty());
    expectSameCircles(shapes.circles, placed);
}

// ============================================================================
// Decompose Tests
// ============================================================================

TEST(ConvertShapesDecompose, LargeObstaclesSplitIntoSmallerCirclesInside)
{
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, ALWAYS_DECOMPOSE);

    EXPECT_TRUE(shapes.lines.empty());
    ASSERT_GT(shapes.circles.size(), OBSTACLES); // on average 2 to 5 pieces each
    ASSERT_LE(shapes.circles.size(), 5 * OBSTACLES);
    for (size_t i = 0; i < shapes.circles.size(); i++)
    {
        const size_t owner = containing(placed, shapes.circles[i].getPoint());
        ASSERT_LT(owner, placed.size()) << "circle " << i;
        const float distance = (shapes.circles[i].getPoint() - placed[owner].getPoint()).norm();
        ASSERT_LE(distance + shapes.circles[i].getRadius(), RADIUS + fixtures::EPSILON) << "circle " << i;
        ASSERT_LE(shapes.circles[i].getRadius(), RADIUS / 2.0f + fixtures::EPSILON) << "circle " << i;
        ASSERT_GT(shapes.circles[i].getRadius(), 0.0f) << "circle " << i;
    }
}

TEST(ConvertShapesDecompose, PiecesDoNotOverlap)
{
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, ALWAYS_DECOMPOSE);

    for (size_t i = 0; i < shapes.circles.size(); i++)
    {
        for (size_t j = 0; j < i; j++)
        {
            const float distance = (shapes.circles[i].getPoint() - shapes.circles[j].getPoint()).norm();
            ASSERT_GE(distance, shapes.circles[i].getRadius() + shapes.circles[j].getRadius() - fixtures::EPSILON)
                << "circles " << i << " and " << j;
        }
    }
}

TEST(ConvertShapesDecompose, ObstaclesAtOrBelowTheThresholdAreNotSplit)
{
    const std::vector<Circle> placed = placedRow(OBSTACLES, MIN_RADIUS * DECOMPOSE_THRESHOLD);

    const Shapes shapes = convert(placed, ALWAYS_DECOMPOSE);

    EXPECT_TRUE(shapes.lines.empty());
    expectSameCircles(shapes.circles, placed);
}

TEST(ConvertShapesDecompose, ZeroProbabilityNeverSplits)
{
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, ALL_CIRCLES);

    expectSameCircles(shapes.circles, placed);
}

TEST(ConvertShapesDecompose, PiecesRollCircleOrPolygonIndependently)
{
    const ShapeDistribution shape = {0.5f, 0.5f, 1.0f, 0.05f};
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes shapes = convert(placed, shape);

    EXPECT_FALSE(shapes.circles.empty());
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::closedLoops(shapes.lines);
    EXPECT_FALSE(loops.empty());
    EXPECT_GT(shapes.circles.size() + loops.size(), OBSTACLES);
    for (size_t i = 0; i < loops.size(); i++)
    {
        for (size_t j = 0; j < loops[i].size(); j++)
        {
            ASSERT_LT(containing(placed, loops[i][j]), placed.size()) << "loop " << i << " vertex " << j;
        }
    }
}

// ============================================================================
// Output Tests
// ============================================================================

TEST(ConvertShapesOutput, EmptyInputAddsNothing)
{
    const Shapes shapes = convert({}, ALL_POLYGONS);

    EXPECT_TRUE(shapes.lines.empty());
    EXPECT_TRUE(shapes.circles.empty());
}

TEST(ConvertShapesOutput, ShapesAreAppended)
{
    Shapes shapes;
    shapes.lines.emplace_back(9.0f, 9.0f, 8.0f, 8.0f);
    shapes.circles.emplace_back(7.0f, 7.0f, 0.5f);
    RandomGenerator random_generator(fixtures::SEED);

    convertShapes(placedRow(2), ALL_CIRCLES, MIN_RADIUS, random_generator, shapes.lines, shapes.circles);

    ASSERT_EQ(shapes.lines.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shapes.lines[0].getPoint1X(), 9.0f);
    ASSERT_EQ(shapes.circles.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(shapes.circles[0].getX(), 7.0f);
    EXPECT_FLOAT_EQ(shapes.circles[1].getX(), 0.0f);
    EXPECT_FLOAT_EQ(shapes.circles[2].getX(), SPACING);
}

TEST(ConvertShapesOutput, SameSeedGivesTheSameShapes)
{
    const ShapeDistribution shape = {0.5f, 0.5f, 0.5f, 0.05f};
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes first = convert(placed, shape, MIN_RADIUS, 7);
    const Shapes second = convert(placed, shape, MIN_RADIUS, 7);

    expectSameCircles(first.circles, second.circles);
    ASSERT_EQ(first.lines.size(), second.lines.size());
    for (size_t i = 0; i < first.lines.size(); i++)
    {
        ASSERT_FLOAT_EQ(first.lines[i].getPoint1X(), second.lines[i].getPoint1X()) << "line " << i;
        ASSERT_FLOAT_EQ(first.lines[i].getPoint2Y(), second.lines[i].getPoint2Y()) << "line " << i;
    }
}

TEST(ConvertShapesOutput, DifferentSeedsGiveDifferentShapes)
{
    const ShapeDistribution shape = {0.5f, 0.5f, 0.5f, 0.05f};
    const std::vector<Circle> placed = placedRow(OBSTACLES);

    const Shapes first = convert(placed, shape, MIN_RADIUS, 7);
    const Shapes second = convert(placed, shape, MIN_RADIUS, 8);

    ASSERT_FALSE(first.circles.empty());
    ASSERT_FALSE(second.circles.empty());
    EXPECT_TRUE(first.lines.size() != second.lines.size() || first.circles.size() != second.circles.size() ||
                first.circles[0].getRadius() != second.circles[0].getRadius());
}
