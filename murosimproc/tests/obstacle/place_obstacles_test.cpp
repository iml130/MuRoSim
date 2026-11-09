// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/obstacle/place_obstacles.hpp"
#include "fixtures.hpp"
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
using murosimproc::obstacle::placeObstacles;
using murosimproc::obstacle::ShapeDistribution;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::random::RandomGenerator;

const Eigen::Vector2f AREA_MIN(0.0f, 0.0f);
const Eigen::Vector2f AREA_MAX(10.0f, 10.0f);
constexpr float DENSITY = 0.2f; // 20 obstacles on the 100 m² area, no fractional part
constexpr size_t EXPECTED_OBSTACLES = 20;
constexpr float SCALE_MIN = 0.4f; // diameters: radii 0.2 .. 0.5
constexpr float SCALE_MAX = 1.0f;
constexpr float RADIUS_MIN = SCALE_MIN / 2.0f;
constexpr float RADIUS_MAX = SCALE_MAX / 2.0f;
// From place_obstacles.cpp: the random fallback shrinks the radius by the
// factor up to three times and never below the fraction of the minimum.
constexpr float RADIUS_REDUCTION_FACTOR = 0.7f;
constexpr size_t MAX_RADIUS_REDUCTION_ROUNDS = 3;
constexpr float MIN_RADIUS_FRACTION = 0.3f;
constexpr uint32_t SEEDS = 40;
const ShapeDistribution ALL_CIRCLES = {1.0f, 0.0f, 0.0f, 0.05f};
const ShapeDistribution ALL_POLYGONS = {0.0f, 1.0f, 0.0f, 0.05f};

struct Shapes
{
    std::vector<Line> lines;
    std::vector<Circle> circles;
};

Shapes place(const float density, const ShapeDistribution &shape, const uint64_t seed = fixtures::SEED,
             const float scale_min = SCALE_MIN, const float scale_max = SCALE_MAX,
             const Eigen::Vector2f &area_min = AREA_MIN, const Eigen::Vector2f &area_max = AREA_MAX)
{
    Shapes shapes;
    RandomGenerator random_generator(seed);
    placeObstacles(density, scale_min, scale_max, shape, area_min, area_max, random_generator, shapes.lines,
                   shapes.circles);
    return shapes;
}

void assertCirclesInside(const std::vector<Circle> &circles, const Eigen::Vector2f &area_min,
                         const Eigen::Vector2f &area_max)
{
    for (size_t i = 0; i < circles.size(); i++)
    {
        const Eigen::Vector2f centre = circles[i].getPoint();
        const float radius = circles[i].getRadius();
        ASSERT_GE(centre.x() - radius, area_min.x() - fixtures::EPSILON) << "circle " << i;
        ASSERT_GE(centre.y() - radius, area_min.y() - fixtures::EPSILON) << "circle " << i;
        ASSERT_LE(centre.x() + radius, area_max.x() + fixtures::EPSILON) << "circle " << i;
        ASSERT_LE(centre.y() + radius, area_max.y() + fixtures::EPSILON) << "circle " << i;
    }
}

void assertNoOverlap(const std::vector<Circle> &circles)
{
    for (size_t i = 0; i < circles.size(); i++)
    {
        for (size_t j = 0; j < i; j++)
        {
            const float distance = (circles[i].getPoint() - circles[j].getPoint()).norm();
            ASSERT_GE(distance, circles[i].getRadius() + circles[j].getRadius() - fixtures::EPSILON)
                << "circles " << i << " and " << j;
        }
    }
}
} // namespace

// ============================================================================
// Argument Tests
// ============================================================================

TEST(PlaceObstaclesArguments, ZeroDensityPlacesNothing)
{
    const Shapes shapes = place(0.0f, ALL_CIRCLES);

    EXPECT_TRUE(shapes.circles.empty());
    EXPECT_TRUE(shapes.lines.empty());
}

TEST(PlaceObstaclesArguments, NonPositiveScalePlacesNothing)
{
    EXPECT_TRUE(place(DENSITY, ALL_CIRCLES, fixtures::SEED, 0.0f, SCALE_MAX).circles.empty());
    EXPECT_TRUE(place(DENSITY, ALL_CIRCLES, fixtures::SEED, -SCALE_MIN, SCALE_MAX).circles.empty());
    EXPECT_TRUE(place(DENSITY, ALL_CIRCLES, fixtures::SEED, SCALE_MIN, 0.0f).circles.empty());
}

TEST(PlaceObstaclesArguments, ReversedScalesPlaceNothing)
{
    const Shapes shapes = place(DENSITY, ALL_CIRCLES, fixtures::SEED, SCALE_MAX, SCALE_MIN);

    EXPECT_TRUE(shapes.circles.empty());
    EXPECT_TRUE(shapes.lines.empty());
}

// Equal scales draw one radius; obstacles that found no free hex candidate
// are retried at 0.7, 0.49 and 0.343 of it (the random fallback), nothing else.
TEST(PlaceObstaclesArguments, EqualScalesGiveOneRadiusUpToTheFallbackReduction)
{
    const Shapes shapes = place(DENSITY, ALL_CIRCLES, fixtures::SEED, SCALE_MAX, SCALE_MAX);

    ASSERT_EQ(shapes.circles.size(), EXPECTED_OBSTACLES);
    size_t reduced = 0;
    for (size_t i = 0; i < shapes.circles.size(); i++)
    {
        const float radius = shapes.circles[i].getRadius();
        bool known = false;
        float expected = RADIUS_MAX;
        for (size_t round = 0; round <= MAX_RADIUS_REDUCTION_ROUNDS; round++)
        {
            known = known || std::abs(radius - expected) < fixtures::EPSILON;
            expected *= RADIUS_REDUCTION_FACTOR;
        }
        ASSERT_TRUE(known) << "circle " << i << " radius " << radius;
        reduced += radius < RADIUS_MAX ? 1 : 0;
    }
    EXPECT_LT(reduced, EXPECTED_OBSTACLES); // the hex candidates place most of them at full size
}

// ============================================================================
// Placement Tests
// ============================================================================

TEST(PlaceObstaclesPlacement, CountFollowsTheDensity)
{
    const Shapes shapes = place(DENSITY, ALL_CIRCLES);

    EXPECT_EQ(shapes.circles.size(), EXPECTED_OBSTACLES);
    EXPECT_TRUE(shapes.lines.empty());
}

TEST(PlaceObstaclesPlacement, FractionalExpectationRoundsUpOrDown)
{
    // 2 x 2 m² at 0.125 / m² expects 0.5 obstacles: some seeds give none, some one
    const Eigen::Vector2f area_max(2.0f, 2.0f);
    size_t with_obstacle = 0;
    for (uint32_t seed = 0; seed < SEEDS; seed++)
    {
        const Shapes shapes = place(0.125f, ALL_CIRCLES, seed, SCALE_MIN, SCALE_MAX, AREA_MIN, area_max);
        ASSERT_LE(shapes.circles.size(), static_cast<size_t>(1)) << "seed " << seed;
        with_obstacle += shapes.circles.size();
    }
    EXPECT_GT(with_obstacle, static_cast<size_t>(0));
    EXPECT_LT(with_obstacle, static_cast<size_t>(SEEDS));
}

TEST(PlaceObstaclesPlacement, CirclesLieInsideTheArea)
{
    const Shapes shapes = place(DENSITY, ALL_CIRCLES);

    ASSERT_EQ(shapes.circles.size(), EXPECTED_OBSTACLES);
    assertCirclesInside(shapes.circles, AREA_MIN, AREA_MAX);
}

TEST(PlaceObstaclesPlacement, CirclesDoNotOverlap)
{
    const Shapes shapes = place(DENSITY, ALL_CIRCLES);

    ASSERT_EQ(shapes.circles.size(), EXPECTED_OBSTACLES);
    assertNoOverlap(shapes.circles);
}

TEST(PlaceObstaclesPlacement, RadiiStayWithinTheScaleRange)
{
    const Shapes shapes = place(DENSITY, ALL_CIRCLES);

    ASSERT_EQ(shapes.circles.size(), EXPECTED_OBSTACLES);
    for (size_t i = 0; i < shapes.circles.size(); i++)
    {
        ASSERT_GE(shapes.circles[i].getRadius(), MIN_RADIUS_FRACTION * RADIUS_MIN) << "circle " << i;
        ASSERT_LE(shapes.circles[i].getRadius(), RADIUS_MAX) << "circle " << i;
    }
}

TEST(PlaceObstaclesPlacement, AreaAwayFromTheOriginIsRespected)
{
    const Eigen::Vector2f area_min(-15.0f, 3.0f);
    const Eigen::Vector2f area_max(-5.0f, 13.0f);

    const Shapes shapes = place(DENSITY, ALL_CIRCLES, fixtures::SEED, SCALE_MIN, SCALE_MAX, area_min, area_max);

    ASSERT_EQ(shapes.circles.size(), EXPECTED_OBSTACLES);
    assertCirclesInside(shapes.circles, area_min, area_max);
    assertNoOverlap(shapes.circles);
}

// A 1 x 1 m² area asked for 100 obstacles of radius 0.2 .. 0.5 can only hold
// a few: the rest is dropped instead of overlapping or leaving the area.
TEST(PlaceObstaclesPlacement, CrowdedAreaDropsWhatDoesNotFit)
{
    const Eigen::Vector2f area_max(1.0f, 1.0f);

    const Shapes shapes = place(100.0f, ALL_CIRCLES, fixtures::SEED, SCALE_MIN, SCALE_MAX, AREA_MIN, area_max);

    ASSERT_FALSE(shapes.circles.empty());
    EXPECT_LT(shapes.circles.size(), static_cast<size_t>(100));
    assertCirclesInside(shapes.circles, AREA_MIN, area_max);
    assertNoOverlap(shapes.circles);
}

TEST(PlaceObstaclesPlacement, RadiiAreClampedToTheArea)
{
    // 1 x 1 m² with 4 m obstacles: radii are clamped to 0.4 .. 0.45 of the short side
    const Eigen::Vector2f area_max(1.0f, 1.0f);

    const Shapes shapes = place(1.0f, ALL_CIRCLES, fixtures::SEED, 4.0f, 4.0f, AREA_MIN, area_max);

    ASSERT_EQ(shapes.circles.size(), static_cast<size_t>(1));
    EXPECT_GE(shapes.circles[0].getRadius(), MIN_RADIUS_FRACTION * 0.4f);
    EXPECT_LE(shapes.circles[0].getRadius(), 0.45f);
    assertCirclesInside(shapes.circles, AREA_MIN, area_max);
}

// ============================================================================
// Shape Tests
// ============================================================================

TEST(PlaceObstaclesShapes, PolygonsAreClosedLoopsInsideTheArea)
{
    const Shapes shapes = place(DENSITY, ALL_POLYGONS);

    EXPECT_TRUE(shapes.circles.empty());
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::closedLoops(shapes.lines);
    EXPECT_EQ(loops.size(), EXPECTED_OBSTACLES);
    for (size_t i = 0; i < shapes.lines.size(); i++)
    {
        for (const Eigen::Vector2f &point : {shapes.lines[i].getPoint1(), shapes.lines[i].getPoint2()})
        {
            ASSERT_GE(point.x(), AREA_MIN.x() - fixtures::EPSILON) << "line " << i;
            ASSERT_GE(point.y(), AREA_MIN.y() - fixtures::EPSILON) << "line " << i;
            ASSERT_LE(point.x(), AREA_MAX.x() + fixtures::EPSILON) << "line " << i;
            ASSERT_LE(point.y(), AREA_MAX.y() + fixtures::EPSILON) << "line " << i;
        }
    }
}

TEST(PlaceObstaclesShapes, DefaultDistributionMixesCirclesAndPolygons)
{
    const Shapes shapes = place(DENSITY, ShapeDistribution());

    EXPECT_FALSE(shapes.circles.empty());
    EXPECT_FALSE(shapes.lines.empty());
}

// ============================================================================
// Output Tests
// ============================================================================

TEST(PlaceObstaclesOutput, ShapesAreAppended)
{
    Shapes shapes;
    shapes.lines.emplace_back(20.0f, 20.0f, 21.0f, 21.0f);
    shapes.circles.emplace_back(30.0f, 30.0f, 0.5f);
    RandomGenerator random_generator(fixtures::SEED);

    placeObstacles(DENSITY, SCALE_MIN, SCALE_MAX, ALL_CIRCLES, AREA_MIN, AREA_MAX, random_generator, shapes.lines,
                   shapes.circles);

    ASSERT_EQ(shapes.lines.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shapes.lines[0].getPoint1X(), 20.0f);
    ASSERT_EQ(shapes.circles.size(), EXPECTED_OBSTACLES + 1);
    EXPECT_FLOAT_EQ(shapes.circles[0].getX(), 30.0f);
}

TEST(PlaceObstaclesOutput, SameSeedGivesTheSamePlacement)
{
    const Shapes first = place(DENSITY, ALL_CIRCLES, 7);
    const Shapes second = place(DENSITY, ALL_CIRCLES, 7);

    ASSERT_EQ(first.circles.size(), second.circles.size());
    for (size_t i = 0; i < first.circles.size(); i++)
    {
        ASSERT_FLOAT_EQ(first.circles[i].getX(), second.circles[i].getX()) << "circle " << i;
        ASSERT_FLOAT_EQ(first.circles[i].getY(), second.circles[i].getY()) << "circle " << i;
        ASSERT_FLOAT_EQ(first.circles[i].getRadius(), second.circles[i].getRadius()) << "circle " << i;
    }
}

TEST(PlaceObstaclesOutput, DifferentSeedsGiveDifferentPlacements)
{
    const Shapes first = place(DENSITY, ALL_CIRCLES, 7);
    const Shapes second = place(DENSITY, ALL_CIRCLES, 8);

    ASSERT_FALSE(first.circles.empty());
    ASSERT_FALSE(second.circles.empty());
    EXPECT_TRUE(first.circles[0].getX() != second.circles[0].getX() ||
                first.circles[0].getY() != second.circles[0].getY());
}
