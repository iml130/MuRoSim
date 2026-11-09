// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/util/geometry.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/random/random_generator.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::random::RandomGenerator;
using fixtures::EPSILON;
using fixtures::SEED;

constexpr float WORLD_SIZE = 10.0f;
constexpr float RADIUS = 4.0f;
const Circle DISK(1.0f, 2.0f, RADIUS);

/// Distance of the farthest line end point from the disk centre.
float farthestVertex(const std::vector<Line> &lines)
{
    float farthest = 0.0f;
    for (const Line &line : lines)
    {
        farthest = std::max(farthest, (line.getPoint1() - DISK.getPoint()).norm());
        farthest = std::max(farthest, (line.getPoint2() - DISK.getPoint()).norm());
    }
    return farthest;
}

/// Distance of the nearest line end point to the disk centre.
float nearestVertex(const std::vector<Line> &lines)
{
    float nearest = RADIUS;
    for (const Line &line : lines)
    {
        nearest = std::min(nearest, (line.getPoint1() - DISK.getPoint()).norm());
    }
    return nearest;
}

bool overlap(const Circle &a, const Circle &b)
{
    return (a.getPoint() - b.getPoint()).norm() < a.getRadius() + b.getRadius();
}

void assertInsideDiskAndDisjoint(const std::vector<Circle> &circles)
{
    for (size_t i = 0; i < circles.size(); i++)
    {
        ASSERT_LE((circles[i].getPoint() - DISK.getPoint()).norm() + circles[i].getRadius(), RADIUS + EPSILON)
            << "circle " << i << " leaves the disk";
        for (size_t j = i + 1; j < circles.size(); j++)
        {
            ASSERT_FALSE(overlap(circles[i], circles[j])) << "circles " << i << " and " << j << " overlap";
        }
    }
}
} // namespace

// ============================================================================
// Border Line Tests
// ============================================================================

TEST(UtilGeometryBorder, SquareBorderIsFourLinesAroundTheWorld)
{
    std::vector<Line> lines;

    murosimproc::util::geometry::addBorderLines(lines, WORLD_SIZE);

    ASSERT_EQ(lines.size(), static_cast<size_t>(4));
    EXPECT_FLOAT_EQ(lines[0].getPoint1X(), 0.0f); // bottom, left to right
    EXPECT_FLOAT_EQ(lines[0].getPoint2X(), WORLD_SIZE);
    EXPECT_FLOAT_EQ(lines[1].getPoint2Y(), WORLD_SIZE); // right, upwards
    EXPECT_FLOAT_EQ(lines[2].getPoint2X(), 0.0f);       // top, right to left
    EXPECT_FLOAT_EQ(lines[3].getPoint2Y(), 0.0f);       // left, downwards
    EXPECT_EQ(fixtures::closedLoops(lines).size(), static_cast<size_t>(1));
}

TEST(UtilGeometryBorder, RectangleBorderUsesBothSizes)
{
    std::vector<Line> lines;

    murosimproc::util::geometry::addBorderLines(lines, WORLD_SIZE, WORLD_SIZE / 2.0f);

    ASSERT_EQ(lines.size(), static_cast<size_t>(4));
    EXPECT_FLOAT_EQ(lines[1].getPoint1X(), WORLD_SIZE);
    EXPECT_FLOAT_EQ(lines[1].getPoint2Y(), WORLD_SIZE / 2.0f);
    EXPECT_FLOAT_EQ(lines[2].getPoint1Y(), WORLD_SIZE / 2.0f);
}

TEST(UtilGeometryBorder, BorderLinesAreAppended)
{
    std::vector<Line> lines = {Line(1.0f, 1.0f, 2.0f, 2.0f)};

    murosimproc::util::geometry::addBorderLines(lines, WORLD_SIZE);

    EXPECT_EQ(lines.size(), static_cast<size_t>(5));
    EXPECT_FLOAT_EQ(lines[0].getPoint1X(), 1.0f); // the existing line is kept first
}

// ============================================================================
// Rectangle From Points Tests
// ============================================================================

TEST(UtilGeometryRectangle, HorizontalPointsGiveAnAxisAlignedRectangle)
{
    std::vector<Line> lines;

    murosimproc::util::geometry::addRectangleLinesFromPoints(lines, Eigen::Vector2f(0.0f, 0.0f),
                                                             Eigen::Vector2f(4.0f, 0.0f), 1.0f);

    // The rectangle spans the segment plus the margin on every side
    ASSERT_EQ(lines.size(), static_cast<size_t>(4));
    EXPECT_NEAR(lines[0].getPoint1X(), 5.0f, EPSILON);
    EXPECT_NEAR(lines[0].getPoint1Y(), 1.0f, EPSILON);
    EXPECT_NEAR(lines[1].getPoint2X(), -1.0f, EPSILON);
    EXPECT_NEAR(lines[1].getPoint2Y(), -1.0f, EPSILON);
    EXPECT_EQ(fixtures::closedLoops(lines).size(), static_cast<size_t>(1));
}

TEST(UtilGeometryRectangle, RectangleFollowsTheSegmentDirection)
{
    std::vector<Line> lines;

    murosimproc::util::geometry::addRectangleLinesFromPoints(lines, Eigen::Vector2f(0.0f, 0.0f),
                                                             Eigen::Vector2f(0.0f, 4.0f), 0.5f);

    // Vertical segment: the long axis is y, the margin adds 0.5 on every side
    for (size_t i = 0; i < lines.size(); i++)
    {
        EXPECT_LE(std::abs(lines[i].getPoint1X()), 0.5f + EPSILON) << "line " << i;
        EXPECT_GE(lines[i].getPoint1Y(), -0.5f - EPSILON) << "line " << i;
        EXPECT_LE(lines[i].getPoint1Y(), 4.5f + EPSILON) << "line " << i;
    }
    EXPECT_NEAR(std::abs(fixtures::signedArea(fixtures::closedLoops(lines).front())), 5.0f * 1.0f, EPSILON);
}

// ============================================================================
// Orientation Tests
// ============================================================================

TEST(UtilGeometryOrientation, AngleFromCentreToPoint)
{
    using murosimproc::util::geometry::getOrientationToCenterPoint;

    EXPECT_NEAR(getOrientationToCenterPoint(Eigen::Vector2f(1.0f, 1.0f), Eigen::Vector2f(0.0f, 0.0f)), PI_4,
                EPSILON);
    EXPECT_NEAR(getOrientationToCenterPoint(Eigen::Vector2f(0.0f, 1.0f), Eigen::Vector2f(1.0f, 1.0f)), PI,
                EPSILON);
    EXPECT_NEAR(getOrientationToCenterPoint(Eigen::Vector2f(2.0f, 1.0f), Eigen::Vector2f(2.0f, 3.0f)), -PI_2,
                EPSILON);
}

// ============================================================================
// Circles In Circle Tests
// ============================================================================

TEST(UtilGeometryCirclesInCircle, PlacedCirclesFitAndDoNotOverlap)
{
    RandomGenerator rng(SEED);

    const std::vector<Circle> circles = murosimproc::util::geometry::createCirclesInCircle(DISK, 0.5f, 50, rng);

    ASSERT_FALSE(circles.empty());
    EXPECT_LE(circles.size(), static_cast<size_t>(50));
    assertInsideDiskAndDisjoint(circles);
}

TEST(UtilGeometryCirclesInCircle, RadiiFollowTheRatio)
{
    RandomGenerator rng(SEED);

    const std::vector<Circle> circles = murosimproc::util::geometry::createCirclesInCircle(DISK, 0.5f, 50, rng);

    // radius in [R * (1 - ratio) / 2, R / 2]
    for (size_t i = 0; i < circles.size(); i++)
    {
        EXPECT_GE(circles[i].getRadius(), RADIUS * 0.5f * 0.5f - EPSILON) << "circle " << i;
        EXPECT_LE(circles[i].getRadius(), RADIUS * 0.5f + EPSILON) << "circle " << i;
    }
}

TEST(UtilGeometryCirclesInCircle, DefaultTriesIsFive)
{
    RandomGenerator rng(SEED);

    const std::vector<Circle> circles = murosimproc::util::geometry::createCirclesInCircle(DISK, 0.5f, rng);

    EXPECT_LE(circles.size(), static_cast<size_t>(5));
    EXPECT_FALSE(circles.empty()); // the first try never collides
}

TEST(UtilGeometryCirclesInCircle, TargetCountOverloadStopsAtTheTarget)
{
    RandomGenerator rng(SEED);

    const std::vector<Circle> circles =
        murosimproc::util::geometry::createCirclesInCircle(DISK, 1.0f, 3, 20, rng); // small radii: 3 always fit

    EXPECT_EQ(circles.size(), static_cast<size_t>(3));
    assertInsideDiskAndDisjoint(circles);
}

TEST(UtilGeometryCirclesInCircle, SameSeedGivesSameCircles)
{
    RandomGenerator first(SEED);
    RandomGenerator second(SEED);

    const std::vector<Circle> a = murosimproc::util::geometry::createCirclesInCircle(DISK, 0.5f, 20, first);
    const std::vector<Circle> b = murosimproc::util::geometry::createCirclesInCircle(DISK, 0.5f, 20, second);

    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); i++)
    {
        EXPECT_FLOAT_EQ(a[i].getX(), b[i].getX()) << "circle " << i;
        EXPECT_FLOAT_EQ(a[i].getRadius(), b[i].getRadius()) << "circle " << i;
    }
}

// ============================================================================
// Polygon In Circle Tests
// ============================================================================

TEST(UtilGeometryPolygonInCircle, IsAClosedLoopInsideTheCircle)
{
    RandomGenerator rng(SEED);

    const std::vector<Line> lines = murosimproc::util::geometry::createPolygonLinesInCircle(DISK, 0.5f, rng);

    EXPECT_GE(lines.size(), static_cast<size_t>(3));
    EXPECT_LE(lines.size(), static_cast<size_t>(8));
    EXPECT_EQ(fixtures::closedLoops(lines).size(), static_cast<size_t>(1));
    EXPECT_LE(farthestVertex(lines), RADIUS + EPSILON);
    EXPECT_GE(nearestVertex(lines), RADIUS * 0.5f - EPSILON); // ratio_length keeps vertices off the centre
}

TEST(UtilGeometryPolygonInCircle, VertexCountFollowsTheSegmentRange)
{
    RandomGenerator rng(SEED);

    const std::vector<Line> lines = murosimproc::util::geometry::createPolygonLinesInCircle(DISK, 0.2f, 5, 5, rng);

    EXPECT_EQ(lines.size(), static_cast<size_t>(5));
    EXPECT_EQ(fixtures::closedLoops(lines).size(), static_cast<size_t>(1));
}

TEST(UtilGeometryPolygonInCircle, RatioOverloadUsesThreeToEightSegments)
{
    for (uint64_t seed = 0; seed < 10; seed++)
    {
        RandomGenerator rng(seed);
        const std::vector<Line> lines = murosimproc::util::geometry::createPolygonLinesInCircle(DISK, 0.2f, rng);
        EXPECT_GE(lines.size(), static_cast<size_t>(3)) << "seed " << seed;
        EXPECT_LE(lines.size(), static_cast<size_t>(8)) << "seed " << seed;
    }
}

// ============================================================================
// Rectangle In Circle Tests
// ============================================================================

TEST(UtilGeometryRectangleInCircle, IsAClosedRectangleInsideTheCircle)
{
    RandomGenerator rng(SEED);

    const std::vector<Line> lines = murosimproc::util::geometry::createRectangleLinesInCircle(DISK, rng);

    ASSERT_EQ(lines.size(), static_cast<size_t>(4));
    EXPECT_EQ(fixtures::closedLoops(lines).size(), static_cast<size_t>(1));
    EXPECT_LE(farthestVertex(lines), RADIUS * 0.95f + EPSILON); // scale is at most 0.95
    // opposite sides have equal length
    EXPECT_NEAR(lines[0].getLength(), lines[2].getLength(), EPSILON);
    EXPECT_NEAR(lines[1].getLength(), lines[3].getLength(), EPSILON);
    // the aspect ratio is at least 0.4
    const float aspect = std::min(lines[0].getLength(), lines[1].getLength()) /
                         std::max(lines[0].getLength(), lines[1].getLength());
    EXPECT_GE(aspect, 0.4f - EPSILON);
}

// ============================================================================
// L Shape In Circle Tests
// ============================================================================

TEST(UtilGeometryLShapeInCircle, IsAClosedHexagonInsideTheCircle)
{
    RandomGenerator rng(SEED);

    const std::vector<Line> lines = murosimproc::util::geometry::createLShapeLinesInCircle(DISK, rng);

    ASSERT_EQ(lines.size(), static_cast<size_t>(6));
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::closedLoops(lines);
    ASSERT_EQ(loops.size(), static_cast<size_t>(1));
    EXPECT_EQ(loops.front().size(), static_cast<size_t>(6));
    EXPECT_LE(farthestVertex(lines), RADIUS * 0.9f + EPSILON); // scale is at most 0.9
    EXPECT_GT(std::abs(fixtures::signedArea(loops.front())), 0.0f);
}

TEST(UtilGeometryLShapeInCircle, NotchRemovesPartOfTheSquare)
{
    RandomGenerator rng(SEED);

    const std::vector<Line> lines = murosimproc::util::geometry::createLShapeLinesInCircle(DISK, rng);

    // Sides 1 and 5 are full square sides, side 2 (the notch) is shorter than side 1
    const float side = lines[0].getLength();
    EXPECT_NEAR(lines[5].getLength(), side, EPSILON);
    EXPECT_LT(lines[1].getLength(), side);
    EXPECT_LT(lines[4].getLength(), side);
}
