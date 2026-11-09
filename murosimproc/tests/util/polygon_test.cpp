// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/util/polygon.hpp"
#include "murosim/geometry/line.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::geometry::Line;
using fixtures::EPSILON;

constexpr float SIDE = 2.0f;
constexpr float OFFSET = 1.0f;
constexpr size_t POINTS_PER_CIRCLE = 16; // four segments per corner arc

/// Unit square from the origin to (SIDE, SIDE), counter-clockwise.
std::vector<Eigen::Vector2f> square()
{
    return {Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(SIDE, 0.0f), Eigen::Vector2f(SIDE, SIDE),
            Eigen::Vector2f(0.0f, SIDE)};
}

void boundingBox(const std::vector<Line> &lines, Eigen::Vector2f &low, Eigen::Vector2f &high)
{
    low = Eigen::Vector2f(1e9f, 1e9f);
    high = -low;
    for (const Line &line : lines)
    {
        low = low.cwiseMin(line.getPoint1());
        high = high.cwiseMax(line.getPoint1());
    }
}
} // namespace

// ============================================================================
// Offset Tests
// ============================================================================

TEST(UtilPolygonOffset, PositiveOffsetGrowsTheSquareWithRoundCorners)
{
    const std::vector<Line> lines = murosimproc::util::polygon::offsetPolygonRound(square(), OFFSET, POINTS_PER_CIRCLE);

    // four sides plus four quarter arcs of four segments each
    ASSERT_EQ(lines.size(), static_cast<size_t>(4 + 4 * 4));
    Eigen::Vector2f low;
    Eigen::Vector2f high;
    boundingBox(lines, low, high);
    EXPECT_NEAR(low.x(), -OFFSET, EPSILON);
    EXPECT_NEAR(low.y(), -OFFSET, EPSILON);
    EXPECT_NEAR(high.x(), SIDE + OFFSET, EPSILON);
    EXPECT_NEAR(high.y(), SIDE + OFFSET, EPSILON);
}

// Regression: every ring ended with a zero-length segment from the repeated
// closing point back to the start.
TEST(UtilPolygonOffset, ResultIsOneClosedLoopWithoutDegenerateSegments)
{
    const std::vector<Line> lines = murosimproc::util::polygon::offsetPolygonRound(square(), OFFSET, POINTS_PER_CIRCLE);

    for (size_t i = 0; i < lines.size(); i++)
    {
        ASSERT_GT(lines[i].getLength(), EPSILON) << "line " << i;
    }

    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::closedLoops(lines);

    ASSERT_EQ(loops.size(), static_cast<size_t>(1));
    // area of the rounded square: side^2 + 4 * side * offset + pi * offset^2, the arcs are inscribed
    const float exact = SIDE * SIDE + 4.0f * SIDE * OFFSET + PI * OFFSET * OFFSET;
    const float area = std::abs(fixtures::signedArea(loops.front()));
    EXPECT_LT(area, exact + EPSILON);
    EXPECT_GT(area, 0.95f * exact);
}

TEST(UtilPolygonOffset, EveryVertexIsAtLeastTheOffsetAwayFromTheSquare)
{
    const std::vector<Line> lines = murosimproc::util::polygon::offsetPolygonRound(square(), OFFSET, POINTS_PER_CIRCLE);

    for (size_t i = 0; i < lines.size(); i++)
    {
        const Eigen::Vector2f point = lines[i].getPoint1();
        // distance from the point to the square (clamped to the box)
        const Eigen::Vector2f clamped = point.cwiseMax(Eigen::Vector2f::Zero()).cwiseMin(Eigen::Vector2f(SIDE, SIDE));
        ASSERT_NEAR((point - clamped).norm(), OFFSET, 1e-4f) << "vertex " << i;
    }
}

TEST(UtilPolygonOffset, NegativeOffsetShrinksTheSquare)
{
    const std::vector<Line> lines = murosimproc::util::polygon::offsetPolygonRound(square(), -0.5f, POINTS_PER_CIRCLE);

    ASSERT_EQ(lines.size(), static_cast<size_t>(4)); // inward offsets keep sharp corners
    Eigen::Vector2f low;
    Eigen::Vector2f high;
    boundingBox(lines, low, high);
    EXPECT_NEAR(low.x(), 0.5f, EPSILON);
    EXPECT_NEAR(high.x(), SIDE - 0.5f, EPSILON);
}

TEST(UtilPolygonOffset, MorePointsPerCircleGiveMoreSegments)
{
    const std::vector<Line> coarse =
        murosimproc::util::polygon::offsetPolygonRound(square(), OFFSET, POINTS_PER_CIRCLE);
    const std::vector<Line> fine =
        murosimproc::util::polygon::offsetPolygonRound(square(), OFFSET, 4 * POINTS_PER_CIRCLE);

    EXPECT_EQ(fine.size(), static_cast<size_t>(4 + 4 * 16));
    EXPECT_GT(fine.size(), coarse.size());
    EXPECT_EQ(fixtures::closedLoops(fine).size(), static_cast<size_t>(1));
}

TEST(UtilPolygonOffset, FewerThanFourPointsAreRaisedToFour)
{
    const std::vector<Line> lines = murosimproc::util::polygon::offsetPolygonRound(square(), OFFSET, 1);

    EXPECT_EQ(lines.size(), static_cast<size_t>(4 + 4 * 1)); // one segment per corner
    EXPECT_EQ(fixtures::closedLoops(lines).size(), static_cast<size_t>(1));
}

TEST(UtilPolygonOffset, ClockwiseInputIsCorrectedFirst)
{
    std::vector<Eigen::Vector2f> clockwise = square();
    std::reverse(clockwise.begin(), clockwise.end());

    const std::vector<Line> lines =
        murosimproc::util::polygon::offsetPolygonRound(clockwise, OFFSET, POINTS_PER_CIRCLE);

    Eigen::Vector2f low;
    Eigen::Vector2f high;
    boundingBox(lines, low, high);
    EXPECT_NEAR(low.x(), -OFFSET, EPSILON); // grown, not shrunk
    EXPECT_NEAR(high.x(), SIDE + OFFSET, EPSILON);
}
