// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/collision/line.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::geometry::Shape;
using murosim::geometry::World;
using murosim::geometry::primitives::pose;

// Boundary margins, see the Boundary Tests section
constexpr float MARGIN = 1e-6f;
constexpr float FAR = 1000.0f;
constexpr float MARGIN_FAR = 1e-2f;

/// lineWithCircleArray with a single circle in the arrays.
bool lineWithCircleSimd(const float x1, const float y1, const float x2, const float y2,
                        const float cx, const float cy, const float cr)
{
    const murosim::simd::ArrayFloat array_cx(std::vector<float>{cx});
    const murosim::simd::ArrayFloat array_cy(std::vector<float>{cy});
    const murosim::simd::ArrayFloat array_cr(std::vector<float>{cr});
    return murosim::collision::lineWithCircleArray(x1, y1, x2, y2, array_cx, array_cy, array_cr);
}

/// lineWithLineArray with a single segment in the arrays.
bool lineWithLineSimd(const float x1, const float y1, const float x2, const float y2,
                      const float x3, const float y3, const float x4, const float y4)
{
    const murosim::simd::ArrayFloat array_x3(std::vector<float>{x3});
    const murosim::simd::ArrayFloat array_y3(std::vector<float>{y3});
    const murosim::simd::ArrayFloat array_x4(std::vector<float>{x4});
    const murosim::simd::ArrayFloat array_y4(std::vector<float>{y4});
    return murosim::collision::lineWithLineArray(x1, y1, x2, y2, array_x3, array_y3, array_x4, array_y4);
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class CollisionLineTest : public ::testing::Test
{
protected:
    Line line{-1.0f, 0.0f, 1.0f, 0.0f}; // horizontal segment through the origin
    Circle circle{0.0f, 0.0f, 0.5f};    // disk of radius 0.5 at the origin
    Shape shape_line = Shape({line}, {});
    Shape shape_circle = Shape({}, {circle});
    Shape square = murosim::geometry::primitives::square(1.0f);
};

// ============================================================================
// Line With World Tests
// ============================================================================

TEST_F(CollisionLineTest, LineWithWorldLine)
{
    const World world_line(shape_line);

    // Crossing, touching at the tip and collinear overlap
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(-1.0f, -0.000001f, 1.0f, 0.000001f), world_line));
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(0.0f, -1.0f, 0.0f, 1.0f), world_line));
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(-1.0f, -1.0f, 1.0f, 1.0f), world_line));

    // Parallel just above, and collinear just beyond the end point
    EXPECT_FALSE(murosim::collision::lineWithWorld(Line(-1.0f, 0.000001f, 1.0f, 0.000001f), world_line));
    EXPECT_FALSE(murosim::collision::lineWithWorld(Line(2.0f, 0.0f, 1.000001f, 0.0f), world_line));
}

TEST_F(CollisionLineTest, LineWithWorldCircle)
{
    const World world_circle(shape_circle);

    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(-1.0f, 0.0f, 1.0f, 0.0f), world_circle));
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(0.0f, -1.0f, 0.0f, 1.0f), world_circle));
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(-1.0f, -1.0f, 1.0f, 1.0f), world_circle));
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(-1.0f, 0.499999f, 1.0f, 0.499999f), world_circle));
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(0.499999f, -1.0f, 0.499999f, 1.0f), world_circle));

    EXPECT_FALSE(murosim::collision::lineWithWorld(Line(-1.0f, 0.500001f, 1.0f, 0.500001f), world_circle));
    EXPECT_FALSE(murosim::collision::lineWithWorld(Line(-0.500001f, -1.0f, -0.500001f, 1.0f), world_circle));

    // A segment inside the disk collides (solid-disk semantics, see
    // collision/line.hpp), whether it crosses the boundary or not
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(-0.2f, 0.0f, 0.2f, 0.0f), world_circle));
    EXPECT_TRUE(murosim::collision::lineWithWorld(Line(0.0f, 0.0f, 0.0f, 0.1f), world_circle));
}

// ============================================================================
// Line With Circle Tests
// ============================================================================

TEST(CollisionLineWithCircle, ObjectOverload)
{
    const Line line_through(-2.0f, 0.0f, 2.0f, 0.0f);
    const Line line_tangent(-2.0f, 1.0f, 2.0f, 1.0f);
    const Line line_outside(-2.0f, 2.0f, 2.0f, 2.0f);
    const Circle circle(0.0f, 0.0f, 1.0f);

    // Line through center
    EXPECT_TRUE(murosim::collision::lineWithCircle(line_through, circle));

    // Tangent line
    EXPECT_TRUE(murosim::collision::lineWithCircle(line_tangent, circle));

    // Line outside
    EXPECT_FALSE(murosim::collision::lineWithCircle(line_outside, circle));

    // Vertical line through center
    const Line line_vertical(0.0f, -2.0f, 0.0f, 2.0f);
    EXPECT_TRUE(murosim::collision::lineWithCircle(line_vertical, circle));
}

TEST(CollisionLineWithCircle, VectorOverload)
{
    const Eigen::Vector2f p1(-2.0f, 0.0f);
    const Eigen::Vector2f p2(2.0f, 0.0f);
    const Eigen::Vector2f center(0.0f, 0.0f);
    const float radius = 1.0f;

    // Line through center
    EXPECT_TRUE(murosim::collision::lineWithCircle(p1, p2, center, radius));

    // Tangent line
    EXPECT_TRUE(murosim::collision::lineWithCircle(Eigen::Vector2f(-2.0f, 1.0f), Eigen::Vector2f(2.0f, 1.0f),
                                                   center, radius));

    // Line outside
    EXPECT_FALSE(murosim::collision::lineWithCircle(Eigen::Vector2f(-2.0f, 1.5f), Eigen::Vector2f(2.0f, 1.5f),
                                                    center, radius));
}

TEST(CollisionLineWithCircle, FloatOverload)
{
    // Line through circle (intersects boundary)
    EXPECT_TRUE(murosim::collision::lineWithCircle(-2.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 1.0f));

    // Tangent line
    EXPECT_TRUE(murosim::collision::lineWithCircle(-2.0f, 1.0f, 2.0f, 1.0f, 0.0f, 0.0f, 1.0f));

    // Line outside
    EXPECT_FALSE(murosim::collision::lineWithCircle(-2.0f, 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 1.0f));

    // Line segment starting at center (intersects boundary)
    EXPECT_TRUE(murosim::collision::lineWithCircle(0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 1.0f));

    // Line through circle at an angle
    EXPECT_TRUE(murosim::collision::lineWithCircle(-2.0f, -2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 1.0f));
}

TEST(CollisionLineWithCircle, ArrayOverload)
{
    murosim::simd::ArrayFloat array_cx(3);
    murosim::simd::ArrayFloat array_cy(3);
    murosim::simd::ArrayFloat array_cr(3);

    array_cx[0] = 0.0f;   array_cy[0] = 0.0f;   array_cr[0] = 1.0f; // At the origin
    array_cx[1] = 10.0f;  array_cy[1] = 10.0f;  array_cr[1] = 1.0f; // Far away
    array_cx[2] = -10.0f; array_cy[2] = -10.0f; array_cr[2] = 1.0f; // Far away

    // Line through the first circle
    EXPECT_TRUE(murosim::collision::lineWithCircleArray(-2.0f, 0.0f, 2.0f, 0.0f, array_cx, array_cy, array_cr));

    // Line that misses all circles
    EXPECT_FALSE(murosim::collision::lineWithCircleArray(5.0f, 5.0f, 6.0f, 5.0f, array_cx, array_cy, array_cr));
}

// ============================================================================
// Line With Line Tests
// ============================================================================

TEST(CollisionLineWithLine, ObjectOverload)
{
    const Line line1(-1.0f, 0.0f, 1.0f, 0.0f);
    const Line line2(0.0f, -1.0f, 0.0f, 1.0f);
    const Line line3(2.0f, -1.0f, 2.0f, 1.0f);
    const Line line4(-1.0f, 0.5f, 1.0f, 0.5f);

    // Crossing lines
    EXPECT_TRUE(murosim::collision::lineWithLine(line1, line2));

    // Parallel non-overlapping lines
    EXPECT_FALSE(murosim::collision::lineWithLine(line1, line4));

    // Non-intersecting perpendicular lines
    EXPECT_FALSE(murosim::collision::lineWithLine(line1, line3));

    // Diagonal crossing
    const Line diagonal1(-1.0f, -1.0f, 1.0f, 1.0f);
    const Line diagonal2(-1.0f, 1.0f, 1.0f, -1.0f);
    EXPECT_TRUE(murosim::collision::lineWithLine(diagonal1, diagonal2));

    // X-shape crossing with reversed direction
    const Line x1(-1.0f, -1.0f, 1.0f, 1.0f);
    const Line x2(1.0f, -1.0f, -1.0f, 1.0f);
    EXPECT_TRUE(murosim::collision::lineWithLine(x1, x2));
}

TEST(CollisionLineWithLine, FloatOverload)
{
    // Crossing lines
    EXPECT_TRUE(murosim::collision::lineWithLine(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f));

    // Parallel lines
    EXPECT_FALSE(murosim::collision::lineWithLine(-1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f, 1.0f, 1.0f));

    // T-junction (touching at endpoint)
    EXPECT_TRUE(murosim::collision::lineWithLine(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

    // L-shape (touching at corners)
    EXPECT_TRUE(murosim::collision::lineWithLine(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

    // Collinear but not overlapping
    EXPECT_FALSE(murosim::collision::lineWithLine(-2.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 2.0f, 0.0f));

    // Diagonal X intersection
    EXPECT_TRUE(murosim::collision::lineWithLine(-1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f));
}

TEST(CollisionLineWithLine, ArrayOverload)
{
    murosim::simd::ArrayFloat array_p3x(3);
    murosim::simd::ArrayFloat array_p3y(3);
    murosim::simd::ArrayFloat array_p4x(3);
    murosim::simd::ArrayFloat array_p4y(3);

    array_p3x[0] = 0.0f;  array_p3y[0] = -1.0f; array_p4x[0] = 0.0f;  array_p4y[0] = 1.0f;  // Vertical, crossing
    array_p3x[1] = -1.0f; array_p3y[1] = 2.0f;  array_p4x[1] = 1.0f;  array_p4y[1] = 2.0f;  // Parallel above
    array_p3x[2] = 10.0f; array_p3y[2] = 10.0f; array_p4x[2] = 11.0f; array_p4y[2] = 11.0f; // Far away

    // Horizontal line intersects the first (vertical) line
    EXPECT_TRUE(murosim::collision::lineWithLineArray(-1.0f, 0.0f, 1.0f, 0.0f,
                                                      array_p3x, array_p3y, array_p4x, array_p4y));

    // Line that misses all
    EXPECT_FALSE(murosim::collision::lineWithLineArray(5.0f, 5.0f, 6.0f, 5.0f,
                                                       array_p3x, array_p3y, array_p4x, array_p4y));
}

// ============================================================================
// Line With Shape Tests
// ============================================================================

TEST_F(CollisionLineTest, LineWithShapeLine)
{
    const Eigen::Affine2f identity = Eigen::Affine2f::Identity();
    const Eigen::Affine2f shifted = pose(1.0f, 0.0f, 0.0f);

    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, -0.000001f, 1.0f, 0.000001f), shape_line, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(0.0f, -1.0f, 0.0f, 1.0f), shape_line, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, -1.0f, 1.0f, 1.0f), shape_line, identity));

    EXPECT_FALSE(murosim::collision::lineWithShape(Line(-1.0f, 0.000001f, 1.0f, 0.000001f), shape_line, identity));
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(2.0f, 0.0f, 1.000001f, 0.0f), shape_line, identity));

    // The shape pose moves the segment by one unit in x
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(0.0f, -0.000001f, 2.0f, 0.000001f), shape_line, shifted));
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(0.0f, 0.000001f, 2.0f, 0.000001f), shape_line, shifted));
}

TEST_F(CollisionLineTest, LineWithShapeCircle)
{
    const Eigen::Affine2f identity = Eigen::Affine2f::Identity();
    const Eigen::Affine2f shifted = pose(1.0f, 0.0f, 0.0f);

    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, 0.0f, 1.0f, 0.0f), shape_circle, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(0.0f, -1.0f, 0.0f, 1.0f), shape_circle, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, -1.0f, 1.0f, 1.0f), shape_circle, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, 0.499999f, 1.0f, 0.499999f), shape_circle, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(0.499999f, -1.0f, 0.499999f, 1.0f), shape_circle, identity));

    EXPECT_FALSE(murosim::collision::lineWithShape(Line(-1.0f, 0.500001f, 1.0f, 0.500001f), shape_circle, identity));
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(-0.500001f, -1.0f, -0.500001f, 1.0f), shape_circle, identity));

    // The shape pose moves the disk by one unit in x
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(0.0f, 0.0f, 2.0f, 0.0f), shape_circle, shifted));
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(0.0f, 0.500001f, 2.0f, 0.500001f), shape_circle, shifted));
}

TEST_F(CollisionLineTest, LineWithShapeSquare)
{
    const Eigen::Affine2f identity = Eigen::Affine2f::Identity();
    const Eigen::Affine2f rotated = pose(0.0f, 0.0f, PI);

    // Segments crossing the square's edges
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, 0.0f, 1.0f, 0.0f), square, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(0.0f, -1.0f, 0.0f, 1.0f), square, identity));
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, -1.0f, 1.0f, 1.0f), square, identity));

    // Segments fully inside the outline do not touch an edge
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(-0.499999f, 0.0f, 0.499999f, 0.0f), square, identity));
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(0.0f, -0.499999f, 0.0f, 0.499999f), square, identity));
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(-0.499999f, -0.499999f, 0.499999f, 0.499999f),
                                                   square, identity));

    // A rotated square is the same square
    EXPECT_TRUE(murosim::collision::lineWithShape(Line(-1.0f, -1.0f, 1.0f, 1.0f), square, rotated));
    EXPECT_FALSE(murosim::collision::lineWithShape(Line(-0.499999f, -0.499999f, 0.499999f, 0.499999f),
                                                   square, rotated));
}

// ============================================================================
// Degenerate Segment Tests
// ============================================================================

// Regression: the scalar and SIMD backends must agree on zero-length segments.
// Historically the SIMD kernels produced NaN while the scalar guard
// short-circuited. Under the solid-disk semantics a segment collides if it
// touches or lies inside the disk, so a degenerate segment collides exactly
// when its point is inside the circle.
TEST(CollisionLineDegenerateSegment, PointInsideCircleIsCollision)
{
    EXPECT_TRUE(murosim::collision::lineWithCircle(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));
}

TEST(CollisionLineDegenerateSegment, PointOutsideCircleIsNoCollision)
{
    EXPECT_FALSE(murosim::collision::lineWithCircle(5.0f, 5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 1.0f));
}

TEST(CollisionLineDegenerateSegment, ArrayBackendMatchesScalar)
{
    // Exercises the SIMD lineWithCircleArray backend selected by the build
    const murosim::simd::ArrayFloat circle_x(std::vector<float>{0.0f});
    const murosim::simd::ArrayFloat circle_y(std::vector<float>{0.0f});
    const murosim::simd::ArrayFloat circle_r(std::vector<float>{1.0f});

    // A point inside the disk collides, agreeing with the scalar path
    EXPECT_TRUE(murosim::collision::lineWithCircleArray(0.0f, 0.0f, 0.0f, 0.0f, circle_x, circle_y, circle_r));

    // A real segment through the circle still collides
    EXPECT_TRUE(murosim::collision::lineWithCircleArray(-2.0f, 0.0f, 2.0f, 0.0f, circle_x, circle_y, circle_r));
}

// ============================================================================
// Boundary Tests
// ============================================================================

// The kernels compare squared distances with <=, so exactly touching counts
// as a collision. Just-inside/just-outside pairs use a margin well above the
// float resolution at their magnitude: MARGIN near the origin (16 ulps at
// 0.5) and MARGIN_FAR at FAR (160 ulps at 1000). The *Simd helpers run the
// array kernels on a single element, which must decide the same way as the
// scalar kernels.

TEST(CollisionLineBoundary, TangentLineCollides)
{
    // Horizontal segment at y = 1 touching the unit circle from above
    EXPECT_TRUE(murosim::collision::lineWithCircle(-2.0f, 1.0f, 2.0f, 1.0f, 0.0f, 0.0f, 1.0f));
    EXPECT_TRUE(lineWithCircleSimd(-2.0f, 1.0f, 2.0f, 1.0f, 0.0f, 0.0f, 1.0f));
}

TEST(CollisionLineBoundary, CircleJustInsideAndJustOutside)
{
    EXPECT_TRUE(murosim::collision::lineWithCircle(-2.0f, 1.0f - MARGIN, 2.0f, 1.0f - MARGIN, 0.0f, 0.0f, 1.0f));
    EXPECT_FALSE(murosim::collision::lineWithCircle(-2.0f, 1.0f + MARGIN, 2.0f, 1.0f + MARGIN, 0.0f, 0.0f, 1.0f));

    EXPECT_TRUE(lineWithCircleSimd(-2.0f, 1.0f - MARGIN, 2.0f, 1.0f - MARGIN, 0.0f, 0.0f, 1.0f));
    EXPECT_FALSE(lineWithCircleSimd(-2.0f, 1.0f + MARGIN, 2.0f, 1.0f + MARGIN, 0.0f, 0.0f, 1.0f));
}

TEST(CollisionLineBoundary, CircleFarFromOrigin)
{
    const float inside = FAR + 1.0f - MARGIN_FAR;
    const float outside = FAR + 1.0f + MARGIN_FAR;

    EXPECT_TRUE(murosim::collision::lineWithCircle(FAR - 2.0f, inside, FAR + 2.0f, inside, FAR, FAR, 1.0f));
    EXPECT_FALSE(murosim::collision::lineWithCircle(FAR - 2.0f, outside, FAR + 2.0f, outside, FAR, FAR, 1.0f));

    EXPECT_TRUE(lineWithCircleSimd(FAR - 2.0f, inside, FAR + 2.0f, inside, FAR, FAR, 1.0f));
    EXPECT_FALSE(lineWithCircleSimd(FAR - 2.0f, outside, FAR + 2.0f, outside, FAR, FAR, 1.0f));
}

TEST(CollisionLineBoundary, TouchingEndpointCollides)
{
    // Vertical segment starting exactly on the horizontal one (T-junction)
    EXPECT_TRUE(murosim::collision::lineWithLine(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));
    EXPECT_TRUE(lineWithLineSimd(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));
}

TEST(CollisionLineBoundary, LineJustCrossingAndJustApart)
{
    EXPECT_TRUE(murosim::collision::lineWithLine(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, -MARGIN, 0.0f, 1.0f));
    EXPECT_FALSE(murosim::collision::lineWithLine(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, MARGIN, 0.0f, 1.0f));

    EXPECT_TRUE(lineWithLineSimd(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, -MARGIN, 0.0f, 1.0f));
    EXPECT_FALSE(lineWithLineSimd(-1.0f, 0.0f, 1.0f, 0.0f, 0.0f, MARGIN, 0.0f, 1.0f));
}

TEST(CollisionLineBoundary, LineFarFromOrigin)
{
    const float crossing = FAR - MARGIN_FAR;
    const float apart = FAR + MARGIN_FAR;

    EXPECT_TRUE(murosim::collision::lineWithLine(FAR - 1.0f, FAR, FAR + 1.0f, FAR, FAR, crossing, FAR, FAR + 1.0f));
    EXPECT_FALSE(murosim::collision::lineWithLine(FAR - 1.0f, FAR, FAR + 1.0f, FAR, FAR, apart, FAR, FAR + 1.0f));

    EXPECT_TRUE(lineWithLineSimd(FAR - 1.0f, FAR, FAR + 1.0f, FAR, FAR, crossing, FAR, FAR + 1.0f));
    EXPECT_FALSE(lineWithLineSimd(FAR - 1.0f, FAR, FAR + 1.0f, FAR, FAR, apart, FAR, FAR + 1.0f));
}
