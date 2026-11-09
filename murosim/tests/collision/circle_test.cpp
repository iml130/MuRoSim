// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/collision/circle.hpp"
#include "murosim/geometry/primitives.hpp"
#include <vector>

namespace
{
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::geometry::Shape;
using murosim::geometry::World;
using murosim::geometry::primitives::pose;

// Boundary margins, see the Boundary Tests section
constexpr float MARGIN = 1e-6f;
constexpr float FAR = 1000.0f;
constexpr float MARGIN_FAR = 1e-2f;

/// circleWithCircleArray with a single circle in the arrays.
bool circleWithCircleSimd(const float x, const float y, const float r, const float cx, const float cy, const float cr)
{
    const murosim::simd::ArrayFloat array_x(std::vector<float>{cx});
    const murosim::simd::ArrayFloat array_y(std::vector<float>{cy});
    const murosim::simd::ArrayFloat array_r(std::vector<float>{cr});
    return murosim::collision::circleWithCircleArray(x, y, r, array_x, array_y, array_r);
}

/// circleWithLineArray with a single segment in the arrays.
bool circleWithLineSimd(const float x, const float y, const float r,
                        const float x1, const float y1, const float x2, const float y2)
{
    const murosim::simd::ArrayFloat array_x1(std::vector<float>{x1});
    const murosim::simd::ArrayFloat array_y1(std::vector<float>{y1});
    const murosim::simd::ArrayFloat array_x2(std::vector<float>{x2});
    const murosim::simd::ArrayFloat array_y2(std::vector<float>{y2});
    return murosim::collision::circleWithLineArray(x, y, r, array_x1, array_y1, array_x2, array_y2);
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class CollisionCircleTest : public ::testing::Test
{
protected:
    Line line{-1.0f, 0.0f, 1.0f, 0.0f}; // horizontal segment through the origin
    Circle circle{0.0f, 0.0f, 0.5f};    // disk of radius 0.5 at the origin
    Shape shape_line = Shape({line}, {});
    Shape shape_circle = Shape({}, {circle});
    Shape square = murosim::geometry::primitives::square(1.0f);
};

// ============================================================================
// Circle With World Tests
// ============================================================================

TEST_F(CollisionCircleTest, CircleWithWorldLine)
{
    const World world_line(shape_line);

    EXPECT_TRUE(murosim::collision::circleWithWorld(Circle(0.0f, 0.0f, 0.1f), world_line));
    EXPECT_TRUE(murosim::collision::circleWithWorld(Circle(0.0f, 0.09f, 0.1f), world_line));
    EXPECT_FALSE(murosim::collision::circleWithWorld(Circle(1.11f, 0.0f, 0.1f), world_line));

    // A zero-radius circle collides only when its centre lies on the segment
    EXPECT_TRUE(murosim::collision::circleWithWorld(Circle(0.0f, 0.0f, 0.0f), world_line));
    EXPECT_FALSE(murosim::collision::circleWithWorld(Circle(0.0f, -0.1f, 0.0f), world_line));
    EXPECT_FALSE(murosim::collision::circleWithWorld(Circle(-1.1f, 0.0f, 0.0f), world_line));
}

TEST_F(CollisionCircleTest, CircleWithWorldCircle)
{
    const World world_circle(shape_circle);

    EXPECT_TRUE(murosim::collision::circleWithWorld(Circle(0.0f, 0.0f, 0.1f), world_circle));
    EXPECT_FALSE(murosim::collision::circleWithWorld(Circle(0.0f, 0.61f, 0.1f), world_circle));
    EXPECT_FALSE(murosim::collision::circleWithWorld(Circle(0.61f, 0.0f, 0.1f), world_circle));

    // A zero-radius circle collides only when its centre lies inside the disk
    EXPECT_TRUE(murosim::collision::circleWithWorld(Circle(0.0f, 0.0f, 0.0f), world_circle));
    EXPECT_FALSE(murosim::collision::circleWithWorld(Circle(-0.51f, 0.0f, 0.0f), world_circle));
    EXPECT_FALSE(murosim::collision::circleWithWorld(Circle(0.0f, -0.51f, 0.0f), world_circle));
}

// ============================================================================
// Circle With Circle Tests
// ============================================================================

TEST(CollisionCircleWithCircle, ObjectOverload)
{
    const Circle c1(0.0f, 0.0f, 1.0f);
    const Circle c2(1.5f, 0.0f, 1.0f);
    const Circle c3(3.0f, 0.0f, 1.0f);

    // Overlapping circles
    EXPECT_TRUE(murosim::collision::circleWithCircle(c1, c2));

    // Touching circles
    const Circle c_touch(2.0f, 0.0f, 1.0f);
    EXPECT_TRUE(murosim::collision::circleWithCircle(c1, c_touch));

    // Non-overlapping circles
    EXPECT_FALSE(murosim::collision::circleWithCircle(c1, c3));

    // Same circle
    EXPECT_TRUE(murosim::collision::circleWithCircle(c1, c1));

    // Concentric circles
    const Circle c_inner(0.0f, 0.0f, 0.5f);
    EXPECT_TRUE(murosim::collision::circleWithCircle(c1, c_inner));
}

TEST(CollisionCircleWithCircle, FloatOverload)
{
    // Overlapping
    EXPECT_TRUE(murosim::collision::circleWithCircle(0.0f, 0.0f, 1.0f, 1.5f, 0.0f, 1.0f));

    // Touching
    EXPECT_TRUE(murosim::collision::circleWithCircle(0.0f, 0.0f, 1.0f, 2.0f, 0.0f, 1.0f));

    // Not overlapping
    EXPECT_FALSE(murosim::collision::circleWithCircle(0.0f, 0.0f, 1.0f, 3.0f, 0.0f, 1.0f));

    // Zero radius circles at same point
    EXPECT_TRUE(murosim::collision::circleWithCircle(1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f));

    // Zero radius circles at different points
    EXPECT_FALSE(murosim::collision::circleWithCircle(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f));

    // Diagonal overlap
    EXPECT_TRUE(murosim::collision::circleWithCircle(0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f));

    // Diagonal no overlap
    EXPECT_FALSE(murosim::collision::circleWithCircle(0.0f, 0.0f, 0.5f, 2.0f, 2.0f, 0.5f));
}

TEST(CollisionCircleWithCircle, ArrayOverload)
{
    murosim::simd::ArrayFloat array_x(5);
    murosim::simd::ArrayFloat array_y(5);
    murosim::simd::ArrayFloat array_r(5);

    array_x[0] = 0.5f;  array_y[0] = 0.0f;  array_r[0] = 0.5f; // Overlapping
    array_x[1] = 2.0f;  array_y[1] = 0.0f;  array_r[1] = 1.0f; // Touching
    array_x[2] = 5.0f;  array_y[2] = 0.0f;  array_r[2] = 1.0f; // Not overlapping
    array_x[3] = 0.0f;  array_y[3] = 0.5f;  array_r[3] = 0.5f; // Overlapping
    array_x[4] = 10.0f; array_y[4] = 10.0f; array_r[4] = 1.0f; // Far away

    // Circle at origin with radius 1 collides with some of them
    EXPECT_TRUE(murosim::collision::circleWithCircleArray(0.0f, 0.0f, 1.0f, array_x, array_y, array_r));

    // Circle far away collides with none
    EXPECT_FALSE(murosim::collision::circleWithCircleArray(100.0f, 100.0f, 1.0f, array_x, array_y, array_r));
}

// ============================================================================
// Circle With Line Tests
// ============================================================================

TEST(CollisionCircleWithLine, ObjectOverload)
{
    const Circle circle(0.0f, 0.0f, 1.0f);
    const Line line_through(-2.0f, 0.0f, 2.0f, 0.0f);
    const Line line_tangent(-2.0f, 1.0f, 2.0f, 1.0f);
    const Line line_outside(-2.0f, 2.0f, 2.0f, 2.0f);

    // Line through center
    EXPECT_TRUE(murosim::collision::circleWithLine(circle, line_through));

    // Tangent line
    EXPECT_TRUE(murosim::collision::circleWithLine(circle, line_tangent));

    // Line outside
    EXPECT_FALSE(murosim::collision::circleWithLine(circle, line_outside));

    // Vertical line through center
    const Line line_vertical(0.0f, -2.0f, 0.0f, 2.0f);
    EXPECT_TRUE(murosim::collision::circleWithLine(circle, line_vertical));

    // Diagonal line through center
    const Line line_diagonal(-2.0f, -2.0f, 2.0f, 2.0f);
    EXPECT_TRUE(murosim::collision::circleWithLine(circle, line_diagonal));
}

TEST(CollisionCircleWithLine, VectorOverload)
{
    const Eigen::Vector2f center(0.0f, 0.0f);
    const float radius = 1.0f;

    // Line through center
    EXPECT_TRUE(murosim::collision::circleWithLine(center, radius,
                                                   Eigen::Vector2f(-2.0f, 0.0f), Eigen::Vector2f(2.0f, 0.0f)));

    // Tangent line
    EXPECT_TRUE(murosim::collision::circleWithLine(center, radius,
                                                   Eigen::Vector2f(-2.0f, 1.0f), Eigen::Vector2f(2.0f, 1.0f)));

    // Line outside
    EXPECT_FALSE(murosim::collision::circleWithLine(center, radius,
                                                    Eigen::Vector2f(-2.0f, 1.5f), Eigen::Vector2f(2.0f, 1.5f)));
}

TEST(CollisionCircleWithLine, FloatOverload)
{
    // Line through circle (intersects boundary)
    EXPECT_TRUE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, -2.0f, 0.0f, 2.0f, 0.0f));

    // Tangent line
    EXPECT_TRUE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, -2.0f, 1.0f, 2.0f, 1.0f));

    // Line outside
    EXPECT_FALSE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, -2.0f, 2.0f, 2.0f, 2.0f));

    // Line segment one end inside, one outside (intersects boundary)
    EXPECT_TRUE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 5.0f, 0.0f));

    // Line segment near circle edge
    EXPECT_TRUE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, -2.0f, 0.5f, 2.0f, 0.5f));
}

TEST(CollisionCircleWithLine, ArrayOverload)
{
    murosim::simd::ArrayFloat array_x1(3);
    murosim::simd::ArrayFloat array_y1(3);
    murosim::simd::ArrayFloat array_x2(3);
    murosim::simd::ArrayFloat array_y2(3);

    array_x1[0] = -2.0f; array_y1[0] = 0.0f; array_x2[0] = 2.0f; array_y2[0] = 0.0f; // Through the circle
    array_x1[1] = -2.0f; array_y1[1] = 5.0f; array_x2[1] = 2.0f; array_y2[1] = 5.0f; // Outside
    array_x1[2] = 5.0f;  array_y1[2] = 5.0f; array_x2[2] = 6.0f; array_y2[2] = 6.0f; // Outside

    // Collides with the first line
    EXPECT_TRUE(murosim::collision::circleWithLineArray(0.0f, 0.0f, 1.0f, array_x1, array_y1, array_x2, array_y2));

    // Circle far away collides with none
    EXPECT_FALSE(murosim::collision::circleWithLineArray(100.0f, 100.0f, 1.0f, array_x1, array_y1, array_x2, array_y2));
}

// ============================================================================
// Circle With Shape Tests
// ============================================================================

TEST_F(CollisionCircleTest, CircleWithShapeLine)
{
    const Eigen::Affine2f identity = Eigen::Affine2f::Identity();
    const Eigen::Affine2f shifted = pose(1.0f, 0.0f, 0.0f);

    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(0.0f, 0.0f, 0.1f), shape_line, identity));
    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(0.0f, 0.09f, 0.1f), shape_line, identity));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(1.11f, 0.0f, 0.1f), shape_line, identity));

    // The shape pose moves the segment by one unit in x
    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(1.0f, 0.0f, 0.1f), shape_line, shifted));
    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(1.0f, 0.09f, 0.1f), shape_line, shifted));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(2.11f, 0.0f, 0.1f), shape_line, shifted));
}

TEST_F(CollisionCircleTest, CircleWithShapeCircle)
{
    const Eigen::Affine2f identity = Eigen::Affine2f::Identity();
    const Eigen::Affine2f shifted = pose(1.0f, 0.0f, 0.0f);

    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(0.0f, 0.0f, 0.1f), shape_circle, identity));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(0.0f, 0.61f, 0.1f), shape_circle, identity));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(0.61f, 0.0f, 0.1f), shape_circle, identity));

    // The shape pose moves the disk by one unit in x
    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(1.0f, 0.0f, 0.1f), shape_circle, shifted));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(1.0f, 0.61f, 0.1f), shape_circle, shifted));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(1.61f, 0.0f, 0.1f), shape_circle, shifted));
}

TEST_F(CollisionCircleTest, CircleWithShapeSquare)
{
    const Eigen::Affine2f identity = Eigen::Affine2f::Identity();

    // The square is an outline of edges: a disk inside it collides only once
    // it reaches an edge (half the side length away)
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(0.0f, 0.0f, 0.1f), square, identity));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(0.0f, 0.0f, 0.499999f), square, identity));
    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(0.0f, 0.0f, 0.500001f), square, identity));

    // The corners (distance ~0.707) lie inside this disk, so the edges overlap
    // it: solid-disk semantics report a collision
    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(0.0f, 0.0f, 1.0f), square, identity));

    // Same checks with the square moved by its pose
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(1.0f, 0.0f, 0.1f), square, pose(1.0f, 0.0f, 0.0f)));
    EXPECT_FALSE(murosim::collision::circleWithShape(Circle(0.0f, 1.0f, 0.499999f), square, pose(0.0f, 1.0f, 0.0f)));
    EXPECT_TRUE(murosim::collision::circleWithShape(Circle(1.0f, 1.0f, 0.500001f), square, pose(1.0f, 1.0f, 0.0f)));
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

TEST(CollisionCircleBoundary, TouchingCirclesCollide)
{
    // Unit circles with centres exactly 2.0 apart
    EXPECT_TRUE(murosim::collision::circleWithCircle(0.0f, 0.0f, 1.0f, 2.0f, 0.0f, 1.0f));
    EXPECT_TRUE(circleWithCircleSimd(0.0f, 0.0f, 1.0f, 2.0f, 0.0f, 1.0f));
}

TEST(CollisionCircleBoundary, CirclesJustInsideAndJustOutside)
{
    EXPECT_TRUE(murosim::collision::circleWithCircle(0.0f, 0.0f, 1.0f, 2.0f - MARGIN, 0.0f, 1.0f));
    EXPECT_FALSE(murosim::collision::circleWithCircle(0.0f, 0.0f, 1.0f, 2.0f + MARGIN, 0.0f, 1.0f));

    EXPECT_TRUE(circleWithCircleSimd(0.0f, 0.0f, 1.0f, 2.0f - MARGIN, 0.0f, 1.0f));
    EXPECT_FALSE(circleWithCircleSimd(0.0f, 0.0f, 1.0f, 2.0f + MARGIN, 0.0f, 1.0f));
}

TEST(CollisionCircleBoundary, CirclesFarFromOrigin)
{
    EXPECT_TRUE(murosim::collision::circleWithCircle(FAR, FAR, 1.0f, FAR + 2.0f - MARGIN_FAR, FAR, 1.0f));
    EXPECT_FALSE(murosim::collision::circleWithCircle(FAR, FAR, 1.0f, FAR + 2.0f + MARGIN_FAR, FAR, 1.0f));

    EXPECT_TRUE(circleWithCircleSimd(FAR, FAR, 1.0f, FAR + 2.0f - MARGIN_FAR, FAR, 1.0f));
    EXPECT_FALSE(circleWithCircleSimd(FAR, FAR, 1.0f, FAR + 2.0f + MARGIN_FAR, FAR, 1.0f));
}

TEST(CollisionCircleBoundary, TangentLineCollides)
{
    // Horizontal segment at y = 1 touching the unit circle from above
    EXPECT_TRUE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, -2.0f, 1.0f, 2.0f, 1.0f));
    EXPECT_TRUE(circleWithLineSimd(0.0f, 0.0f, 1.0f, -2.0f, 1.0f, 2.0f, 1.0f));
}

TEST(CollisionCircleBoundary, LineJustInsideAndJustOutside)
{
    EXPECT_TRUE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, -2.0f, 1.0f - MARGIN, 2.0f, 1.0f - MARGIN));
    EXPECT_FALSE(murosim::collision::circleWithLine(0.0f, 0.0f, 1.0f, -2.0f, 1.0f + MARGIN, 2.0f, 1.0f + MARGIN));

    EXPECT_TRUE(circleWithLineSimd(0.0f, 0.0f, 1.0f, -2.0f, 1.0f - MARGIN, 2.0f, 1.0f - MARGIN));
    EXPECT_FALSE(circleWithLineSimd(0.0f, 0.0f, 1.0f, -2.0f, 1.0f + MARGIN, 2.0f, 1.0f + MARGIN));
}

TEST(CollisionCircleBoundary, LineFarFromOrigin)
{
    const float inside = FAR + 1.0f - MARGIN_FAR;
    const float outside = FAR + 1.0f + MARGIN_FAR;

    EXPECT_TRUE(murosim::collision::circleWithLine(FAR, FAR, 1.0f, FAR - 2.0f, inside, FAR + 2.0f, inside));
    EXPECT_FALSE(murosim::collision::circleWithLine(FAR, FAR, 1.0f, FAR - 2.0f, outside, FAR + 2.0f, outside));

    EXPECT_TRUE(circleWithLineSimd(FAR, FAR, 1.0f, FAR - 2.0f, inside, FAR + 2.0f, inside));
    EXPECT_FALSE(circleWithLineSimd(FAR, FAR, 1.0f, FAR - 2.0f, outside, FAR + 2.0f, outside));
}
