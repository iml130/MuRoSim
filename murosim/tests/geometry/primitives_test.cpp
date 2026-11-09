// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>

namespace
{
using murosim::math::PI_2;
using murosim::math::PI_4;
constexpr float EPSILON = 1e-5f;

// Vertices of the unit equilateral triangle (side 1, centroid at the origin)
const Eigen::Vector2f TRIANGLE_TOP(0.0f, std::sqrt(3.0f) / 3.0f);
const Eigen::Vector2f TRIANGLE_LEFT(-0.5f, -std::sqrt(3.0f) / 6.0f);
const Eigen::Vector2f TRIANGLE_RIGHT(0.5f, -std::sqrt(3.0f) / 6.0f);
} // namespace

// ============================================================================
// Pose Creation Tests
// ============================================================================

TEST(PrimitivesPose, CreatePoseFromFloats)
{
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(1.0f, 2.0f, PI_2);

    EXPECT_FLOAT_EQ(result.translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), PI_2, EPSILON);
}

TEST(PrimitivesPose, CreatePoseFromFloatsAndRotation)
{
    const Eigen::Rotation2Df rotation = Eigen::Rotation2Df(PI_2);
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(1.0f, 2.0f, rotation);

    EXPECT_FLOAT_EQ(result.translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), PI_2, EPSILON);
}

TEST(PrimitivesPose, CreatePoseFromTranslationAndFloat)
{
    const Eigen::Translation2f translation = Eigen::Translation2f(1.0f, 2.0f);
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(translation, PI_2);

    EXPECT_FLOAT_EQ(result.translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), PI_2, EPSILON);
}

TEST(PrimitivesPose, CreatePoseFromTranslationAndRotation)
{
    const Eigen::Translation2f translation = Eigen::Translation2f(1.0f, 2.0f);
    const Eigen::Rotation2Df rotation = Eigen::Rotation2Df(PI_2);
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(translation, rotation);

    EXPECT_FLOAT_EQ(result.translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), PI_2, EPSILON);
}

TEST(PrimitivesPose, CreatePoseFromVectorAndFloat)
{
    const Eigen::Vector2f vector = Eigen::Vector2f(1.0f, 2.0f);
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(vector, PI_2);

    EXPECT_FLOAT_EQ(result.translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), PI_2, EPSILON);
}

TEST(PrimitivesPose, CreatePoseFromVectorAndRotation)
{
    const Eigen::Vector2f vector = Eigen::Vector2f(1.0f, 2.0f);
    const Eigen::Rotation2Df rotation = Eigen::Rotation2Df(PI_2);
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(vector, rotation);

    EXPECT_FLOAT_EQ(result.translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), PI_2, EPSILON);
}

TEST(PrimitivesPose, ZeroRotation)
{
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(1.0f, 2.0f, 0.0f);

    EXPECT_FLOAT_EQ(result.translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), 0.0f, EPSILON);
}

TEST(PrimitivesPose, ZeroTranslation)
{
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(0.0f, 0.0f, PI_2);

    EXPECT_FLOAT_EQ(result.translation().x(), 0.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), 0.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), PI_2, EPSILON);
}

TEST(PrimitivesPose, NegativeCoordinates)
{
    const Eigen::Affine2f result = murosim::geometry::primitives::pose(-1.0f, -2.0f, -PI_4);

    EXPECT_FLOAT_EQ(result.translation().x(), -1.0f);
    EXPECT_FLOAT_EQ(result.translation().y(), -2.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(result.rotation()).angle(), -PI_4, EPSILON);
}

// ============================================================================
// Square Creation Tests
// ============================================================================

TEST(PrimitivesSquare, CreateBasicSquare)
{
    murosim::geometry::Shape square = murosim::geometry::primitives::square(2.0f);

    ASSERT_EQ(square.getLines().size(), static_cast<size_t>(4));
    EXPECT_EQ(square.getCircles().size(), static_cast<size_t>(0));

    // Verify it forms a closed square
    const Eigen::Vector2f p1 = Eigen::Vector2f(1.0f, 1.0f);
    const Eigen::Vector2f p2 = Eigen::Vector2f(-1.0f, 1.0f);
    const Eigen::Vector2f p3 = Eigen::Vector2f(-1.0f, -1.0f);
    const Eigen::Vector2f p4 = Eigen::Vector2f(1.0f, -1.0f);

    // Line 1: top (p1 to p2)
    EXPECT_NEAR(square.getLines()[0].getPoint1().x(), p1.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[0].getPoint1().y(), p1.y(), EPSILON);
    EXPECT_NEAR(square.getLines()[0].getPoint2().x(), p2.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[0].getPoint2().y(), p2.y(), EPSILON);

    // Line 2: left (p2 to p3)
    EXPECT_NEAR(square.getLines()[1].getPoint1().x(), p2.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[1].getPoint1().y(), p2.y(), EPSILON);
    EXPECT_NEAR(square.getLines()[1].getPoint2().x(), p3.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[1].getPoint2().y(), p3.y(), EPSILON);

    // Line 3: bottom (p3 to p4)
    EXPECT_NEAR(square.getLines()[2].getPoint1().x(), p3.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[2].getPoint1().y(), p3.y(), EPSILON);
    EXPECT_NEAR(square.getLines()[2].getPoint2().x(), p4.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[2].getPoint2().y(), p4.y(), EPSILON);

    // Line 4: right (p4 to p1)
    EXPECT_NEAR(square.getLines()[3].getPoint1().x(), p4.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[3].getPoint1().y(), p4.y(), EPSILON);
    EXPECT_NEAR(square.getLines()[3].getPoint2().x(), p1.x(), EPSILON);
    EXPECT_NEAR(square.getLines()[3].getPoint2().y(), p1.y(), EPSILON);
}

TEST(PrimitivesSquare, CreateTransformedSquare)
{
    const Eigen::Affine2f pose = Eigen::Translation2f(1.0f, 2.0f) * Eigen::Rotation2Df(PI_2);
    murosim::geometry::Shape square = murosim::geometry::primitives::square(2.0f, pose);

    ASSERT_EQ(square.getLines().size(), static_cast<size_t>(4));

    const Eigen::Vector2f p1 = Eigen::Vector2f(1.0f, 1.0f);
    const Eigen::Vector2f p2 = Eigen::Vector2f(-1.0f, 1.0f);
    const Eigen::Vector2f p3 = Eigen::Vector2f(-1.0f, -1.0f);
    const Eigen::Vector2f p4 = Eigen::Vector2f(1.0f, -1.0f);

    EXPECT_NEAR(square.getLines()[0].getPoint1().x(), (pose * p1).x(), EPSILON);
    EXPECT_NEAR(square.getLines()[0].getPoint1().y(), (pose * p1).y(), EPSILON);
    EXPECT_NEAR(square.getLines()[0].getPoint2().x(), (pose * p2).x(), EPSILON);
    EXPECT_NEAR(square.getLines()[0].getPoint2().y(), (pose * p2).y(), EPSILON);
    EXPECT_NEAR(square.getLines()[1].getPoint2().x(), (pose * p3).x(), EPSILON);
    EXPECT_NEAR(square.getLines()[1].getPoint2().y(), (pose * p3).y(), EPSILON);
    EXPECT_NEAR(square.getLines()[2].getPoint2().x(), (pose * p4).x(), EPSILON);
    EXPECT_NEAR(square.getLines()[2].getPoint2().y(), (pose * p4).y(), EPSILON);
    EXPECT_NEAR(square.getLines()[3].getPoint2().x(), (pose * p1).x(), EPSILON);
    EXPECT_NEAR(square.getLines()[3].getPoint2().y(), (pose * p1).y(), EPSILON);
}

TEST(PrimitivesSquare, CreateZeroLengthSquare)
{
    murosim::geometry::Shape square = murosim::geometry::primitives::square(0.0f);

    ASSERT_EQ(square.getLines().size(), static_cast<size_t>(4));
    // All points should be at origin
    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_FLOAT_EQ(square.getLines()[i].getPoint1().x(), 0.0f);
        EXPECT_FLOAT_EQ(square.getLines()[i].getPoint1().y(), 0.0f);
        EXPECT_FLOAT_EQ(square.getLines()[i].getPoint2().x(), 0.0f);
        EXPECT_FLOAT_EQ(square.getLines()[i].getPoint2().y(), 0.0f);
        EXPECT_FLOAT_EQ(square.getLines()[i].getLength(), 0.0f);
    }
}

TEST(PrimitivesSquare, AllSidesEqualLength)
{
    murosim::geometry::Shape square = murosim::geometry::primitives::square(2.0f);

    ASSERT_EQ(square.getLines().size(), static_cast<size_t>(4));
    const float expected_length = 2.0f;

    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(square.getLines()[i].getLength(), expected_length, EPSILON);
    }
}

// ============================================================================
// Rectangle Creation Tests
// ============================================================================

TEST(PrimitivesRectangle, CreateBasicRectangle)
{
    murosim::geometry::Shape rectangle = murosim::geometry::primitives::rectangle(2.0f, 4.0f);

    ASSERT_EQ(rectangle.getLines().size(), static_cast<size_t>(4));
    EXPECT_EQ(rectangle.getCircles().size(), static_cast<size_t>(0));

    const Eigen::Vector2f p1 = Eigen::Vector2f(1.0f, 2.0f);
    const Eigen::Vector2f p2 = Eigen::Vector2f(-1.0f, 2.0f);
    const Eigen::Vector2f p3 = Eigen::Vector2f(-1.0f, -2.0f);
    const Eigen::Vector2f p4 = Eigen::Vector2f(1.0f, -2.0f);

    // Line 1: top
    EXPECT_NEAR(rectangle.getLines()[0].getPoint1().x(), p1.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[0].getPoint1().y(), p1.y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[0].getPoint2().x(), p2.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[0].getPoint2().y(), p2.y(), EPSILON);

    // Line 2: left
    EXPECT_NEAR(rectangle.getLines()[1].getPoint1().x(), p2.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[1].getPoint1().y(), p2.y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[1].getPoint2().x(), p3.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[1].getPoint2().y(), p3.y(), EPSILON);

    // Line 3: bottom
    EXPECT_NEAR(rectangle.getLines()[2].getPoint1().x(), p3.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[2].getPoint1().y(), p3.y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[2].getPoint2().x(), p4.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[2].getPoint2().y(), p4.y(), EPSILON);

    // Line 4: right
    EXPECT_NEAR(rectangle.getLines()[3].getPoint1().x(), p4.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[3].getPoint1().y(), p4.y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[3].getPoint2().x(), p1.x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[3].getPoint2().y(), p1.y(), EPSILON);
}

TEST(PrimitivesRectangle, CreateTransformedRectangle)
{
    const Eigen::Affine2f pose = Eigen::Translation2f(1.0f, 2.0f) * Eigen::Rotation2Df(PI_2);
    murosim::geometry::Shape rectangle = murosim::geometry::primitives::rectangle(2.0f, 4.0f, pose);

    ASSERT_EQ(rectangle.getLines().size(), static_cast<size_t>(4));

    const Eigen::Vector2f p1 = Eigen::Vector2f(1.0f, 2.0f);
    const Eigen::Vector2f p2 = Eigen::Vector2f(-1.0f, 2.0f);
    const Eigen::Vector2f p3 = Eigen::Vector2f(-1.0f, -2.0f);
    const Eigen::Vector2f p4 = Eigen::Vector2f(1.0f, -2.0f);

    EXPECT_NEAR(rectangle.getLines()[0].getPoint1().x(), (pose * p1).x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[0].getPoint1().y(), (pose * p1).y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[0].getPoint2().x(), (pose * p2).x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[0].getPoint2().y(), (pose * p2).y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[1].getPoint2().x(), (pose * p3).x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[1].getPoint2().y(), (pose * p3).y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[2].getPoint2().x(), (pose * p4).x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[2].getPoint2().y(), (pose * p4).y(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[3].getPoint2().x(), (pose * p1).x(), EPSILON);
    EXPECT_NEAR(rectangle.getLines()[3].getPoint2().y(), (pose * p1).y(), EPSILON);
}

TEST(PrimitivesRectangle, OppositeSidesEqualLength)
{
    murosim::geometry::Shape rectangle = murosim::geometry::primitives::rectangle(2.0f, 4.0f);

    ASSERT_EQ(rectangle.getLines().size(), static_cast<size_t>(4));

    // Top and bottom should have same length (width)
    EXPECT_NEAR(rectangle.getLines()[0].getLength(), 2.0f, EPSILON);
    EXPECT_NEAR(rectangle.getLines()[2].getLength(), 2.0f, EPSILON);

    // Left and right should have same length (height)
    EXPECT_NEAR(rectangle.getLines()[1].getLength(), 4.0f, EPSILON);
    EXPECT_NEAR(rectangle.getLines()[3].getLength(), 4.0f, EPSILON);
}

TEST(PrimitivesRectangle, SquareFromRectangle)
{
    // Rectangle with equal width and height should be a square
    murosim::geometry::Shape rectangle = murosim::geometry::primitives::rectangle(2.0f, 2.0f);

    ASSERT_EQ(rectangle.getLines().size(), static_cast<size_t>(4));

    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(rectangle.getLines()[i].getLength(), 2.0f, EPSILON);
    }
}

// ============================================================================
// Triangle Creation Tests
// ============================================================================

TEST(PrimitivesTriangle, CreateBasicTriangle)
{
    murosim::geometry::Shape triangle = murosim::geometry::primitives::triangle(1.0f);

    ASSERT_EQ(triangle.getLines().size(), static_cast<size_t>(3));
    EXPECT_EQ(triangle.getCircles().size(), static_cast<size_t>(0));

    const Eigen::Vector2f p1 = TRIANGLE_TOP;
    const Eigen::Vector2f p2 = TRIANGLE_LEFT;
    const Eigen::Vector2f p3 = TRIANGLE_RIGHT;

    // The three lines form a closed loop through the vertices
    EXPECT_NEAR(triangle.getLines()[0].getPoint1().x(), p1.x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[0].getPoint1().y(), p1.y(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[0].getPoint2().x(), p2.x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[0].getPoint2().y(), p2.y(), EPSILON);

    EXPECT_NEAR(triangle.getLines()[1].getPoint1().x(), p2.x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[1].getPoint1().y(), p2.y(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[1].getPoint2().x(), p3.x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[1].getPoint2().y(), p3.y(), EPSILON);

    EXPECT_NEAR(triangle.getLines()[2].getPoint1().x(), p3.x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[2].getPoint1().y(), p3.y(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[2].getPoint2().x(), p1.x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[2].getPoint2().y(), p1.y(), EPSILON);
}

TEST(PrimitivesTriangle, CreateTransformedTriangle)
{
    const Eigen::Affine2f pose = Eigen::Translation2f(1.0f, 2.0f) * Eigen::Rotation2Df(PI_2);
    murosim::geometry::Shape triangle = murosim::geometry::primitives::triangle(1.0f, pose);

    ASSERT_EQ(triangle.getLines().size(), static_cast<size_t>(3));

    EXPECT_NEAR(triangle.getLines()[0].getPoint1().x(), (pose * TRIANGLE_TOP).x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[0].getPoint1().y(), (pose * TRIANGLE_TOP).y(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[0].getPoint2().x(), (pose * TRIANGLE_LEFT).x(), EPSILON);
    EXPECT_NEAR(triangle.getLines()[0].getPoint2().y(), (pose * TRIANGLE_LEFT).y(), EPSILON);
}

TEST(PrimitivesTriangle, EquilateralTriangle)
{
    murosim::geometry::Shape triangle = murosim::geometry::primitives::triangle(1.0f);

    ASSERT_EQ(triangle.getLines().size(), static_cast<size_t>(3));

    // All sides of an equilateral triangle have the same length
    EXPECT_NEAR(triangle.getLines()[0].getLength(), 1.0f, EPSILON);
    EXPECT_NEAR(triangle.getLines()[1].getLength(), 1.0f, EPSILON);
    EXPECT_NEAR(triangle.getLines()[2].getLength(), 1.0f, EPSILON);
}

TEST(PrimitivesTriangle, ScaledTriangle)
{
    murosim::geometry::Shape triangle = murosim::geometry::primitives::triangle(2.0f);

    ASSERT_EQ(triangle.getLines().size(), static_cast<size_t>(3));

    // All sides should be scaled proportionally
    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(triangle.getLines()[i].getLength(), 2.0f, EPSILON);
    }
}

TEST(PrimitivesTriangle, CentroidAtOrigin)
{
    murosim::geometry::Shape triangle = murosim::geometry::primitives::triangle(1.0f);

    ASSERT_EQ(triangle.getLines().size(), static_cast<size_t>(3));

    // The lines form a closed loop, so the first point of each line visits
    // every vertex exactly once
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (size_t i = 0; i < 3; ++i)
    {
        centroid += triangle.getLines()[i].getPoint1();
    }
    centroid /= 3.0f;

    EXPECT_NEAR(centroid.x(), 0.0f, EPSILON);
    EXPECT_NEAR(centroid.y(), 0.0f, EPSILON);
}
