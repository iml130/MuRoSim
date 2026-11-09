// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/geometry/shape.hpp"
#include "murosim/math/constants.hpp"
#include <sstream>
#include <utility>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::geometry::Shape;

constexpr float EPSILON = 1e-5f;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(ShapeConstructor, DefaultConstructor)
{
    Shape shape;

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(0));
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 0.0f);
}

TEST(ShapeConstructor, ParameterizedConstructorWithVectors)
{
    std::vector<Line> lines = {Line(1.0f, 2.0f, 3.0f, 4.0f)};
    std::vector<Circle> circles = {Circle(1.0f, -1.0f, 1.0f)};

    Shape shape(lines, circles);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getX(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getY(), -1.0f);
}

TEST(ShapeConstructor, ParameterizedConstructorWithInitializerLists)
{
    Shape shape({{1.0f, 2.0f, 3.0f, 4.0f}}, {{1.0f, -1.0f, 1.0f}});

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(1));
}

TEST(ShapeConstructor, EmptyVectors)
{
    std::vector<Line> lines;
    std::vector<Circle> circles;

    Shape shape(lines, circles);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(0));
}

TEST(ShapeConstructor, MultipleElements)
{
    std::vector<Line> lines = {
        Line(1.0f, 2.0f, 3.0f, 4.0f),
        Line(5.0f, 6.0f, 7.0f, 8.0f)
    };
    std::vector<Circle> circles = {
        Circle(1.0f, 1.0f, 1.0f),
        Circle(2.0f, 2.0f, 2.0f),
        Circle(3.0f, 3.0f, 3.0f)
    };

    Shape shape(lines, circles);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(2));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(3));
}

TEST(ShapeConstructor, DefaultConstructorHasZeroBounds)
{
    Shape empty;

    EXPECT_EQ(empty.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(empty.getCircles().size(), static_cast<size_t>(0));
    EXPECT_FLOAT_EQ(empty.getAabbMinX(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbMinY(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbMaxX(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbMaxY(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbSizeX(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbSizeY(), 0.0f);

    EXPECT_FLOAT_EQ(empty.getAabbMin().x(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbMin().y(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(empty.getAabbMax().y(), 0.0f);
}

// ============================================================================
// Test Fixture
// ============================================================================

class ShapeTest : public ::testing::Test
{
protected:
    // One line from (1, 2) to (3, 4) and one unit circle at (1, -1)
    Shape shape{{{1.0f, 2.0f, 3.0f, 4.0f}}, {{1.0f, -1.0f, 1.0f}}};
};

// ============================================================================
// Getter Tests
// ============================================================================

TEST_F(ShapeTest, GetLineAccessors)
{
    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint2X(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint2Y(), 4.0f);
}

TEST_F(ShapeTest, GetCircleAccessors)
{
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getX(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getY(), -1.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getRadius(), 1.0f);
}

TEST_F(ShapeTest, GetAreaBounds)
{
    EXPECT_FLOAT_EQ(shape.getAabbMinX(), 0.0f);
    EXPECT_FLOAT_EQ(shape.getAabbMinY(), -2.0f);
    EXPECT_FLOAT_EQ(shape.getAabbMaxX(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getAabbMaxY(), 4.0f);

    EXPECT_FLOAT_EQ(shape.getAabbMin().x(), 0.0f);
    EXPECT_FLOAT_EQ(shape.getAabbMin().y(), -2.0f);
    EXPECT_FLOAT_EQ(shape.getAabbMax().x(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getAabbMax().y(), 4.0f);
}

TEST_F(ShapeTest, GetAreaLengths)
{
    EXPECT_FLOAT_EQ(shape.getAabbSizeX(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getAabbSizeY(), 6.0f);

    EXPECT_FLOAT_EQ(shape.getAabbSize().x(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getAabbSize().y(), 6.0f);
}

// ============================================================================
// Add Element Tests
// ============================================================================

TEST_F(ShapeTest, AddLineWithComponents)
{
    shape.addLine(2.0f, 3.0f, 4.0f, 5.0f);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1X(), 2.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1Y(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint2X(), 4.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint2Y(), 5.0f);
}

TEST_F(ShapeTest, AddLineWithObject)
{
    const Line line(6.0f, 7.0f, 8.0f, 9.0f);
    shape.addLine(line);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1X(), 6.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1Y(), 7.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint2X(), 8.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint2Y(), 9.0f);
}

TEST_F(ShapeTest, AddLineWithVectors)
{
    shape.addLine(Eigen::Vector2f(10.0f, 11.0f), Eigen::Vector2f(12.0f, 13.0f));

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1X(), 10.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1Y(), 11.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint2X(), 12.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint2Y(), 13.0f);
}

TEST_F(ShapeTest, AddLineMultiple)
{
    std::vector<Line> lines = {
        Line(1.0f, 1.0f, 2.0f, 2.0f),
        Line(3.0f, 3.0f, 4.0f, 4.0f)
    };
    shape.addLine(lines);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(3));
}

TEST_F(ShapeTest, AddCircleWithComponents)
{
    shape.addCircle(2.0f, -2.0f, 2.0f);

    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getX(), 2.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getY(), -2.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getRadius(), 2.0f);
}

TEST_F(ShapeTest, AddCircleWithObject)
{
    const Circle circle(3.0f, 3.0f, 3.0f);
    shape.addCircle(circle);

    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getX(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getY(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getRadius(), 3.0f);
}

TEST_F(ShapeTest, AddCircleWithVectorAndRadius)
{
    shape.addCircle(Eigen::Vector2f(4.0f, 4.0f), 4.0f);

    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getX(), 4.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getY(), 4.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getRadius(), 4.0f);
}

TEST_F(ShapeTest, AddCircleMultiple)
{
    std::vector<Circle> circles = {
        Circle(1.0f, 1.0f, 1.0f),
        Circle(2.0f, 2.0f, 2.0f)
    };
    shape.addCircle(circles);

    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(3));
}

TEST_F(ShapeTest, AddShape)
{
    const Shape other({{10.0f, 11.0f, 12.0f, 13.0f}}, {{10.0f, -10.0f, 10.0f}});
    shape.addShape(other);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(2));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1X(), 10.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1Y(), 11.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getX(), 10.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getY(), -10.0f);
}

TEST_F(ShapeTest, AddShapeWithTransform)
{
    Shape other({{0.0f, 0.0f, 1.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}});
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.translate(Eigen::Vector2f(5.0f, 5.0f));

    shape.addShape(other, affine);

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(2));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(2));
    EXPECT_NEAR(shape.getLines()[1].getPoint1X(), 5.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[1].getPoint1Y(), 5.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[1].getX(), 5.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[1].getY(), 5.0f, EPSILON);
}

// ============================================================================
// Clear Tests
// ============================================================================

TEST_F(ShapeTest, ClearLines)
{
    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(1));

    shape.clearLines();

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(1));
}

TEST_F(ShapeTest, ClearCircles)
{
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(1));

    shape.clearCircles();

    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(1));
}

TEST_F(ShapeTest, ClearShape)
{
    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(1));

    shape.clearShape();

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(0));
}

TEST_F(ShapeTest, ClearShapeWithManyElements)
{
    for (float i = 1.0f; i <= 4.0f; i += 1.0f)
    {
        shape.addLine(i, i, i, i);
    }
    for (float i = 1.0f; i <= 2.0f; i += 1.0f)
    {
        shape.addCircle(i, i, i);
    }
    ASSERT_EQ(shape.getLines().size(), static_cast<size_t>(5));
    ASSERT_EQ(shape.getCircles().size(), static_cast<size_t>(3));

    shape.clearShape();

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(0));
}

// ============================================================================
// Transform Tests
// ============================================================================

TEST_F(ShapeTest, TransformWithComponents)
{
    shape.transform(Eigen::Rotation2Df(PI) * Eigen::Translation2f(0.0f, 0.0f));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), -2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), -3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), -4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getRadius(), 1.0f);
}

TEST_F(ShapeTest, TransformWithAffine)
{
    const Eigen::Affine2f affine = Eigen::Rotation2Df(PI) * Eigen::Translation2f(0.0f, 0.0f);
    shape.transform(affine);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), -2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), -3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), -4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
}

TEST_F(ShapeTest, TransformMultiplicationOperator)
{
    const Eigen::Affine2f affine = Eigen::Rotation2Df(0.0f) * Eigen::Translation2f(1.0f, 1.0f);
    shape = affine * shape;

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 5.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 0.0f, EPSILON);
}

TEST_F(ShapeTest, TransformWithScaling)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.scale(Eigen::Vector2f(2.0f, 2.0f));

    shape.transform(affine);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 8.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getRadius(), 2.0f, EPSILON);
}

TEST_F(ShapeTest, SetPose)
{
    shape.setPose(5.0f, 5.0f, PI_2);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 6.0f, EPSILON);
}

TEST_F(ShapeTest, TransformRoundTrip)
{
    shape.transform(Eigen::Rotation2Df(PI) * Eigen::Translation2f(0.0f, 0.0f));
    shape.transform(Eigen::Rotation2Df(-PI) * Eigen::Translation2f(0.0f, 0.0f));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -1.0f, EPSILON);
}

TEST_F(ShapeTest, TransformPoseRotatesThenTranslates)
{
    // pose (5, 0, 90 deg): rotate about the origin, THEN translate by +5 in x
    // line (1, 2) -> (-2, 1) -> (3, 1), (3, 4) -> (-4, 3) -> (1, 3); circle (1, -1) -> (1, 1) -> (6, 1)
    shape.transform(5.0f, 0.0f, PI_2);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
}

TEST_F(ShapeTest, SetPoseMatchesTransformPose)
{
    // setPose is an alias for transform(x, y, angle)
    Shape other = shape;
    shape.transform(5.0f, 0.0f, PI_2);
    other.setPose(5.0f, 0.0f, PI_2);

    EXPECT_FLOAT_EQ(other.getLines()[0].getPoint1X(), shape.getLines()[0].getPoint1X());
    EXPECT_FLOAT_EQ(other.getLines()[0].getPoint1Y(), shape.getLines()[0].getPoint1Y());
    EXPECT_FLOAT_EQ(other.getLines()[0].getPoint2X(), shape.getLines()[0].getPoint2X());
    EXPECT_FLOAT_EQ(other.getLines()[0].getPoint2Y(), shape.getLines()[0].getPoint2Y());
    EXPECT_FLOAT_EQ(other.getCircles()[0].getX(), shape.getCircles()[0].getX());
    EXPECT_FLOAT_EQ(other.getCircles()[0].getY(), shape.getCircles()[0].getY());
}

// ============================================================================
// Translation Tests
// ============================================================================

TEST_F(ShapeTest, TranslateWithComponents)
{
    shape.translate(1.0f, 2.0f);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
}

TEST_F(ShapeTest, TranslateWithVector)
{
    shape.translate(Eigen::Vector2f(1.0f, 2.0f));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
}

TEST_F(ShapeTest, TranslateMultipleTimes)
{
    shape.translate(1.0f, 2.0f);
    shape.translate(Eigen::Vector2f(-1.0f, -2.0f));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -1.0f, EPSILON);
}

// ============================================================================
// Rotation Tests
// ============================================================================

TEST_F(ShapeTest, RotateWithFloat)
{
    shape.rotate(PI);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), -2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), -3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), -4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
}

TEST_F(ShapeTest, RotateWithRotation2D)
{
    shape.rotate(Eigen::Rotation2Df(PI));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), -2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), -3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), -4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
}

TEST_F(ShapeTest, RotateFullCircle)
{
    shape.rotate(PI);
    shape.rotate(Eigen::Rotation2Df(-PI));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -1.0f, EPSILON);
}

// ============================================================================
// Scale Tests
// ============================================================================

TEST_F(ShapeTest, ScaleUniform)
{
    shape.scale(2.0f);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 8.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getRadius(), 2.0f, EPSILON);
}

TEST_F(ShapeTest, ScaleNonUniform)
{
    shape.scale(2.0f, 3.0f);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 12.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -3.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getRadius(), 2.0f, EPSILON);
}

TEST_F(ShapeTest, ScaleWithVector)
{
    shape.scale(Eigen::Vector2f(2.0f, 3.0f));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 6.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 12.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -3.0f, EPSILON);
}

TEST_F(ShapeTest, ScaleNegative)
{
    shape.scale(-1.0f, -1.0f);

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), -2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), -3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), -4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), -1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getRadius(), 1.0f, EPSILON);
}

TEST_F(ShapeTest, ScaleRoundTrip)
{
    shape.scale(2.0f, 3.0f);
    shape.scale(Eigen::Vector2f(0.5f, 1.0f / 3.0f));

    EXPECT_NEAR(shape.getLines()[0].getPoint1X(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint1Y(), 2.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2X(), 3.0f, EPSILON);
    EXPECT_NEAR(shape.getLines()[0].getPoint2Y(), 4.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getX(), 1.0f, EPSILON);
    EXPECT_NEAR(shape.getCircles()[0].getY(), -1.0f, EPSILON);
}

// ============================================================================
// Collision Radius Tests
// ============================================================================

TEST(ShapeCollisionRadius, EmptyShape)
{
    Shape shape;
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 0.0f);
}

TEST(ShapeCollisionRadius, WithCircles)
{
    Shape shape;

    shape.addCircle(0.0f, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 1.0f);

    shape.addCircle(0.0f, 0.0f, 2.0f);
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 2.0f);

    shape.addCircle(0.0f, 0.0f, 3.0f);
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 3.0f);
}

TEST(ShapeCollisionRadius, WithLines)
{
    Shape shape;

    shape.addLine(0.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 1.0f);

    shape.addLine(0.0f, 0.0f, 0.0f, -4.0f);
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 4.0f);
}

TEST(ShapeCollisionRadius, Combined)
{
    Shape shape;

    shape.addCircle(0.0f, 0.0f, 1.0f);
    shape.addLine(0.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 1.0f);

    shape.addCircle(0.0f, 0.0f, 2.0f);
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 2.0f);

    shape.clearCircles();
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 1.0f);

    shape.clearLines();
    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 0.0f);
}

TEST(ShapeCollisionRadius, LargestDistanceWins)
{
    Shape shape;

    shape.addCircle(0.0f, 0.0f, 5.0f);
    shape.addLine(0.0f, 0.0f, 3.0f, 4.0f);

    EXPECT_FLOAT_EQ(shape.getCollisionRadius(), 5.0f);
}

// ============================================================================
// CopyTo Tests
// ============================================================================

TEST_F(ShapeTest, CopyToEmptyVectors)
{
    std::vector<Line> lines;
    std::vector<Circle> circles;

    shape.copyTo(lines, circles);

    EXPECT_EQ(lines.size(), static_cast<size_t>(1));
    EXPECT_EQ(circles.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(lines[0].getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(circles[0].getX(), 1.0f);
}

TEST_F(ShapeTest, CopyToPopulatedVectors)
{
    std::vector<Line> lines = {Line(0.0f, 0.0f, 0.0f, 0.0f)};
    std::vector<Circle> circles = {Circle(0.0f, 0.0f, 0.0f)};

    shape.copyTo(lines, circles);

    EXPECT_EQ(lines.size(), static_cast<size_t>(2));
    EXPECT_EQ(circles.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(lines[1].getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(circles[1].getX(), 1.0f);
}

TEST(ShapeCopyTo, EmptyShape)
{
    Shape shape;
    std::vector<Line> lines;
    std::vector<Circle> circles;

    shape.copyTo(lines, circles);

    EXPECT_EQ(lines.size(), static_cast<size_t>(0));
    EXPECT_EQ(circles.size(), static_cast<size_t>(0));
}

// ============================================================================
// Move Semantics Tests
// ============================================================================

TEST(ShapeMoveSemantics, AddLineRValue)
{
    Shape shape;
    Line line(1.0f, 2.0f, 3.0f, 4.0f);

    shape.addLine(std::move(line));

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint2X(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint2Y(), 4.0f);
}

TEST(ShapeMoveSemantics, AddLineVectorRValue)
{
    Shape shape;
    std::vector<Line> lines = {
        Line(1.0f, 2.0f, 3.0f, 4.0f),
        Line(5.0f, 6.0f, 7.0f, 8.0f)
    };

    shape.addLine(std::move(lines));

    EXPECT_EQ(shape.getLines().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(shape.getLines()[0].getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getLines()[1].getPoint1X(), 5.0f);
}

TEST(ShapeMoveSemantics, AddCircleRValue)
{
    Shape shape;
    Circle circle(1.0f, 2.0f, 0.5f);

    shape.addCircle(std::move(circle));

    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getX(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getY(), 2.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getRadius(), 0.5f);
}

TEST(ShapeMoveSemantics, AddCircleVectorRValue)
{
    Shape shape;
    std::vector<Circle> circles = {
        Circle(1.0f, 2.0f, 0.5f),
        Circle(3.0f, 4.0f, 1.0f),
        Circle(5.0f, 6.0f, 1.5f)
    };

    shape.addCircle(std::move(circles));

    EXPECT_EQ(shape.getCircles().size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(shape.getCircles()[0].getX(), 1.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[1].getX(), 3.0f);
    EXPECT_FLOAT_EQ(shape.getCircles()[2].getX(), 5.0f);
}

TEST(ShapeMoveSemantics, AddShapeRValue)
{
    Shape shape1;
    shape1.addLine(1.0f, 2.0f, 3.0f, 4.0f);
    shape1.addCircle(5.0f, 6.0f, 0.5f);

    Shape shape2;
    shape2.addLine(10.0f, 20.0f, 30.0f, 40.0f);

    shape2.addShape(std::move(shape1));

    EXPECT_EQ(shape2.getLines().size(), static_cast<size_t>(2));
    EXPECT_EQ(shape2.getCircles().size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shape2.getLines()[0].getPoint1X(), 10.0f);
    EXPECT_FLOAT_EQ(shape2.getLines()[1].getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(shape2.getCircles()[0].getX(), 5.0f);

    // Original shape should be cleared after move
    EXPECT_EQ(shape1.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape1.getCircles().size(), static_cast<size_t>(0));
}

TEST(ShapeMoveSemantics, AddShapeRValueWithTransform)
{
    Shape shape1;
    shape1.addLine(1.0f, 0.0f, 2.0f, 0.0f);
    shape1.addCircle(0.0f, 0.0f, 1.0f);

    Shape shape2;

    Eigen::Affine2f transform = Eigen::Affine2f::Identity();
    transform.translate(Eigen::Vector2f(10.0f, 20.0f));

    shape2.addShape(std::move(shape1), transform);

    EXPECT_EQ(shape2.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(shape2.getCircles().size(), static_cast<size_t>(1));
    EXPECT_NEAR(shape2.getLines()[0].getPoint1X(), 11.0f, EPSILON);
    EXPECT_NEAR(shape2.getLines()[0].getPoint1Y(), 20.0f, EPSILON);
    EXPECT_NEAR(shape2.getCircles()[0].getX(), 10.0f, EPSILON);
    EXPECT_NEAR(shape2.getCircles()[0].getY(), 20.0f, EPSILON);

    // Original shape should be cleared after move
    EXPECT_EQ(shape1.getLines().size(), static_cast<size_t>(0));
    EXPECT_EQ(shape1.getCircles().size(), static_cast<size_t>(0));
}

// ============================================================================
// Indexed Access Tests
// ============================================================================

TEST(ShapeIndexedAccess, GetLineSize)
{
    Shape shape;
    shape.addLine(1.0f, 2.0f, 3.0f, 4.0f);
    shape.addLine(5.0f, 6.0f, 7.0f, 8.0f);
    shape.addLine(9.0f, 10.0f, 11.0f, 12.0f);

    EXPECT_EQ(shape.getLineSize(), static_cast<size_t>(3));
}

TEST(ShapeIndexedAccess, GetLineSizeEmpty)
{
    Shape shape;

    EXPECT_EQ(shape.getLineSize(), static_cast<size_t>(0));
}

TEST(ShapeIndexedAccess, GetLineAt)
{
    Shape shape;
    shape.addLine(1.0f, 2.0f, 3.0f, 4.0f);
    shape.addLine(5.0f, 6.0f, 7.0f, 8.0f);
    shape.addLine(9.0f, 10.0f, 11.0f, 12.0f);

    const Line line0 = shape.getLineAt(0);
    EXPECT_FLOAT_EQ(line0.getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(line0.getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(line0.getPoint2X(), 3.0f);
    EXPECT_FLOAT_EQ(line0.getPoint2Y(), 4.0f);

    const Line line1 = shape.getLineAt(1);
    EXPECT_FLOAT_EQ(line1.getPoint1X(), 5.0f);
    EXPECT_FLOAT_EQ(line1.getPoint1Y(), 6.0f);
    EXPECT_FLOAT_EQ(line1.getPoint2X(), 7.0f);
    EXPECT_FLOAT_EQ(line1.getPoint2Y(), 8.0f);

    const Line line2 = shape.getLineAt(2);
    EXPECT_FLOAT_EQ(line2.getPoint1X(), 9.0f);
    EXPECT_FLOAT_EQ(line2.getPoint1Y(), 10.0f);
    EXPECT_FLOAT_EQ(line2.getPoint2X(), 11.0f);
    EXPECT_FLOAT_EQ(line2.getPoint2Y(), 12.0f);
}

TEST(ShapeIndexedAccess, GetLineAtConsistentWithVector)
{
    Shape shape;
    shape.addLine(1.0f, 2.0f, 3.0f, 4.0f);
    shape.addLine(5.0f, 6.0f, 7.0f, 8.0f);
    ASSERT_EQ(shape.getLineSize(), static_cast<size_t>(2));

    for (size_t i = 0; i < shape.getLineSize(); i++)
    {
        const Line lineAt = shape.getLineAt(i);
        const Line& lineVector = shape.getLines()[i];

        EXPECT_FLOAT_EQ(lineAt.getPoint1X(), lineVector.getPoint1X());
        EXPECT_FLOAT_EQ(lineAt.getPoint1Y(), lineVector.getPoint1Y());
        EXPECT_FLOAT_EQ(lineAt.getPoint2X(), lineVector.getPoint2X());
        EXPECT_FLOAT_EQ(lineAt.getPoint2Y(), lineVector.getPoint2Y());
    }
}

TEST(ShapeIndexedAccess, GetCircleSize)
{
    Shape shape;
    shape.addCircle(1.0f, 2.0f, 0.5f);
    shape.addCircle(3.0f, 4.0f, 1.0f);
    shape.addCircle(5.0f, 6.0f, 1.5f);
    shape.addCircle(7.0f, 8.0f, 2.0f);

    EXPECT_EQ(shape.getCircleSize(), static_cast<size_t>(4));
}

TEST(ShapeIndexedAccess, GetCircleSizeEmpty)
{
    Shape shape;

    EXPECT_EQ(shape.getCircleSize(), static_cast<size_t>(0));
}

TEST(ShapeIndexedAccess, GetCircleAt)
{
    Shape shape;
    shape.addCircle(1.0f, 2.0f, 0.5f);
    shape.addCircle(3.0f, 4.0f, 1.0f);
    shape.addCircle(5.0f, 6.0f, 1.5f);

    const Circle circle0 = shape.getCircleAt(0);
    EXPECT_FLOAT_EQ(circle0.getX(), 1.0f);
    EXPECT_FLOAT_EQ(circle0.getY(), 2.0f);
    EXPECT_FLOAT_EQ(circle0.getRadius(), 0.5f);

    const Circle circle1 = shape.getCircleAt(1);
    EXPECT_FLOAT_EQ(circle1.getX(), 3.0f);
    EXPECT_FLOAT_EQ(circle1.getY(), 4.0f);
    EXPECT_FLOAT_EQ(circle1.getRadius(), 1.0f);

    const Circle circle2 = shape.getCircleAt(2);
    EXPECT_FLOAT_EQ(circle2.getX(), 5.0f);
    EXPECT_FLOAT_EQ(circle2.getY(), 6.0f);
    EXPECT_FLOAT_EQ(circle2.getRadius(), 1.5f);
}

TEST(ShapeIndexedAccess, GetCircleAtConsistentWithVector)
{
    Shape shape;
    shape.addCircle(1.0f, 2.0f, 0.5f);
    shape.addCircle(3.0f, 4.0f, 1.0f);
    ASSERT_EQ(shape.getCircleSize(), static_cast<size_t>(2));

    for (size_t i = 0; i < shape.getCircleSize(); i++)
    {
        const Circle circleAt = shape.getCircleAt(i);
        const Circle& circleVector = shape.getCircles()[i];

        EXPECT_FLOAT_EQ(circleAt.getX(), circleVector.getX());
        EXPECT_FLOAT_EQ(circleAt.getY(), circleVector.getY());
        EXPECT_FLOAT_EQ(circleAt.getRadius(), circleVector.getRadius());
    }
}

// ============================================================================
// Stream Operator Tests
// ============================================================================

TEST(ShapeStreamOperator, EmptyShape)
{
    const Shape shape;
    std::ostringstream oss;

    oss << shape;

    EXPECT_EQ(oss.str(), "Shape(lines={}, circles={})");
}

TEST(ShapeStreamOperator, ShapeWithLines)
{
    Shape shape;
    shape.addLine(1.0f, 2.0f, 3.0f, 4.0f);
    shape.addLine(5.0f, 6.0f, 7.0f, 8.0f);

    std::ostringstream oss;
    oss << shape;

    EXPECT_EQ(oss.str(), "Shape(lines={(1, 2, 3, 4), (5, 6, 7, 8)}, circles={})");
}

TEST(ShapeStreamOperator, ShapeWithCircles)
{
    Shape shape;
    shape.addCircle(1.0f, 2.0f, 0.5f);
    shape.addCircle(3.0f, 4.0f, 1.0f);

    std::ostringstream oss;
    oss << shape;

    EXPECT_EQ(oss.str(), "Shape(lines={}, circles={(1, 2, 0.5), (3, 4, 1)})");
}

TEST(ShapeStreamOperator, ShapeWithBoth)
{
    Shape shape;
    shape.addLine(1.0f, 2.0f, 3.0f, 4.0f);
    shape.addCircle(5.0f, 6.0f, 0.5f);

    std::ostringstream oss;
    oss << shape;

    EXPECT_EQ(oss.str(), "Shape(lines={(1, 2, 3, 4)}, circles={(5, 6, 0.5)})");
}
