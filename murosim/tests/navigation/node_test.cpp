// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/navigation/node.hpp"
#include "murosim/math/constants.hpp"

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
constexpr float EPSILON = 1e-5f;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(NodeConstructor, DefaultConstructor)
{
    murosim::navigation::Node node;

    EXPECT_FLOAT_EQ(node.getX(), 0.0f);
    EXPECT_FLOAT_EQ(node.getY(), 0.0f);
    EXPECT_FLOAT_EQ(node.getRadius(), 0.0f);
    EXPECT_FLOAT_EQ(node.getStartAngle(), 0.0f);
    EXPECT_FLOAT_EQ(node.getStartBias(), 0.0f);
    EXPECT_EQ(node.getId(), static_cast<size_t>(0));
    EXPECT_EQ(node.getGroup(), static_cast<size_t>(0));
}

TEST(NodeConstructor, CircleOnly)
{
    const murosim::geometry::Circle circle(1.0f, 2.0f, 0.5f);
    murosim::navigation::Node node(circle);

    EXPECT_FLOAT_EQ(node.getX(), 1.0f);
    EXPECT_FLOAT_EQ(node.getY(), 2.0f);
    EXPECT_FLOAT_EQ(node.getRadius(), 0.5f);
    EXPECT_FLOAT_EQ(node.getStartAngle(), 0.0f);
}

TEST(NodeConstructor, CircleWithAngle)
{
    const murosim::geometry::Circle circle(1.0f, 2.0f, 0.5f);
    murosim::navigation::Node node(circle, PI_2);

    EXPECT_FLOAT_EQ(node.getX(), 1.0f);
    EXPECT_FLOAT_EQ(node.getY(), 2.0f);
    EXPECT_FLOAT_EQ(node.getStartAngle(), PI_2);
}

TEST(NodeConstructor, CircleWithAngleAndBias)
{
    const murosim::geometry::Circle circle(1.0f, 2.0f, 0.5f);
    murosim::navigation::Node node(circle, PI_2, 0.1f);

    EXPECT_FLOAT_EQ(node.getStartAngle(), PI_2);
    EXPECT_FLOAT_EQ(node.getStartBias(), 0.1f);
}

TEST(NodeConstructor, CircleWithId)
{
    const murosim::geometry::Circle circle(1.0f, 2.0f, 0.5f);
    murosim::navigation::Node node(circle, PI_2, 0.1f, 42);

    EXPECT_EQ(node.getId(), static_cast<size_t>(42));
    EXPECT_EQ(node.getGroup(), static_cast<size_t>(0));
}

TEST(NodeConstructor, CircleFullParams)
{
    const murosim::geometry::Circle circle(1.0f, 2.0f, 0.5f);
    murosim::navigation::Node node(circle, PI_2, 0.1f, 42, 5);

    EXPECT_EQ(node.getId(), static_cast<size_t>(42));
    EXPECT_EQ(node.getGroup(), static_cast<size_t>(5));
}

TEST(NodeConstructor, FloatsMinimal)
{
    murosim::navigation::Node node(1.0f, 2.0f, 0.5f);

    EXPECT_FLOAT_EQ(node.getX(), 1.0f);
    EXPECT_FLOAT_EQ(node.getY(), 2.0f);
    EXPECT_FLOAT_EQ(node.getRadius(), 0.5f);
}

TEST(NodeConstructor, FloatsWithAngle)
{
    murosim::navigation::Node node(1.0f, 2.0f, 0.5f, PI);

    EXPECT_FLOAT_EQ(node.getStartAngle(), PI);
}

TEST(NodeConstructor, FloatsWithBias)
{
    murosim::navigation::Node node(1.0f, 2.0f, 0.5f, PI, 0.2f);

    EXPECT_FLOAT_EQ(node.getStartBias(), 0.2f);
}

TEST(NodeConstructor, FloatsWithId)
{
    murosim::navigation::Node node(1.0f, 2.0f, 0.5f, PI, 0.2f, 10);

    EXPECT_EQ(node.getId(), static_cast<size_t>(10));
}

TEST(NodeConstructor, FloatsFullParams)
{
    murosim::navigation::Node node(1.0f, 2.0f, 0.5f, PI, 0.2f, 10, 3);

    EXPECT_EQ(node.getId(), static_cast<size_t>(10));
    EXPECT_EQ(node.getGroup(), static_cast<size_t>(3));
}

// ============================================================================
// Setter Tests
// ============================================================================

TEST(NodeSetter, SetId)
{
    murosim::navigation::Node node;
    node.setId(123);

    EXPECT_EQ(node.getId(), static_cast<size_t>(123));
}

TEST(NodeSetter, SetGroup)
{
    murosim::navigation::Node node;
    node.setGroup(7);

    EXPECT_EQ(node.getGroup(), static_cast<size_t>(7));
}

TEST(NodeSetter, SetGoalRadius)
{
    murosim::navigation::Node node;
    node.setGoalRadius(2.5f);

    EXPECT_FLOAT_EQ(node.getGoalRadius(), 2.5f);
}

TEST(NodeSetter, SetStartAngle)
{
    murosim::navigation::Node node;
    node.setStartAngle(PI_4);

    EXPECT_FLOAT_EQ(node.getStartAngle(), PI_4);
}

TEST(NodeSetter, SetStartBias)
{
    murosim::navigation::Node node;
    node.setStartBias(0.5f);

    EXPECT_FLOAT_EQ(node.getStartBias(), 0.5f);
}

// ============================================================================
// Getter Tests
// ============================================================================

TEST(NodeGetter, GetCircle)
{
    murosim::navigation::Node node(1.0f, 2.0f, 0.5f);
    const murosim::geometry::Circle circle = node.getCircle();

    EXPECT_FLOAT_EQ(circle.getX(), 1.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 0.5f);
}

TEST(NodeGetter, GetPoint)
{
    murosim::navigation::Node node(3.0f, 4.0f, 0.5f);
    const Eigen::Vector2f point = node.getPoint();

    EXPECT_FLOAT_EQ(point.x(), 3.0f);
    EXPECT_FLOAT_EQ(point.y(), 4.0f);
}

TEST(NodeGetter, GetOrientation)
{
    murosim::navigation::Node node(0.0f, 0.0f, 1.0f, PI_2);
    const Eigen::Vector2f orientation = node.getOrientation();

    // At PI/2, orientation should be (0, 1)
    EXPECT_NEAR(orientation.x(), 0.0f, EPSILON);
    EXPECT_NEAR(orientation.y(), 1.0f, EPSILON);
}

TEST(NodeGetter, GetOrientationZeroAngle)
{
    murosim::navigation::Node node(0.0f, 0.0f, 1.0f, 0.0f);
    const Eigen::Vector2f orientation = node.getOrientation();

    // At 0, orientation should be (1, 0)
    EXPECT_NEAR(orientation.x(), 1.0f, EPSILON);
    EXPECT_NEAR(orientation.y(), 0.0f, EPSILON);
}

// ============================================================================
// Transform Tests
// ============================================================================

TEST(NodeTransform, TransformAffine)
{
    murosim::navigation::Node node(1.0f, 0.0f, 0.5f, 0.0f);

    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.translate(Eigen::Vector2f(2.0f, 0.0f));
    node.transform(affine);

    EXPECT_NEAR(node.getX(), 3.0f, EPSILON);
    EXPECT_NEAR(node.getY(), 0.0f, EPSILON);
}

TEST(NodeTransform, TransformAffineTracksOrientation)
{
    murosim::navigation::Node node(1.0f, 0.0f, 0.5f, 0.0f);

    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.rotate(PI_2);
    node.transform(affine);

    // center rotated 90 deg about the origin: (1, 0) -> (0, 1)
    EXPECT_NEAR(node.getX(), 0.0f, EPSILON);
    EXPECT_NEAR(node.getY(), 1.0f, EPSILON);
    // stored orientation advanced by 90 deg
    EXPECT_NEAR(node.getStartAngle(), PI_2, EPSILON);
}

TEST(NodeTransform, RotateAngle)
{
    murosim::navigation::Node node(1.0f, 0.0f, 0.5f, 0.0f);
    node.rotate(PI_2);

    EXPECT_NEAR(node.getX(), 0.0f, EPSILON);
    EXPECT_NEAR(node.getY(), 1.0f, EPSILON);
}

TEST(NodeTransform, RotateRotation2D)
{
    murosim::navigation::Node node(1.0f, 0.0f, 0.5f, 0.0f);
    const Eigen::Rotation2Df rotation(PI);
    node.rotate(rotation);

    EXPECT_NEAR(node.getX(), -1.0f, EPSILON);
    EXPECT_NEAR(node.getY(), 0.0f, EPSILON);
}

TEST(NodeTransform, TranslateFloats)
{
    murosim::navigation::Node node(1.0f, 1.0f, 0.5f);
    node.translate(2.0f, 3.0f);

    EXPECT_FLOAT_EQ(node.getX(), 3.0f);
    EXPECT_FLOAT_EQ(node.getY(), 4.0f);
}

TEST(NodeTransform, TranslateVector)
{
    murosim::navigation::Node node(1.0f, 1.0f, 0.5f);
    node.translate(Eigen::Vector2f(5.0f, -2.0f));

    EXPECT_FLOAT_EQ(node.getX(), 6.0f);
    EXPECT_FLOAT_EQ(node.getY(), -1.0f);
}

TEST(NodeTransform, ScaleUniform)
{
    murosim::navigation::Node node(2.0f, 3.0f, 1.0f);
    node.scale(2.0f);

    EXPECT_FLOAT_EQ(node.getX(), 4.0f);
    EXPECT_FLOAT_EQ(node.getY(), 6.0f);
    EXPECT_FLOAT_EQ(node.getRadius(), 2.0f);
}

TEST(NodeTransform, ScaleNonUniform)
{
    murosim::navigation::Node node(2.0f, 3.0f, 1.0f);
    node.scale(2.0f, 3.0f);

    EXPECT_FLOAT_EQ(node.getX(), 4.0f);
    EXPECT_FLOAT_EQ(node.getY(), 9.0f);
}

TEST(NodeTransform, ScaleVector)
{
    murosim::navigation::Node node(2.0f, 3.0f, 1.0f);
    node.scale(Eigen::Vector2f(0.5f, 2.0f));

    EXPECT_FLOAT_EQ(node.getX(), 1.0f);
    EXPECT_FLOAT_EQ(node.getY(), 6.0f);
}

// ============================================================================
// Operator Tests
// ============================================================================

TEST(NodeOperator, LessThanById)
{
    murosim::navigation::Node node1(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1);
    murosim::navigation::Node node2(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2);

    EXPECT_TRUE(node1 < node2);
    EXPECT_FALSE(node2 < node1);
}

TEST(NodeOperator, LessThanEqual)
{
    murosim::navigation::Node node1(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 5);
    murosim::navigation::Node node2(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 5);

    EXPECT_FALSE(node1 < node2);
    EXPECT_FALSE(node2 < node1);
}

TEST(NodeOperator, AffineMultiplication)
{
    murosim::navigation::Node node(1.0f, 0.0f, 0.5f, 0.0f);

    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.rotate(Eigen::Rotation2Df(PI_2));

    const murosim::navigation::Node transformed = affine * node;

    EXPECT_NEAR(transformed.getX(), 0.0f, EPSILON);
    EXPECT_NEAR(transformed.getY(), 1.0f, EPSILON);
}

// ============================================================================
// Goal Radius Tests
// ============================================================================

// Regression: getGoalRadius returned the -1 sentinel for nodes that never
// passed through setGoalRadius, making goals created via
// Robot::setGoal(x, y, radius) unreachable.
TEST(NodeGoalRadius, FallsBackToCircleRadiusWhenUnset)
{
    const murosim::navigation::Node node(1.0f, 2.0f, 0.5f);

    EXPECT_FLOAT_EQ(node.getGoalRadius(), 0.5f);
}

TEST(NodeGoalRadius, SetGoalRadiusNonPositiveFallsBackToCircleRadius)
{
    murosim::navigation::Node node(1.0f, 2.0f, 0.5f);
    node.setGoalRadius(-1.0f);

    EXPECT_FLOAT_EQ(node.getGoalRadius(), 0.5f);
}
