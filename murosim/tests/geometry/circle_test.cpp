// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/geometry/circle.hpp"
#include "murosim/math/constants.hpp"

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
constexpr float EPSILON = 1e-5f;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(CircleConstructor, DefaultConstructor)
{
    murosim::geometry::Circle circle;

    EXPECT_FLOAT_EQ(circle.getX(), 0.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 0.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 0.0f);
}

TEST(CircleConstructor, ParameterizedConstructor)
{
    murosim::geometry::Circle circle(1.0f, 2.0f, 3.0f);

    EXPECT_FLOAT_EQ(circle.getX(), 1.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

TEST(CircleConstructor, NegativeRadiusClamped)
{
    murosim::geometry::Circle circle(1.0f, 2.0f, -5.0f);

    EXPECT_FLOAT_EQ(circle.getX(), 1.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 0.0f);  // Negative radius clamped to 0
}

TEST(CircleConstructor, ZeroRadius)
{
    murosim::geometry::Circle circle(1.0f, 2.0f, 0.0f);

    EXPECT_FLOAT_EQ(circle.getX(), 1.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 0.0f);
}

// ============================================================================
// Test Fixture
// ============================================================================

class CircleTest : public ::testing::Test
{
protected:
    murosim::geometry::Circle circle{1.0f, 2.0f, 3.0f};
};

// ============================================================================
// Getter Tests
// ============================================================================

TEST_F(CircleTest, GetPositionComponents)
{
    EXPECT_FLOAT_EQ(circle.getX(), 1.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);
}

TEST_F(CircleTest, GetRadius)
{
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

TEST_F(CircleTest, GetPointAsVector)
{
    EXPECT_TRUE(circle.getPoint().isApprox(Eigen::Vector2f(1.0f, 2.0f)));
}

// ============================================================================
// Setter Tests
// ============================================================================

TEST_F(CircleTest, SetPositionComponents)
{
    circle.setX(4.0f);
    EXPECT_FLOAT_EQ(circle.getX(), 4.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);  // Y unchanged
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged

    circle.setY(5.0f);
    EXPECT_FLOAT_EQ(circle.getX(), 4.0f);  // X unchanged
    EXPECT_FLOAT_EQ(circle.getY(), 5.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged
}

TEST_F(CircleTest, SetRadius)
{
    circle.setRadius(6.0f);
    EXPECT_FLOAT_EQ(circle.getX(), 1.0f);  // Position unchanged
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 6.0f);
}

TEST_F(CircleTest, SetNegativeRadiusClamped)
{
    circle.setRadius(-1.0f);
    EXPECT_FLOAT_EQ(circle.getX(), 1.0f);  // Position unchanged
    EXPECT_FLOAT_EQ(circle.getY(), 2.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 0.0f);  // Clamped to 0
}

TEST_F(CircleTest, SetPointWithVector)
{
    circle.setPoint(Eigen::Vector2f(7.0f, 8.0f));
    EXPECT_FLOAT_EQ(circle.getX(), 7.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 8.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged
}

TEST_F(CircleTest, SetPointWithComponents)
{
    circle.setPoint(9.0f, 10.0f);
    EXPECT_FLOAT_EQ(circle.getX(), 9.0f);
    EXPECT_FLOAT_EQ(circle.getY(), 10.0f);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged
}

// ============================================================================
// Translation Tests
// ============================================================================

TEST_F(CircleTest, TranslateWithComponents)
{
    circle.translate(1.0f, 2.0f);
    EXPECT_TRUE(circle.getPoint().isApprox(Eigen::Vector2f(2.0f, 4.0f)));
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged
}

TEST_F(CircleTest, TranslateWithVector)
{
    circle.translate(Eigen::Vector2f(-1.0f, -2.0f));
    EXPECT_TRUE(circle.getPoint().isApprox(Eigen::Vector2f(0.0f, 0.0f)));
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged
}

TEST_F(CircleTest, TranslateMultipleTimes)
{
    circle.translate(1.0f, 2.0f);
    EXPECT_TRUE(circle.getPoint().isApprox(Eigen::Vector2f(2.0f, 4.0f)));

    circle.translate(Eigen::Vector2f(-1.0f, -2.0f));
    EXPECT_TRUE(circle.getPoint().isApprox(Eigen::Vector2f(1.0f, 2.0f)));
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

// ============================================================================
// Rotation Tests
// ============================================================================

TEST_F(CircleTest, RotateWithFloat)
{
    circle.rotate(PI);
    EXPECT_NEAR(circle.getPoint().x(), -1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged
}

TEST_F(CircleTest, RotateWithRotation2D)
{
    circle.rotate(Eigen::Rotation2Df(PI));
    EXPECT_NEAR(circle.getPoint().x(), -1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);  // Radius unchanged
}

TEST_F(CircleTest, RotateFullCircle)
{
    circle.rotate(PI);
    EXPECT_NEAR(circle.getPoint().x(), -1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -2.0f, EPSILON);

    circle.rotate(Eigen::Rotation2Df(PI));
    EXPECT_NEAR(circle.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

// ============================================================================
// Transform Tests
// ============================================================================

TEST_F(CircleTest, TransformRotationOnly)
{
    circle.transform(Eigen::Rotation2Df(PI) * Eigen::Translation2f(0.0f, 0.0f));
    EXPECT_NEAR(circle.getPoint().x(), -1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);

    circle.transform(Eigen::Rotation2Df(-PI) * Eigen::Translation2f(0.0f, 0.0f));
    EXPECT_NEAR(circle.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

TEST_F(CircleTest, TransformTranslationOnly)
{
    circle.transform(Eigen::Rotation2Df(0.0f) * Eigen::Translation2f(1.0f, 1.0f));
    EXPECT_NEAR(circle.getPoint().x(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 3.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);

    circle.transform(Eigen::Rotation2Df(0.0f) * Eigen::Translation2f(-1.0f, -1.0f));
    EXPECT_NEAR(circle.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

TEST_F(CircleTest, TransformCombinedTranslationAndRotation)
{
    circle.transform(Eigen::Rotation2Df(PI_2) * Eigen::Translation2f(1.0f, 1.0f));
    EXPECT_NEAR(circle.getPoint().x(), -3.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);

    circle.transform(Eigen::Rotation2Df(-PI_2) * Eigen::Translation2f(1.0f, -1.0f));
    EXPECT_NEAR(circle.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

TEST_F(CircleTest, TransformPoseRotatesThenTranslates)
{
    // pose (5, 0, 90 deg): rotate (1, 2) -> (-2, 1) about the origin, THEN translate by +5 in x
    circle.transform(5.0f, 0.0f, PI_2);
    EXPECT_NEAR(circle.getX(), 3.0f, EPSILON);
    EXPECT_NEAR(circle.getY(), 1.0f, EPSILON);
    EXPECT_FLOAT_EQ(circle.getRadius(), 3.0f);
}

TEST_F(CircleTest, TransformWithScaling)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.scale(Eigen::Vector2f(2.0f, -0.5f));

    circle.transform(affine);
    EXPECT_NEAR(circle.getPoint().x(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -1.0f, EPSILON);
    // Radius scaled by minimum of the (absolute) scale factors
    EXPECT_NEAR(circle.getRadius(), 1.5f, EPSILON);
}

TEST_F(CircleTest, TransformInverseRestoresOriginal)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.scale(Eigen::Vector2f(2.0f, -0.5f));

    circle.transform(affine);
    EXPECT_NEAR(circle.getPoint().x(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -1.0f, EPSILON);
    EXPECT_NEAR(circle.getRadius(), 1.5f, EPSILON);

    circle.transform(affine.inverse());
    EXPECT_NEAR(circle.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getRadius(), 0.75f, EPSILON);
}

TEST_F(CircleTest, TransformMultiplicationOperator)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();

    circle = affine * circle;
    EXPECT_NEAR(circle.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getRadius(), 3.0f, EPSILON);

    affine.translate(Eigen::Vector2f(1.0f, 1.0f));
    affine.rotate(PI);
    affine.scale(Eigen::Vector2f(4.0f, 2.0f));

    circle = affine * circle;
    EXPECT_NEAR(circle.getPoint().x(), -3.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -3.0f, EPSILON);
    EXPECT_NEAR(circle.getRadius(), 6.0f, EPSILON);
}

// ============================================================================
// Scale Tests
// ============================================================================

TEST_F(CircleTest, ScaleWithComponents)
{
    circle.scale(2.0f, 0.5f);
    EXPECT_NEAR(circle.getPoint().x(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 1.0f, EPSILON);
    // Radius scaled by minimum of the (absolute) scale factors
    EXPECT_NEAR(circle.getRadius(), 1.5f, EPSILON);
}

TEST_F(CircleTest, ScaleWithVector)
{
    circle.scale(Eigen::Vector2f(0.5f, 2.0f));
    EXPECT_NEAR(circle.getPoint().x(), 0.5f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 4.0f, EPSILON);
    // Radius scaled by minimum of the (absolute) scale factors
    EXPECT_NEAR(circle.getRadius(), 1.5f, EPSILON);
}

TEST_F(CircleTest, ScaleMultipleTimes)
{
    circle.scale(2.0f, 0.5f);
    EXPECT_NEAR(circle.getPoint().x(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getRadius(), 1.5f, EPSILON);

    circle.scale(Eigen::Vector2f(0.5f, 2.0f));
    EXPECT_NEAR(circle.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getRadius(), 0.75f, EPSILON);
}

TEST_F(CircleTest, ScaleUniform)
{
    circle.scale(2.0f, 2.0f);
    EXPECT_NEAR(circle.getPoint().x(), 2.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), 4.0f, EPSILON);
    EXPECT_NEAR(circle.getRadius(), 6.0f, EPSILON);
}

TEST_F(CircleTest, ScaleNegative)
{
    circle.scale(-1.0f, -1.0f);
    EXPECT_NEAR(circle.getPoint().x(), -1.0f, EPSILON);
    EXPECT_NEAR(circle.getPoint().y(), -2.0f, EPSILON);
    // Radius uses absolute value of scale
    EXPECT_NEAR(circle.getRadius(), 3.0f, EPSILON);
}
