// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/geometry/line.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
constexpr float EPSILON = 1e-5f;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(LineConstructor, DefaultConstructor)
{
    murosim::geometry::Line line;

    EXPECT_FLOAT_EQ(line.getPoint1X(), 0.0f);
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 0.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 0.0f);
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 0.0f);
    EXPECT_FLOAT_EQ(line.getLength(), 0.0f);
}

TEST(LineConstructor, ParameterizedConstructor)
{
    murosim::geometry::Line line(1.0f, 2.0f, 3.0f, 4.0f);

    EXPECT_FLOAT_EQ(line.getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 3.0f);
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 4.0f);

    // Length should be calculated correctly
    EXPECT_NEAR(line.getLength(), std::hypot(2.0f, 2.0f), EPSILON);
}

TEST(LineConstructor, ZeroLengthLine)
{
    murosim::geometry::Line line(1.0f, 2.0f, 1.0f, 2.0f);

    EXPECT_FLOAT_EQ(line.getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 1.0f);
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getLength(), 0.0f);
}

// ============================================================================
// Test Fixture
// ============================================================================

class LineTest : public ::testing::Test
{
protected:
    murosim::geometry::Line line{1.0f, 2.0f, 3.0f, 4.0f};
    const float expected_length = std::hypot(2.0f, 2.0f); // from (1, 2) to (3, 4)
};

// ============================================================================
// Getter Tests
// ============================================================================

TEST_F(LineTest, GetPointComponents)
{
    EXPECT_FLOAT_EQ(line.getPoint1X(), 1.0f);
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 3.0f);
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 4.0f);
}

TEST_F(LineTest, GetPointVectors)
{
    EXPECT_FLOAT_EQ(line.getPoint1().x(), 1.0f);
    EXPECT_FLOAT_EQ(line.getPoint1().y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getPoint2().x(), 3.0f);
    EXPECT_FLOAT_EQ(line.getPoint2().y(), 4.0f);
}

TEST_F(LineTest, GetLength)
{
    EXPECT_NEAR(line.getLength(), expected_length, EPSILON);
}

// ============================================================================
// Setter Tests
// ============================================================================

TEST_F(LineTest, SetPoint1Components)
{
    line.setPoint1X(5.0f);
    EXPECT_FLOAT_EQ(line.getPoint1X(), 5.0f);
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 2.0f);  // Y unchanged
    EXPECT_FLOAT_EQ(line.getPoint2X(), 3.0f);  // Point2 unchanged
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 4.0f);

    line.setPoint1Y(6.0f);
    EXPECT_FLOAT_EQ(line.getPoint1X(), 5.0f);  // X unchanged
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 6.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 3.0f);  // Point2 unchanged
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 4.0f);
}

TEST_F(LineTest, SetPoint2Components)
{
    line.setPoint2X(7.0f);
    EXPECT_FLOAT_EQ(line.getPoint1X(), 1.0f);  // Point1 unchanged
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 7.0f);
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 4.0f);  // Y unchanged

    line.setPoint2Y(8.0f);
    EXPECT_FLOAT_EQ(line.getPoint1X(), 1.0f);  // Point1 unchanged
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 7.0f);  // X unchanged
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 8.0f);
}

TEST_F(LineTest, SetPoint1WithVector)
{
    line.setPoint1(Eigen::Vector2f(9.0f, 10.0f));
    EXPECT_FLOAT_EQ(line.getPoint1X(), 9.0f);
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 10.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 3.0f);  // Point2 unchanged
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 4.0f);
}

TEST_F(LineTest, SetPoint2WithVector)
{
    line.setPoint2(Eigen::Vector2f(11.0f, 12.0f));
    EXPECT_FLOAT_EQ(line.getPoint1X(), 1.0f);  // Point1 unchanged
    EXPECT_FLOAT_EQ(line.getPoint1Y(), 2.0f);
    EXPECT_FLOAT_EQ(line.getPoint2X(), 11.0f);
    EXPECT_FLOAT_EQ(line.getPoint2Y(), 12.0f);
}

TEST_F(LineTest, SetPoint1WithComponents)
{
    line.setPoint1(13.0f, 14.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(13.0f, 14.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(3.0f, 4.0f)));
}

TEST_F(LineTest, SetPoint2WithComponents)
{
    line.setPoint2(15.0f, 16.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(1.0f, 2.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(15.0f, 16.0f)));
}

// ============================================================================
// Translation Tests
// ============================================================================

TEST_F(LineTest, TranslateWithComponents)
{
    line.translate(1.0f, 2.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(2.0f, 4.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(4.0f, 6.0f)));
    // Length should remain the same
    EXPECT_NEAR(line.getLength(), expected_length, EPSILON);
}

TEST_F(LineTest, TranslateWithVector)
{
    line.translate(Eigen::Vector2f(-1.0f, -2.0f));
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(0.0f, 0.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(2.0f, 2.0f)));
    // Length should remain the same
    EXPECT_NEAR(line.getLength(), expected_length, EPSILON);
}

TEST_F(LineTest, TranslateMultipleTimes)
{
    line.translate(1.0f, 2.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(2.0f, 4.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(4.0f, 6.0f)));

    line.translate(Eigen::Vector2f(-1.0f, -2.0f));
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(1.0f, 2.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(3.0f, 4.0f)));
}

// ============================================================================
// Rotation Tests
// ============================================================================

TEST_F(LineTest, RotateWithFloat)
{
    line.rotate(PI);
    EXPECT_NEAR(line.getPoint1().x(), -1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), -2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), -3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), -4.0f, EPSILON);
    // Length should remain the same
    EXPECT_NEAR(line.getLength(), expected_length, EPSILON);
}

TEST_F(LineTest, RotateWithRotation2D)
{
    line.rotate(Eigen::Rotation2Df(PI));
    EXPECT_NEAR(line.getPoint1().x(), -1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), -2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), -3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), -4.0f, EPSILON);
    // Length should remain the same
    EXPECT_NEAR(line.getLength(), expected_length, EPSILON);
}

TEST_F(LineTest, RotateFullCircle)
{
    line.rotate(PI);
    EXPECT_NEAR(line.getPoint1().x(), -1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), -2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), -3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), -4.0f, EPSILON);

    line.rotate(Eigen::Rotation2Df(PI));
    EXPECT_NEAR(line.getPoint1().x(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 4.0f, EPSILON);
}

// ============================================================================
// Transform Tests
// ============================================================================

TEST_F(LineTest, TransformRotationOnly)
{
    line.transform(Eigen::Rotation2Df(PI) * Eigen::Translation2f(0.0f, 0.0f));
    EXPECT_NEAR(line.getPoint1().x(), -1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), -2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), -3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), -4.0f, EPSILON);

    line.transform(Eigen::Rotation2Df(-PI) * Eigen::Translation2f(0.0f, 0.0f));
    EXPECT_NEAR(line.getPoint1().x(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 4.0f, EPSILON);
}

TEST_F(LineTest, TransformTranslationOnly)
{
    line.transform(Eigen::Rotation2Df(0.0f) * Eigen::Translation2f(1.0f, 1.0f));
    EXPECT_NEAR(line.getPoint1().x(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 4.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 5.0f, EPSILON);

    line.transform(Eigen::Rotation2Df(0.0f) * Eigen::Translation2f(-1.0f, -1.0f));
    EXPECT_NEAR(line.getPoint1().x(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 4.0f, EPSILON);
}

TEST_F(LineTest, TransformCombinedTranslationAndRotation)
{
    line.transform(Eigen::Rotation2Df(PI_2) * Eigen::Translation2f(1.0f, 1.0f));
    EXPECT_NEAR(line.getPoint1().x(), -3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), -5.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 4.0f, EPSILON);

    line.transform(Eigen::Rotation2Df(-PI_2) * Eigen::Translation2f(1.0f, -1.0f));
    EXPECT_NEAR(line.getPoint1().x(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 4.0f, EPSILON);
}

TEST_F(LineTest, TransformPoseRotatesThenTranslates)
{
    // pose (5, 0, 90 deg): rotate about the origin, THEN translate by +5 in x
    // (1, 2) -> (-2, 1) -> (3, 1) ; (3, 4) -> (-4, 3) -> (1, 3)
    line.transform(5.0f, 0.0f, PI_2);
    EXPECT_NEAR(line.getPoint1X(), 3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1Y(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2X(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2Y(), 3.0f, EPSILON);
}

TEST_F(LineTest, TransformIdentity)
{
    line.transform(Eigen::Affine2f::Identity());
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(1.0f, 2.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(3.0f, 4.0f)));
}

TEST_F(LineTest, TransformWithScaling)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.scale(Eigen::Vector2f(2.0f, -0.5f));
    line.transform(affine);
    EXPECT_NEAR(line.getPoint1().x(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), -1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 6.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), -2.0f, EPSILON);
}

TEST_F(LineTest, TransformInverseRestoresOriginal)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.scale(Eigen::Vector2f(2.0f, -0.5f));
    line.transform(affine);
    EXPECT_NEAR(line.getPoint1().x(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), -1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 6.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), -2.0f, EPSILON);

    line.transform(affine.inverse());
    EXPECT_NEAR(line.getPoint1().x(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 2.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 3.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 4.0f, EPSILON);
}

TEST_F(LineTest, TransformMultiplicationOperator)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();

    line = affine * line;
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(1.0f, 2.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(3.0f, 4.0f)));

    affine.translate(Eigen::Vector2f(-1.0f, -1.0f));
    affine.scale(Eigen::Vector2f(2.0f, 1.0f));

    line = affine * line;
    EXPECT_NEAR(line.getPoint1().x(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint1().y(), 1.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().x(), 5.0f, EPSILON);
    EXPECT_NEAR(line.getPoint2().y(), 3.0f, EPSILON);
}

// ============================================================================
// Scale Tests
// ============================================================================

TEST_F(LineTest, ScaleWithComponents)
{
    line.scale(2.0f, 3.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(2.0f, 6.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(6.0f, 12.0f)));
}

TEST_F(LineTest, ScaleWithVector)
{
    line.scale(Eigen::Vector2f(0.5f, -1.0f));
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(0.5f, -2.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(1.5f, -4.0f)));
}

TEST_F(LineTest, ScaleMultipleTimes)
{
    line.scale(2.0f, 3.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(2.0f, 6.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(6.0f, 12.0f)));

    line.scale(Eigen::Vector2f(0.5f, -1.0f));
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(1.0f, -6.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(3.0f, -12.0f)));
}

TEST_F(LineTest, ScaleUniform)
{
    line.scale(2.0f, 2.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(2.0f, 4.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(6.0f, 8.0f)));
    // Length should be scaled uniformly
    EXPECT_NEAR(line.getLength(), expected_length * 2.0f, EPSILON);
}

TEST_F(LineTest, ScaleNegative)
{
    line.scale(-1.0f, -1.0f);
    EXPECT_TRUE(line.getPoint1().isApprox(Eigen::Vector2f(-1.0f, -2.0f)));
    EXPECT_TRUE(line.getPoint2().isApprox(Eigen::Vector2f(-3.0f, -4.0f)));
    // Length should remain the same
    EXPECT_NEAR(line.getLength(), expected_length, EPSILON);
}

TEST_F(LineTest, ScaleZeroCreatesZeroLengthLine)
{
    line.scale(0.0f, 0.0f);
    EXPECT_FLOAT_EQ(line.getPoint1().x(), 0.0f);
    EXPECT_FLOAT_EQ(line.getPoint1().y(), 0.0f);
    EXPECT_FLOAT_EQ(line.getPoint2().x(), 0.0f);
    EXPECT_FLOAT_EQ(line.getPoint2().y(), 0.0f);
    EXPECT_FLOAT_EQ(line.getLength(), 0.0f);
}
