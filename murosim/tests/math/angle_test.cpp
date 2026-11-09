// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/math/angle.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>

namespace
{
using murosim::math::angleBetweenVectorAtan2;
using murosim::math::deltaAngle;
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::math::rotationAngle;
using murosim::math::rotationVector;

constexpr float EPSILON = 1e-5f;

Eigen::Affine2f poseAt(const float x, const float y, const float angle)
{
    return murosim::geometry::primitives::pose(x, y, angle);
}
} // namespace

// ============================================================================
// Rotation Tests
// ============================================================================

TEST(MathAngleRotation, AngleZero)
{
    EXPECT_FLOAT_EQ(rotationAngle(poseAt(0.0f, 0.0f, 0.0f)), 0.0f);
}

TEST(MathAngleRotation, AnglePiHalf)
{
    EXPECT_NEAR(rotationAngle(poseAt(0.0f, 0.0f, PI_2)), PI_2, EPSILON);
}

TEST(MathAngleRotation, AnglePi)
{
    // +pi and -pi are the same rotation; the sign depends on rounding
    EXPECT_NEAR(std::abs(rotationAngle(poseAt(0.0f, 0.0f, PI))), PI, EPSILON);
}

TEST(MathAngleRotation, AngleIgnoresTranslation)
{
    EXPECT_NEAR(rotationAngle(poseAt(3.0f, -7.0f, PI_4)), PI_4, EPSILON);
}

TEST(MathAngleRotation, VectorZero)
{
    const Eigen::Vector2f heading = rotationVector(poseAt(0.0f, 0.0f, 0.0f));

    EXPECT_NEAR(heading.x(), 1.0f, EPSILON);
    EXPECT_NEAR(heading.y(), 0.0f, EPSILON);
}

TEST(MathAngleRotation, VectorPiHalf)
{
    const Eigen::Vector2f heading = rotationVector(poseAt(0.0f, 0.0f, PI_2));

    EXPECT_NEAR(heading.x(), 0.0f, EPSILON);
    EXPECT_NEAR(heading.y(), 1.0f, EPSILON);
}

// ============================================================================
// Angle Between Vector Tests
// ============================================================================

TEST(MathAngleBetweenVector, Atan2SameDirection)
{
    EXPECT_FLOAT_EQ(angleBetweenVectorAtan2(Eigen::Vector2f(1.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f)), 0.0f);
}

TEST(MathAngleBetweenVector, Atan2Perpendicular)
{
    EXPECT_NEAR(angleBetweenVectorAtan2(Eigen::Vector2f(1.0f, 0.0f), Eigen::Vector2f(0.0f, 1.0f)), PI_2,
                EPSILON);
}

TEST(MathAngleBetweenVector, Atan2IsSigned)
{
    EXPECT_NEAR(angleBetweenVectorAtan2(Eigen::Vector2f(1.0f, 0.0f), Eigen::Vector2f(0.0f, -1.0f)), -PI_2,
                EPSILON);
}

TEST(MathAngleBetweenVector, Atan2Opposite)
{
    EXPECT_NEAR(std::abs(angleBetweenVectorAtan2(Eigen::Vector2f(1.0f, 0.0f), Eigen::Vector2f(-1.0f, 0.0f))), PI,
                EPSILON);
}

TEST(MathAngleBetweenVector, Atan2IgnoresMagnitude)
{
    EXPECT_NEAR(angleBetweenVectorAtan2(Eigen::Vector2f(2.0f, 0.0f), Eigen::Vector2f(3.0f, 3.0f)), PI_4, EPSILON);
}

// ============================================================================
// Delta Angle Tests
// ============================================================================

TEST(MathAngleDelta, Zero)
{
    EXPECT_FLOAT_EQ(deltaAngle(0.0f, 0.0f), 0.0f);
}

TEST(MathAngleDelta, Positive)
{
    EXPECT_NEAR(deltaAngle(1.0f, 0.5f), 0.5f, EPSILON);
}

TEST(MathAngleDelta, Negative)
{
    EXPECT_NEAR(deltaAngle(0.5f, 1.0f), -0.5f, EPSILON);
}

TEST(MathAngleDelta, WrapsAcrossPi)
{
    // From just below +pi to just above -pi is a short step forward, not a
    // full turn backwards
    EXPECT_NEAR(deltaAngle(-PI + 0.1f, PI - 0.1f), 0.2f, EPSILON);
    EXPECT_NEAR(deltaAngle(PI - 0.1f, -PI + 0.1f), -0.2f, EPSILON);
}

TEST(MathAngleDelta, HalfTurnIsPi)
{
    EXPECT_NEAR(std::abs(deltaAngle(PI, 0.0f)), PI, EPSILON);
}
