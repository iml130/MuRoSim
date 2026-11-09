// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/robot/util.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::navigation::Node;
using murosim::robot::util::deltaPose;
using murosim::robot::util::distanceToGoal;
using murosim::robot::util::orientationToGoal;

constexpr float EPSILON = 1e-5f;

Eigen::Affine2f poseAt(const float x, const float y, const float angle)
{
    return murosim::geometry::primitives::pose(x, y, angle);
}
} // namespace

// ============================================================================
// Orientation To Goal Tests
// ============================================================================

TEST(RobotUtilOrientationToGoal, NodeAhead)
{
    const Eigen::Vector2f orientation = orientationToGoal(poseAt(0.0f, 0.0f, 0.0f), Node(5.0f, 0.0f, 1.0f));

    EXPECT_NEAR(orientation.x(), 1.0f, EPSILON);
    EXPECT_NEAR(orientation.y(), 0.0f, EPSILON);
}

TEST(RobotUtilOrientationToGoal, VectorToTheLeft)
{
    const Eigen::Vector2f orientation = orientationToGoal(poseAt(0.0f, 0.0f, 0.0f), Eigen::Vector2f(0.0f, 5.0f));

    EXPECT_NEAR(orientation.x(), 0.0f, EPSILON);
    EXPECT_NEAR(orientation.y(), 1.0f, EPSILON);
}

TEST(RobotUtilOrientationToGoal, DiagonalIsNormalised)
{
    const Eigen::Vector2f orientation = orientationToGoal(poseAt(0.0f, 0.0f, 0.0f), Eigen::Vector2f(1.0f, 1.0f));

    const float expected = 1.0f / std::sqrt(2.0f);
    EXPECT_NEAR(orientation.x(), expected, EPSILON);
    EXPECT_NEAR(orientation.y(), expected, EPSILON);
    EXPECT_NEAR(orientation.norm(), 1.0f, EPSILON);
}

TEST(RobotUtilOrientationToGoal, OffsetPose)
{
    const Eigen::Vector2f orientation = orientationToGoal(poseAt(1.0f, 1.0f, 0.0f), Eigen::Vector2f(4.0f, 5.0f));

    // (3, 4) normalised
    EXPECT_NEAR(orientation.x(), 0.6f, EPSILON);
    EXPECT_NEAR(orientation.y(), 0.8f, EPSILON);
}

TEST(RobotUtilOrientationToGoal, IsExpressedInRobotFrame)
{
    // Robot faces +y, goal straight ahead in the world: dead ahead locally
    const Eigen::Vector2f orientation = orientationToGoal(poseAt(0.0f, 0.0f, PI_2), Eigen::Vector2f(0.0f, 5.0f));

    EXPECT_NEAR(orientation.x(), 1.0f, EPSILON);
    EXPECT_NEAR(orientation.y(), 0.0f, EPSILON);
}

// Regression: normalising the zero offset produced NaN in every consumer.
TEST(RobotUtilOrientationToGoal, OnGoalIsZeroVector)
{
    const Eigen::Vector2f orientation = orientationToGoal(poseAt(5.0f, 5.0f, 0.0f), Eigen::Vector2f(5.0f, 5.0f));

    EXPECT_FLOAT_EQ(orientation.x(), 0.0f);
    EXPECT_FLOAT_EQ(orientation.y(), 0.0f);
}

// ============================================================================
// Distance To Goal Tests
// ============================================================================

TEST(RobotUtilDistanceToGoal, Node)
{
    EXPECT_NEAR(distanceToGoal(poseAt(0.0f, 0.0f, 0.0f), Node(3.0f, 4.0f, 1.0f)), 5.0f, EPSILON);
}

TEST(RobotUtilDistanceToGoal, Vector)
{
    EXPECT_NEAR(distanceToGoal(poseAt(0.0f, 0.0f, 0.0f), Eigen::Vector2f(3.0f, 4.0f)), 5.0f, EPSILON);
}

TEST(RobotUtilDistanceToGoal, OnGoalIsZero)
{
    EXPECT_FLOAT_EQ(distanceToGoal(poseAt(5.0f, 5.0f, 0.0f), Eigen::Vector2f(5.0f, 5.0f)), 0.0f);
}

TEST(RobotUtilDistanceToGoal, IgnoresRotation)
{
    EXPECT_NEAR(distanceToGoal(poseAt(1.0f, 1.0f, PI_2), Eigen::Vector2f(4.0f, 5.0f)), 5.0f, EPSILON);
}

// ============================================================================
// Delta Pose Tests
// ============================================================================

TEST(RobotUtilDeltaPose, Identity)
{
    const Eigen::Vector3f delta = deltaPose(poseAt(0.0f, 0.0f, 0.0f), poseAt(0.0f, 0.0f, 0.0f));

    EXPECT_FLOAT_EQ(delta.x(), 0.0f);
    EXPECT_FLOAT_EQ(delta.y(), 0.0f);
    EXPECT_FLOAT_EQ(delta.z(), 0.0f);
}

TEST(RobotUtilDeltaPose, TranslationOnly)
{
    const Eigen::Vector3f delta = deltaPose(poseAt(1.0f, 2.0f, 0.0f), poseAt(0.0f, 0.0f, 0.0f));

    EXPECT_NEAR(delta.x(), 1.0f, EPSILON);
    EXPECT_NEAR(delta.y(), 2.0f, EPSILON);
    EXPECT_NEAR(delta.z(), 0.0f, EPSILON);
}

TEST(RobotUtilDeltaPose, RotationOnly)
{
    const Eigen::Vector3f delta = deltaPose(poseAt(0.0f, 0.0f, PI_2), poseAt(0.0f, 0.0f, 0.0f));

    EXPECT_NEAR(delta.x(), 0.0f, EPSILON);
    EXPECT_NEAR(delta.y(), 0.0f, EPSILON);
    EXPECT_NEAR(delta.z(), PI_2, EPSILON);
}

TEST(RobotUtilDeltaPose, TranslationIsInPreviousFrame)
{
    // Previous pose faces +y; moving +1 in world y is +1 along its local x
    const Eigen::Vector3f delta = deltaPose(poseAt(0.0f, 1.0f, PI_2), poseAt(0.0f, 0.0f, PI_2));

    EXPECT_NEAR(delta.x(), 1.0f, EPSILON);
    EXPECT_NEAR(delta.y(), 0.0f, EPSILON);
    EXPECT_NEAR(delta.z(), 0.0f, EPSILON);
}

TEST(RobotUtilDeltaPose, Combined)
{
    const Eigen::Vector3f delta = deltaPose(poseAt(3.0f, 4.0f, 1.0f), poseAt(1.0f, 1.0f, 0.5f));

    // World offset (2, 3) rotated back by the previous heading of 0.5 rad
    const float cos = std::cos(0.5f);
    const float sin = std::sin(0.5f);
    EXPECT_NEAR(delta.x(), 2.0f * cos + 3.0f * sin, EPSILON);
    EXPECT_NEAR(delta.y(), -2.0f * sin + 3.0f * cos, EPSILON);
    EXPECT_NEAR(delta.z(), 0.5f, EPSILON);
}

TEST(RobotUtilDeltaPose, ReversedArgumentsInvert)
{
    const Eigen::Affine2f a = poseAt(3.0f, 4.0f, 1.0f);
    const Eigen::Affine2f b = poseAt(1.0f, 1.0f, 0.5f);

    const Eigen::Vector3f forward = deltaPose(a, b);
    const Eigen::Vector3f backward = deltaPose(b, a);

    // Applying the forward delta then the backward one returns to the start
    const Eigen::Affine2f round_trip = poseAt(forward.x(), forward.y(), forward.z()) *
                                       poseAt(backward.x(), backward.y(), backward.z());
    EXPECT_TRUE(round_trip.isApprox(Eigen::Affine2f::Identity(), EPSILON));
}
