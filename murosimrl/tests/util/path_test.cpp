// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/util/path.hpp"
#include <vector>

namespace
{
using murosimrl::util::path::interpolateAlongPath;
} // namespace

// ============================================================================
// Interpolate Along Path Tests
// ============================================================================

TEST(UtilPathInterpolate, EmptyPathReturnsFalse)
{
    Eigen::Vector2f result(99.0f, 99.0f);

    EXPECT_FALSE(interpolateAlongPath(Eigen::Vector2f(0.0f, 0.0f), {}, 1.0f, result));
}

TEST(UtilPathInterpolate, PointMidwayAlongASegment)
{
    const std::vector<Eigen::Vector2f> path = {{1.0f, 0.0f}, {3.0f, 0.0f}};
    Eigen::Vector2f result;

    ASSERT_TRUE(interpolateAlongPath(Eigen::Vector2f(0.0f, 0.0f), path, 2.0f, result));

    EXPECT_FLOAT_EQ(result.x(), 2.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.0f);
}

TEST(UtilPathInterpolate, WalksAroundACorner)
{
    // 1 m to the first point, then 1 m up: 1.5 m ends halfway up
    const std::vector<Eigen::Vector2f> path = {{1.0f, 0.0f}, {1.0f, 1.0f}};
    Eigen::Vector2f result;

    ASSERT_TRUE(interpolateAlongPath(Eigen::Vector2f(0.0f, 0.0f), path, 1.5f, result));

    EXPECT_FLOAT_EQ(result.x(), 1.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.5f);
}

TEST(UtilPathInterpolate, ZeroTargetIsTheStart)
{
    const std::vector<Eigen::Vector2f> path = {{1.0f, 0.0f}};
    Eigen::Vector2f result;

    ASSERT_TRUE(interpolateAlongPath(Eigen::Vector2f(0.0f, 0.0f), path, 0.0f, result));

    EXPECT_FLOAT_EQ(result.x(), 0.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.0f);
}

// Regression: the per-consumer copies of this walk diverged exactly here.
// A target beyond the path end clamps to the goal and reports success, so
// the subgoal observation and the along-path reward keep tracking the same
// point near the episode end.
TEST(UtilPathInterpolate, ClampsToGoalWhenPathIsShorterThanTarget)
{
    const std::vector<Eigen::Vector2f> path = {{1.0f, 0.0f}, {2.0f, 0.0f}};
    Eigen::Vector2f result;

    ASSERT_TRUE(interpolateAlongPath(Eigen::Vector2f(0.0f, 0.0f), path, 5.0f, result));

    EXPECT_FLOAT_EQ(result.x(), 2.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.0f);
}

TEST(UtilPathInterpolate, DegenerateSegmentDoesNotDivideByZero)
{
    // The first path point equals the robot position: a zero-length segment
    const std::vector<Eigen::Vector2f> path = {{1.0f, 1.0f}, {1.0f, 1.0f}, {4.0f, 1.0f}};
    Eigen::Vector2f result;

    ASSERT_TRUE(interpolateAlongPath(Eigen::Vector2f(1.0f, 1.0f), path, 2.0f, result));

    EXPECT_TRUE(result.allFinite());
    EXPECT_FLOAT_EQ(result.x(), 3.0f);
    EXPECT_FLOAT_EQ(result.y(), 1.0f);
}
