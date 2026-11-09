// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/delta_distance_along_path.hpp"
#include "fixtures.hpp"
#include <vector>

namespace
{
using murosimrl::reward::robot::DeltaDistanceAlongPath;
using fixtures::EPSILON;
using fixtures::TIME_STEP;

constexpr float POSITIVE = 2.0f;
constexpr float NEGATIVE = -4.0f;
constexpr float SPEED = 0.5f;
constexpr float LOOKAHEAD = 1.0f;
constexpr float LONG_LOOKAHEAD = 10.0f; // beyond the path end: the lookahead point is the goal

/// A short straight path along +x, ending 1.5 m from the origin.
std::vector<Eigen::Vector2f> shortPath()
{
    return {Eigen::Vector2f(1.0f, 0.0f), Eigen::Vector2f(1.5f, 0.0f)};
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class DeltaDistanceAlongPathTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPathPoints(shortPath());
    }
};

// ============================================================================
// Reward Tests
// ============================================================================

TEST_F(DeltaDistanceAlongPathTest, ShapeAndConfiguration)
{
    const DeltaDistanceAlongPath reward(POSITIVE, NEGATIVE, LOOKAHEAD);

    EXPECT_EQ(reward.shape(robots()), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "delta_distance_along_path");
    EXPECT_FLOAT_EQ(reward.positive(), POSITIVE);
    EXPECT_FLOAT_EQ(reward.negative(), NEGATIVE);
    EXPECT_FLOAT_EQ(reward.lookaheadDistance(), LOOKAHEAD);
    EXPECT_FLOAT_EQ(DeltaDistanceAlongPath(POSITIVE, NEGATIVE, 0.0f).lookaheadDistance(), 0.1f); // clamped
}

TEST_F(DeltaDistanceAlongPathTest, SingleStateYieldsZero)
{
    ASSERT_EQ(robot->getStateSize(), static_cast<size_t>(1));
    const DeltaDistanceAlongPath reward(POSITIVE, NEGATIVE, LOOKAHEAD);

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(DeltaDistanceAlongPathTest, WithoutAPathYieldsZero)
{
    robot->setPathPoints({});
    robot->setActuatorCommandVelocity(SPEED, 0.0f, 0.0f);
    step();
    const DeltaDistanceAlongPath reward(POSITIVE, NEGATIVE, LOOKAHEAD);

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

// With the lookahead point clamped to the goal, driving towards it shortens
// the distance by the travelled length.
TEST_F(DeltaDistanceAlongPathTest, ProgressTowardsTheGoalScalesByPositive)
{
    robot->setActuatorCommandVelocity(SPEED, 0.0f, 0.0f);
    step();
    const DeltaDistanceAlongPath reward(POSITIVE, NEGATIVE, LONG_LOOKAHEAD);

    EXPECT_NEAR(reward(robot)[0], POSITIVE * SPEED * TIME_STEP, EPSILON);
}

TEST_F(DeltaDistanceAlongPathTest, RegressionScalesByNegative)
{
    robot->setActuatorCommandVelocity(-SPEED, 0.0f, 0.0f);
    step();
    const DeltaDistanceAlongPath reward(POSITIVE, NEGATIVE, LONG_LOOKAHEAD);

    EXPECT_NEAR(reward(robot)[0], NEGATIVE * SPEED * TIME_STEP, EPSILON);
}

// A lookahead point inside the path moves along with the robot, so its
// distance stays constant while the robot follows the path.
TEST_F(DeltaDistanceAlongPathTest, FollowingThePathKeepsTheLookaheadDistance)
{
    robot->setActuatorCommandVelocity(SPEED, 0.0f, 0.0f);
    step();
    const DeltaDistanceAlongPath reward(POSITIVE, NEGATIVE, LOOKAHEAD);

    EXPECT_NEAR(reward(robot)[0], 0.0f, EPSILON);
}

TEST_F(DeltaDistanceAlongPathTest, StandingStillYieldsZero)
{
    step();
    const DeltaDistanceAlongPath reward(POSITIVE, NEGATIVE, LONG_LOOKAHEAD);

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}
