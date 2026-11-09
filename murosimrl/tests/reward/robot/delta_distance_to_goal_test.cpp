// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/delta_distance_to_goal.hpp"
#include "fixtures.hpp"
#include <vector>

namespace
{
using murosimrl::reward::robot::DeltaDistanceToGoal;
using fixtures::EPSILON;
using fixtures::TIME_STEP;

constexpr float POSITIVE = 2.0f;
constexpr float NEGATIVE = -4.0f;
constexpr float SPEED = 0.5f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// The robot starts at the origin with its goal 3 m ahead; every step at
// SPEED changes the distance by SPEED * TIME_STEP.
class DeltaDistanceToGoalTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        robot->setGoal(3.0f, 0.0f, 0.5f); // before the reset, so the first state already carries it
        fixtures::SingleRobotTest::SetUp();
    }

    DeltaDistanceToGoal reward{POSITIVE, NEGATIVE};
};

// ============================================================================
// Reward Tests
// ============================================================================

TEST_F(DeltaDistanceToGoalTest, ShapeAndConfiguration)
{
    EXPECT_EQ(reward.shape(robots()), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "delta_distance_to_goal");
    EXPECT_FLOAT_EQ(reward.positive(), POSITIVE);
    EXPECT_FLOAT_EQ(reward.negative(), NEGATIVE);
}

// Fewer than two recorded states cannot produce a delta: the reward is
// zero, not stale memory.
TEST_F(DeltaDistanceToGoalTest, SingleStateYieldsZero)
{
    ASSERT_EQ(robot->getStateSize(), static_cast<size_t>(1));

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(DeltaDistanceToGoalTest, ApproachingTheGoalScalesByPositive)
{
    robot->setActuatorCommandVelocity(SPEED, 0.0f, 0.0f);
    step();

    EXPECT_NEAR(reward(robot)[0], POSITIVE * SPEED * TIME_STEP, EPSILON);
}

// The negative factor is passed as a negative value, so a regression yields
// a negative reward.
TEST_F(DeltaDistanceToGoalTest, RetreatingFromTheGoalScalesByNegative)
{
    robot->setActuatorCommandVelocity(-SPEED, 0.0f, 0.0f);
    step();

    EXPECT_NEAR(reward(robot)[0], NEGATIVE * SPEED * TIME_STEP, EPSILON);
}

TEST_F(DeltaDistanceToGoalTest, StandingStillYieldsZero)
{
    step();

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(DeltaDistanceToGoalTest, OnlyTheLastStepCounts)
{
    robot->setActuatorCommandVelocity(SPEED, 0.0f, 0.0f);
    step(3);

    EXPECT_NEAR(reward(robot)[0], POSITIVE * SPEED * TIME_STEP, EPSILON);
}
