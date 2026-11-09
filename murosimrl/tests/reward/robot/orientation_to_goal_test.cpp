// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/orientation_to_goal.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <memory>
#include <vector>

namespace
{
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::robot::Robot;
using murosimrl::reward::robot::OrientationToGoal;
using fixtures::EPSILON;

constexpr float POSITIVE = 2.0f;
constexpr float NEGATIVE = -0.5f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// reward = cos(angle to goal) * positive when facing the goal, * negative
// when facing away, from the robot's current pose and goal.
class OrientationToGoalRewardTest : public ::testing::Test
{
protected:
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    OrientationToGoal reward{POSITIVE, NEGATIVE};
};

// ============================================================================
// Reward Tests
// ============================================================================

TEST_F(OrientationToGoalRewardTest, ShapeAndConfiguration)
{
    EXPECT_EQ(reward.shape(fixtures::robotList(robot)), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "orientation_to_goal");
    EXPECT_FLOAT_EQ(reward.positive(), POSITIVE);
    EXPECT_FLOAT_EQ(reward.negative(), NEGATIVE);
}

TEST_F(OrientationToGoalRewardTest, FacingTheGoalEarnsPositive)
{
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(5.0f, 0.0f, 0.5f);

    EXPECT_NEAR(reward(robot)[0], POSITIVE, EPSILON);
}

TEST_F(OrientationToGoalRewardTest, FacingAwayEarnsNegative)
{
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(-5.0f, 0.0f, 0.5f);

    EXPECT_NEAR(reward(robot)[0], NEGATIVE, EPSILON); // the negative scale is applied as given
}

TEST_F(OrientationToGoalRewardTest, ScalesWithTheCosineOfTheAngle)
{
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(1.0f, 1.0f, 0.5f); // 45 degrees off the heading

    EXPECT_NEAR(reward(robot)[0], POSITIVE * std::cos(PI_4), EPSILON);
}

TEST_F(OrientationToGoalRewardTest, GoalToTheSideIsNeutral)
{
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(0.0f, 5.0f, 0.5f);

    EXPECT_NEAR(reward(robot)[0], 0.0f, EPSILON);
}

TEST_F(OrientationToGoalRewardTest, RotatedRobotUsesItsOwnHeading)
{
    robot->setPose(0.0f, 0.0f, PI_2); // facing +y
    robot->setGoal(0.0f, 5.0f, 0.5f);

    EXPECT_NEAR(reward(robot)[0], POSITIVE, EPSILON);
}

TEST_F(OrientationToGoalRewardTest, OnGoalIsNeutral)
{
    robot->setPose(2.0f, 2.0f, 0.0f);
    robot->setGoal(2.0f, 2.0f, 0.5f);

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

// ============================================================================
// Batch Tests
// ============================================================================

TEST_F(OrientationToGoalRewardTest, OneRewardPerRobotInOrder)
{
    const std::shared_ptr<Robot> other = fixtures::holonomicRobot("other");
    robot->setGoal(5.0f, 0.0f, 0.5f);
    other->setGoal(-5.0f, 0.0f, 0.5f);

    const std::vector<float> output = reward(fixtures::RobotList{robot, other});

    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_NEAR(output[0], POSITIVE, EPSILON);
    EXPECT_NEAR(output[1], NEGATIVE, EPSILON);
}
