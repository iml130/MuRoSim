// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/on_goal.hpp"
#include "fixtures.hpp"
#include <memory>
#include <vector>

namespace
{
using murosim::navigation::Node;
using murosim::robot::Robot;
using murosimrl::reward::robot::OnGoal;
using fixtures::EPSILON;

constexpr float POSITIVE = 1.0f;
constexpr float THRESHOLD_RADIUS = 2.0f;
constexpr float USE_GOAL_RADIUS = 0.0f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// reward = max(0, radius - distance) / radius * positive, from the robot's
// current pose and goal, so no simulation step is needed.
class OnGoalTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        robot->setGoal(0.0f, 0.0f, 0.5f);
    }

    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
};

// ============================================================================
// Threshold Radius Tests
// ============================================================================

TEST_F(OnGoalTest, ShapeAndConfiguration)
{
    const OnGoal reward(THRESHOLD_RADIUS, POSITIVE);

    EXPECT_EQ(reward.shape(fixtures::robotList(robot)), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "on_goal");
    EXPECT_FLOAT_EQ(reward.thresholdRadius(), THRESHOLD_RADIUS);
    EXPECT_FLOAT_EQ(reward.positive(), POSITIVE);
}

TEST_F(OnGoalTest, ExactlyOnGoalEarnsTheMaximum)
{
    const OnGoal reward(THRESHOLD_RADIUS, POSITIVE);

    EXPECT_NEAR(reward(robot)[0], POSITIVE, EPSILON);
}

TEST_F(OnGoalTest, RewardFallsLinearlyWithDistance)
{
    robot->setPose(0.5f, 0.0f, 0.0f);
    const OnGoal reward(THRESHOLD_RADIUS, POSITIVE);

    EXPECT_NEAR(reward(robot)[0], POSITIVE * (THRESHOLD_RADIUS - 0.5f) / THRESHOLD_RADIUS, EPSILON);
}

TEST_F(OnGoalTest, AtOrBeyondTheRadiusIsZero)
{
    robot->setPose(THRESHOLD_RADIUS, 0.0f, 0.0f);
    const OnGoal reward(THRESHOLD_RADIUS, POSITIVE);
    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);

    robot->setPose(5.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

// ============================================================================
// Goal Radius Fallback Tests
// ============================================================================

// Regression: the fallback used the node's circle radius instead of
// Node::getGoalRadius(), the radius the simulation's goal-reached check
// uses, so reward and episode termination disagreed on "on goal".
TEST_F(OnGoalTest, FallbackUsesTheGoalRadiusLikeSimulationTermination)
{
    // Goal node circle radius 0.5, but a separately configured goal radius 1.0
    robot->setGoal(Node(0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0, 0), 1.0f);
    robot->setPose(0.75f, 0.0f, 0.0f);
    const OnGoal reward(USE_GOAL_RADIUS, POSITIVE);

    // distance 0.75 within goal radius 1.0: (1.0 - 0.75) / 1.0; the circle radius would give 0
    EXPECT_NEAR(reward(robot)[0], 0.25f, EPSILON);
}

TEST_F(OnGoalTest, FallbackUsesTheNodeRadiusWithoutAGoalRadius)
{
    robot->setPose(0.25f, 0.0f, 0.0f);
    const OnGoal reward(USE_GOAL_RADIUS, POSITIVE);

    EXPECT_NEAR(reward(robot)[0], POSITIVE * (0.5f - 0.25f) / 0.5f, EPSILON);
}

TEST_F(OnGoalTest, FallbackWithoutAPositiveRadiusYieldsZero)
{
    robot->setGoal(0.0f, 0.0f, 0.0f); // a goal without extent
    const OnGoal reward(USE_GOAL_RADIUS, POSITIVE);

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

// ============================================================================
// Batch Tests
// ============================================================================

TEST_F(OnGoalTest, OneRewardPerRobotInOrder)
{
    const std::shared_ptr<Robot> far = fixtures::holonomicRobot("far");
    far->setGoal(0.0f, 0.0f, 0.5f);
    far->setPose(9.0f, 0.0f, 0.0f);
    const OnGoal reward(THRESHOLD_RADIUS, POSITIVE);

    const std::vector<float> output = reward(fixtures::RobotList{robot, far});

    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_NEAR(output[0], POSITIVE, EPSILON);
    EXPECT_FLOAT_EQ(output[1], 0.0f);
}
