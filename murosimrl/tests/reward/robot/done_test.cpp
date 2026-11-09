// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/done.hpp"
#include "fixtures.hpp"
#include <memory>
#include <vector>

namespace
{
using murosim::robot::Robot;
using murosimrl::reward::robot::Done;

constexpr float REACHED_GOAL = 3.0f;
constexpr float COLLISION_WORLD = -1.5f;
constexpr float COLLISION_ROBOT = -2.0f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// The reward reads the robot's status flags only, which the tests set
// directly instead of driving the simulation into each outcome.
class DoneRewardTest : public ::testing::Test
{
protected:
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    Done reward{REACHED_GOAL, COLLISION_WORLD, COLLISION_ROBOT};
};

// ============================================================================
// Outcome Tests
// ============================================================================

TEST_F(DoneRewardTest, ShapeAndConfiguration)
{
    EXPECT_EQ(reward.shape(fixtures::robotList(robot)), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "done");
    EXPECT_FLOAT_EQ(reward.positiveReachedGoal(), REACHED_GOAL);
    EXPECT_FLOAT_EQ(reward.negativeCollisionWorld(), COLLISION_WORLD);
    EXPECT_FLOAT_EQ(reward.negativeCollisionRobot(), COLLISION_ROBOT);
}

TEST_F(DoneRewardTest, UnfinishedRobotEarnsNothing)
{
    robot->setStatusCollision(Robot::STATUS_COLLISION::WORLD); // not finished yet

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(DoneRewardTest, ReachedGoalEarnsThePositiveReward)
{
    robot->setStatusState(Robot::STATUS_STATE::REACHED_GOAL);
    robot->finished();

    EXPECT_FLOAT_EQ(reward(robot)[0], REACHED_GOAL);
}

TEST_F(DoneRewardTest, WorldCollisionEarnsItsPenalty)
{
    robot->setStatusCollision(Robot::STATUS_COLLISION::WORLD);
    robot->finished();

    EXPECT_FLOAT_EQ(reward(robot)[0], COLLISION_WORLD);
}

TEST_F(DoneRewardTest, RobotCollisionEarnsItsPenalty)
{
    robot->setStatusCollision(Robot::STATUS_COLLISION::ROBOT);
    robot->finished();

    EXPECT_FLOAT_EQ(reward(robot)[0], COLLISION_ROBOT);
}

TEST_F(DoneRewardTest, ReachedGoalWinsOverACollision)
{
    // Another robot can cause a collision at goal arrival: the goal counts
    robot->setStatusState(Robot::STATUS_STATE::REACHED_GOAL);
    robot->setStatusCollision(Robot::STATUS_COLLISION::ROBOT);
    robot->finished();

    EXPECT_FLOAT_EQ(reward(robot)[0], REACHED_GOAL);
}

TEST_F(DoneRewardTest, FinishedForAnotherReasonEarnsNothing)
{
    robot->finished(); // no goal, no collision

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(DoneRewardTest, DynamicEntityCollisionEarnsNothing)
{
    robot->setStatusCollision(Robot::STATUS_COLLISION::DYNAMIC_ENTITY);
    robot->finished();

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

// ============================================================================
// Batch Tests
// ============================================================================

TEST_F(DoneRewardTest, OneRewardPerRobotInOrder)
{
    const std::shared_ptr<Robot> crashed = fixtures::holonomicRobot("crashed");
    robot->setStatusState(Robot::STATUS_STATE::REACHED_GOAL);
    robot->finished();
    crashed->setStatusCollision(Robot::STATUS_COLLISION::WORLD);
    crashed->finished();

    const std::vector<float> output = reward(fixtures::RobotList{robot, crashed});

    EXPECT_EQ(output, (std::vector<float>{REACHED_GOAL, COLLISION_WORLD}));
}
