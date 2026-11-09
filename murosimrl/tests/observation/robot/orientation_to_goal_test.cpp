// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/orientation_to_goal.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <vector>

namespace
{
using murosim::math::PI_2;
using murosimrl::observation::robot::OrientationToGoal;
using fixtures::EPSILON;

constexpr float NO_THRESHOLD = 0.0f;
constexpr float THRESHOLD_RADIUS = 10.0f;
constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class OrientationToGoalTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(1.5f, -2.0f, 0.0f);
        robot->setGoal(4.5f, 2.0f, 0.5f); // offset (3, 4): direction (0.6, 0.8), distance 5
        step();
    }
};

// ============================================================================
// Direction Tests
// ============================================================================

TEST_F(OrientationToGoalTest, ShapeIsBatchByTwo)
{
    const OrientationToGoal observation(NO_THRESHOLD, NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 2}));
    EXPECT_EQ(observation.name(), "orientation_to_goal");
}

TEST_F(OrientationToGoalTest, UnitDirectionInRobotFrame)
{
    const OrientationToGoal observation(NO_THRESHOLD, NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_NEAR(output[0], 0.6f, EPSILON);
    EXPECT_NEAR(output[1], 0.8f, EPSILON);
}

TEST_F(OrientationToGoalTest, DirectionRotatesWithTheRobot)
{
    robot->setPose(1.5f, -2.0f, PI_2); // facing +y: the goal's (3, 4) offset becomes (4, -3) locally
    step();
    const OrientationToGoal observation(NO_THRESHOLD, NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 0.8f, EPSILON);
    EXPECT_NEAR(output[1], -0.6f, EPSILON);
}

TEST_F(OrientationToGoalTest, OnGoalIsZero)
{
    robot->setPose(4.5f, 2.0f, 0.0f);
    step();
    const OrientationToGoal observation(NO_THRESHOLD, NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_FLOAT_EQ(output[0], 0.0f);
    EXPECT_FLOAT_EQ(output[1], 0.0f);
}

// ============================================================================
// Threshold Tests
// ============================================================================

// Inside the threshold radius the direction shrinks with distance / threshold,
// so the observation fades out instead of flipping on the goal.
TEST_F(OrientationToGoalTest, ScalesInsideThreshold)
{
    const OrientationToGoal observation(THRESHOLD_RADIUS, NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 0.6f * 5.0f / THRESHOLD_RADIUS, EPSILON);
    EXPECT_NEAR(output[1], 0.8f * 5.0f / THRESHOLD_RADIUS, EPSILON);
    EXPECT_FLOAT_EQ(observation.thresholdRadius(), THRESHOLD_RADIUS);
}

TEST_F(OrientationToGoalTest, OutsideThresholdIsUnitLength)
{
    const OrientationToGoal observation(2.0f, NO_STACK); // the robot is 5 m away

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 0.6f, EPSILON);
    EXPECT_NEAR(output[1], 0.8f, EPSILON);
}

TEST_F(OrientationToGoalTest, NegativeThresholdIsClampedToZero)
{
    const OrientationToGoal observation(-1.0f, NO_STACK);

    EXPECT_FLOAT_EQ(observation.thresholdRadius(), 0.0f);
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(OrientationToGoalTest, StackIsNewestFirst)
{
    robot->setPose(4.5f, -1.0f, 0.0f); // goal straight in +y: local (0, 1)
    step();
    const OrientationToGoal observation(NO_THRESHOLD, 2);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(4));
    EXPECT_NEAR(output[0], 0.0f, EPSILON);
    EXPECT_NEAR(output[1], 1.0f, EPSILON);
    EXPECT_NEAR(output[2], 0.6f, EPSILON);
    EXPECT_NEAR(output[3], 0.8f, EPSILON);
}
