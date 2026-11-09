// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/delta_pose.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <vector>

namespace
{
using murosim::math::PI_2;
using murosimrl::observation::robot::DeltaPose;
using fixtures::EPSILON;

constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// After SetUp the robot has one state at the origin (the reset).
class DeltaPoseTest : public fixtures::SingleRobotTest
{
};

// ============================================================================
// Delta Tests
// ============================================================================

TEST_F(DeltaPoseTest, ShapeIsBatchByThree)
{
    const DeltaPose observation(NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 3}));
    EXPECT_EQ(observation.name(), "delta_pose");
}

TEST_F(DeltaPoseTest, SingleStateHasZeroDelta)
{
    ASSERT_EQ(robot->getStateSize(), static_cast<size_t>(1));
    const DeltaPose observation(NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(output[0], 0.0f);
    EXPECT_FLOAT_EQ(output[1], 0.0f);
    EXPECT_FLOAT_EQ(output[2], 0.0f);
}

TEST_F(DeltaPoseTest, DeltaFromTheOrigin)
{
    robot->setPose(1.0f, 0.5f, PI_2);
    step();
    const DeltaPose observation(NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 1.0f, EPSILON);
    EXPECT_NEAR(output[1], 0.5f, EPSILON);
    EXPECT_NEAR(output[2], PI_2, EPSILON);
}

TEST_F(DeltaPoseTest, DeltaIsExpressedInThePreviousFrame)
{
    robot->setPose(1.0f, 0.5f, PI_2);
    step();
    robot->setPose(2.0f, 0.5f, PI_2); // one metre in world x while facing +y
    step();
    const DeltaPose observation(NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 0.0f, EPSILON);
    EXPECT_NEAR(output[1], -1.0f, EPSILON); // to the robot's right
    EXPECT_NEAR(output[2], 0.0f, EPSILON);
}

TEST_F(DeltaPoseTest, DeltaOfDrivenMotion)
{
    robot->setActuatorCommandVelocity(0.5f, 0.0f, 0.0f);
    step();
    const DeltaPose observation(NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 0.5f * fixtures::TIME_STEP, EPSILON);
    EXPECT_NEAR(output[1], 0.0f, EPSILON);
    EXPECT_NEAR(output[2], 0.0f, EPSILON);
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(DeltaPoseTest, StackIsNewestFirstAndClampsToZero)
{
    robot->setPose(1.0f, 0.0f, 0.0f);
    step();
    robot->setPose(1.0f, 2.0f, 0.0f);
    step();
    const DeltaPose observation(3);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 3, 3}));
    ASSERT_EQ(output.size(), static_cast<size_t>(9));
    EXPECT_NEAR(output[1], 2.0f, EPSILON); // newest delta: +2 in y
    EXPECT_NEAR(output[3], 1.0f, EPSILON); // one step back: +1 in x
    EXPECT_FLOAT_EQ(output[6], 0.0f);      // beyond the history both reads clamp to the oldest state
    EXPECT_FLOAT_EQ(output[7], 0.0f);
    EXPECT_FLOAT_EQ(output[8], 0.0f);
}
