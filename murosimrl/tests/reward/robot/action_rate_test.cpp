// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/action_rate.hpp"
#include "fixtures.hpp"
#include <memory>
#include <vector>

namespace
{
using murosim::robot::Robot;
using murosimrl::reward::robot::ActionRate;
using fixtures::ACCELERATION_ANGULAR;
using fixtures::ACCELERATION_LINEAR;
using fixtures::EPSILON;
using fixtures::TIME_STEP;

constexpr float THRESHOLD_X = 0.15f;
constexpr float THRESHOLD_Y = 0.075f;
constexpr float THRESHOLD_ANGULAR = 0.2f;
constexpr float NEGATIVE = -0.1f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// The penalty compares the latest state's commanded and actual velocity.
// The fixture robot has acceleration limits, so after one step the actual
// velocity lags the command by a known amount.
class ActionRateTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        robot = fixtures::holonomicRobot("limited", false, true);
        fixtures::SingleRobotTest::SetUp();
    }

    ActionRate reward{THRESHOLD_X, THRESHOLD_Y, THRESHOLD_ANGULAR, NEGATIVE};
};

// ============================================================================
// Penalty Tests
// ============================================================================

TEST_F(ActionRateTest, ShapeAndConfiguration)
{
    EXPECT_EQ(reward.shape(robots()), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "action_rate");
    EXPECT_FLOAT_EQ(reward.thresholdX(), THRESHOLD_X);
    EXPECT_FLOAT_EQ(reward.thresholdY(), THRESHOLD_Y);
    EXPECT_FLOAT_EQ(reward.thresholdAngular(), THRESHOLD_ANGULAR);
    EXPECT_FLOAT_EQ(reward.negative(), NEGATIVE);
}

TEST_F(ActionRateTest, RobotWithoutStateEarnsNothing)
{
    const std::shared_ptr<Robot> fresh = fixtures::holonomicRobot("fresh");

    EXPECT_FLOAT_EQ(reward(fresh)[0], 0.0f);
}

TEST_F(ActionRateTest, MatchingCommandAndVelocityEarnsNothing)
{
    // after the reset both are zero
    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(ActionRateTest, ExcessBeyondThresholdIsPenalised)
{
    robot->setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);
    step();

    // after one step the velocity ramped to a * dt: the difference is 1 - a * dt
    const float excess = (1.0f - ACCELERATION_LINEAR * TIME_STEP) - THRESHOLD_X;
    EXPECT_NEAR(reward(robot)[0], NEGATIVE * excess, EPSILON);
}

TEST_F(ActionRateTest, DifferenceWithinThresholdEarnsNothing)
{
    // a command of 0.1 ramps to 0.05 in one step: the difference of 0.05 is below the 0.15 threshold
    robot->setActuatorCommandVelocity(0.1f, 0.0f, 0.0f);
    step();

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(ActionRateTest, AxesAreSummed)
{
    robot->setActuatorCommandVelocity(1.0f, 1.0f, 1.0f);
    step();

    const float excess_linear = (1.0f - ACCELERATION_LINEAR * TIME_STEP);
    const float excess_angular = (1.0f - ACCELERATION_ANGULAR * TIME_STEP);
    const float expected = NEGATIVE * ((excess_linear - THRESHOLD_X) + (excess_linear - THRESHOLD_Y) +
                                       (excess_angular - THRESHOLD_ANGULAR));
    EXPECT_NEAR(reward(robot)[0], expected, EPSILON);
}

TEST_F(ActionRateTest, PenaltyVanishesOnceTheVelocityCatchesUp)
{
    robot->setActuatorCommandVelocity(0.5f, 0.0f, 0.0f);
    step(20); // 2 s at 0.5 m/s^2 reaches 0.5 m/s after 1 s

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}
