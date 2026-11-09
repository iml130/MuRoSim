// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/forward_driving.hpp"
#include "fixtures.hpp"
#include <memory>
#include <vector>

namespace
{
using murosim::robot::Robot;
using murosimrl::reward::robot::ForwardDriving;
using fixtures::EPSILON;

constexpr float POSITIVE = 2.0f;
constexpr float THRESHOLD_Y = 0.2f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// reward = max(0, vx) / vx_max * max(0, threshold - |vy|) / threshold * positive,
// read from the commanded velocity, so no simulation step is needed.
class ForwardDrivingTest : public ::testing::Test
{
protected:
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    ForwardDriving reward{POSITIVE, THRESHOLD_Y};
};

// ============================================================================
// Reward Tests
// ============================================================================

TEST_F(ForwardDrivingTest, ShapeAndConfiguration)
{
    EXPECT_EQ(reward.shape(fixtures::robotList(robot)), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "forward_driving");
    EXPECT_FLOAT_EQ(reward.positive(), POSITIVE);
    EXPECT_FLOAT_EQ(reward.thresholdYVelocity(), THRESHOLD_Y);
}

TEST_F(ForwardDrivingTest, FullForwardWithoutSidewaysEarnsTheMaximum)
{
    robot->setActuatorCommandVelocity(fixtures::VELOCITY_MAX, 0.0f, 0.0f);

    EXPECT_NEAR(reward(robot)[0], POSITIVE, EPSILON);
}

TEST_F(ForwardDrivingTest, ForwardMotionIsScaledBySideward)
{
    robot->setActuatorCommandVelocity(0.5f, 0.1f, 0.0f);

    // vx scale 0.5, vy scale (0.2 - 0.1) / 0.2 = 0.5
    EXPECT_NEAR(reward(robot)[0], POSITIVE * 0.5f * 0.5f, EPSILON);
}

TEST_F(ForwardDrivingTest, BackwardMotionYieldsZero)
{
    robot->setActuatorCommandVelocity(-0.5f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(ForwardDrivingTest, SidewardAtOrAboveThresholdYieldsZero)
{
    robot->setActuatorCommandVelocity(1.0f, THRESHOLD_Y, 0.0f);
    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);

    robot->setActuatorCommandVelocity(1.0f, -0.5f, 0.0f);
    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

TEST_F(ForwardDrivingTest, AngularVelocityDoesNotMatter)
{
    robot->setActuatorCommandVelocity(1.0f, 0.0f, 1.0f);

    EXPECT_NEAR(reward(robot)[0], POSITIVE, EPSILON);
}

// ============================================================================
// Disabled Tests
// ============================================================================

// A non-positive threshold disables the reward instead of dividing by zero.
TEST_F(ForwardDrivingTest, ZeroThresholdDisablesReward)
{
    robot->setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);
    const ForwardDriving disabled(POSITIVE, 0.0f);

    EXPECT_FLOAT_EQ(disabled(robot)[0], 0.0f);
}

TEST_F(ForwardDrivingTest, ZeroMaxForwardVelocityDisablesReward)
{
    std::shared_ptr<Robot> stuck = fixtures::holonomicRobot("stuck", false, false, 0.0f);
    // x limits [0, 1] still allow forward motion; a robot that cannot drive forward has max 0
    const std::unique_ptr<murosim::actuator::Actuator> actuator =
        std::make_unique<murosim::actuator::HolonomicSimple>(0.0f, 0.0f, 1.0f, 1.0f);
    stuck = std::make_shared<Robot>("stuck", actuator, murosim::geometry::primitives::square(fixtures::BODY_SIZE),
                                    murosim::geometry::Shape(), std::vector<murosim::sensor::Lidar>());
    stuck->setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(reward(stuck)[0], 0.0f);
}

// ============================================================================
// Batch Tests
// ============================================================================

TEST_F(ForwardDrivingTest, OneRewardPerRobotInOrder)
{
    const std::shared_ptr<Robot> other = fixtures::holonomicRobot("other");
    robot->setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);
    other->setActuatorCommandVelocity(-1.0f, 0.0f, 0.0f);

    const std::vector<float> output = reward(fixtures::RobotList{robot, other});

    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_NEAR(output[0], POSITIVE, EPSILON);
    EXPECT_FLOAT_EQ(output[1], 0.0f);
}

TEST_F(ForwardDrivingTest, EmptyInputYieldsNothing)
{
    EXPECT_TRUE(reward(fixtures::RobotList()).empty());
    EXPECT_NO_THROW(reward(fixtures::RobotList(), nullptr));
}
