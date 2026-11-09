// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/action.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::actuator::CONTROL_MODE;
using murosim::robot::Robot;
using murosimrl::Action;
using fixtures::ACCELERATION_ANGULAR;
using fixtures::ACCELERATION_LINEAR;
using fixtures::EPSILON;
using fixtures::TIME_STEP;
using fixtures::VELOCITY_MAX;

constexpr bool NORMALIZED = true;
constexpr bool RAW = false;
} // namespace

// ============================================================================
// Size And Shape Tests
// ============================================================================

TEST(ActionShape, HolonomicRobotHasThreeAxes)
{
    const std::shared_ptr<Robot> robot = fixtures::holonomicRobot("holonomic");

    EXPECT_EQ(Action::size(robot), static_cast<size_t>(3));
    EXPECT_EQ(Action::shape(robot), (std::vector<size_t>{3}));
}

TEST(ActionShape, DifferentialRobotHasTwoAxes)
{
    const std::shared_ptr<Robot> robot = fixtures::differentialRobot("differential");

    EXPECT_EQ(Action::size(robot), static_cast<size_t>(2));
    EXPECT_EQ(Action::shape(robot), (std::vector<size_t>{2}));
}

TEST(ActionShape, RobotListIsBatchByAxes)
{
    const fixtures::RobotList robots = {fixtures::differentialRobot("a"), fixtures::differentialRobot("b")};

    EXPECT_EQ(Action::shape(robots), (std::vector<size_t>{2, 2}));
    EXPECT_TRUE(Action::shape(fixtures::RobotList()).empty());
}

TEST(ActionShape, NormalizeFlagIsStored)
{
    EXPECT_TRUE(Action(NORMALIZED).normalize());
    EXPECT_FALSE(Action(RAW).normalize());
}

// ============================================================================
// Velocity Mode Tests
// ============================================================================

TEST(ActionVelocityMode, NormalizedActionMapsOntoVelocityLimits)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, true);
    ASSERT_EQ(robot->getActuatorControlMode(), CONTROL_MODE::VELOCITY);
    const Action action(NORMALIZED);
    const float data[3] = {1.0f, -1.0f, 0.0f};

    action(robot, data);

    // [-1, 1] maps onto [-VELOCITY_MAX, VELOCITY_MAX]
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().x(), VELOCITY_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().y(), -VELOCITY_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().z(), 0.0f);
    EXPECT_TRUE(robot->getActuatorCommandAcceleration().isZero());
}

TEST(ActionVelocityMode, NormalizedActionOnAsymmetricRange)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, false, 0.0f); // x in [0, 1]
    const Action action(NORMALIZED);
    const float data[3] = {-1.0f, 0.0f, 0.0f};

    action(robot, data);

    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().x(), 0.0f); // -1 is the range minimum
    const float middle[3] = {0.0f, 0.0f, 0.0f};
    action(robot, middle);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().x(), VELOCITY_MAX / 2.0f); // 0 is the range middle
}

TEST(ActionVelocityMode, RawActionIsWrittenUnchanged)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    const Action action(RAW);
    const float data[3] = {0.25f, -0.5f, 0.75f};

    action(robot, data);

    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().x(), 0.25f);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().y(), -0.5f);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().z(), 0.75f);
}

TEST(ActionVelocityMode, RawActionIsClampedByTheActuator)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    const Action action(RAW);
    const float data[3] = {5.0f, 0.0f, 0.0f};

    action(robot, data);

    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().x(), VELOCITY_MAX);
}

TEST(ActionVelocityMode, DifferentialActionSkipsTheYAxis)
{
    std::shared_ptr<Robot> robot = fixtures::differentialRobot("robot");
    const Action action(RAW);
    const float data[2] = {0.5f, 0.25f}; // x, angular

    action(robot, data);

    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().z(), 0.25f);
}

// ============================================================================
// Acceleration Mode Tests
// ============================================================================

TEST(ActionAccelerationMode, NormalizedActionMapsOntoAccelerationLimits)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, true);
    robot->setActuatorControlMode(CONTROL_MODE::ACCELERATION);
    const Action action(NORMALIZED);
    const float data[3] = {1.0f, -1.0f, 0.5f};

    action(robot, data);

    // [-1, 1] maps onto the symmetric acceleration limits, not the velocity limits
    EXPECT_FLOAT_EQ(robot->getActuatorCommandAcceleration().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandAcceleration().y(), -ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(robot->getActuatorCommandAcceleration().z(), ACCELERATION_ANGULAR / 2.0f);
    EXPECT_TRUE(robot->getActuatorCommandVelocity().isZero());
}

TEST(ActionAccelerationMode, CommandIntegratesIntoVelocity)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, true);
    robot->setActuatorControlMode(CONTROL_MODE::ACCELERATION);
    const Action action(NORMALIZED);
    const float data[3] = {1.0f, 0.0f, 0.0f};
    action(robot, data);

    robot->move(TIME_STEP);

    EXPECT_NEAR(robot->getActuatorVelocity().x(), ACCELERATION_LINEAR * TIME_STEP, EPSILON);
}

TEST(ActionAccelerationMode, WithoutAccelerationLimitsThrows)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, false);
    robot->setActuatorControlMode(CONTROL_MODE::ACCELERATION);
    const Action action(NORMALIZED);
    const float data[3] = {1.0f, 0.0f, 0.0f};

    // Without limits every acceleration command would clamp to zero; the
    // action must fail loudly instead of being silently inert
    EXPECT_THROW(action(robot, data), std::runtime_error);
}

// ============================================================================
// Robot List Tests
// ============================================================================

TEST(ActionRobotList, AppliesOneActionPerRobotInOrder)
{
    const fixtures::RobotList robots = {fixtures::differentialRobot("a"), fixtures::differentialRobot("b")};
    const Action action(RAW);
    const float data[4] = {0.1f, 0.2f, 0.3f, 0.4f}; // robot a: x, angular; robot b: x, angular

    action(robots, data);

    EXPECT_FLOAT_EQ(robots[0]->getActuatorCommandVelocity().x(), 0.1f);
    EXPECT_FLOAT_EQ(robots[0]->getActuatorCommandVelocity().z(), 0.2f);
    EXPECT_FLOAT_EQ(robots[1]->getActuatorCommandVelocity().x(), 0.3f);
    EXPECT_FLOAT_EQ(robots[1]->getActuatorCommandVelocity().z(), 0.4f);
}

TEST(ActionRobotList, MixedActuatorTypesThrow)
{
    const fixtures::RobotList robots = {fixtures::holonomicRobot("a"), fixtures::differentialRobot("b")};
    const Action action(RAW);
    const float data[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    EXPECT_THROW(action(robots, data), std::runtime_error);
}

TEST(ActionRobotList, EmptyListIsNoOp)
{
    const Action action(RAW);

    EXPECT_NO_THROW(action(fixtures::RobotList(), nullptr));
}

// ============================================================================
// Neutral Point Tests
// ============================================================================

TEST(ActionNeutralPoint, SymmetricVelocityRangeHasZeroNeutralPoint)
{
    const std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");

    const std::vector<float> neutral = Action::getNeutralPoint(robot);

    ASSERT_EQ(neutral.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(neutral[0], 0.0f);
    EXPECT_FLOAT_EQ(neutral[1], 0.0f);
    EXPECT_FLOAT_EQ(neutral[2], 0.0f);
}

TEST(ActionNeutralPoint, AsymmetricVelocityRangeShiftsNeutralPoint)
{
    const std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, false, 0.0f); // x in [0, 1]

    const std::vector<float> neutral = Action::getNeutralPoint(robot);

    ASSERT_EQ(neutral.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(neutral[0], -1.0f); // zero velocity is the lower end of [0, 1]
    EXPECT_FLOAT_EQ(neutral[1], 0.0f);
}

TEST(ActionNeutralPoint, FollowsControlMode)
{
    std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, true, 0.0f);
    ASSERT_FLOAT_EQ(Action::getNeutralPoint(robot)[0], -1.0f);

    robot->setActuatorControlMode(CONTROL_MODE::ACCELERATION);

    // the acceleration range is symmetric, so its neutral point is 0
    EXPECT_FLOAT_EQ(Action::getNeutralPoint(robot)[0], 0.0f);
}

TEST(ActionNeutralPoint, PointerOverloadMatchesVector)
{
    const std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot", false, false, 0.0f);
    float output[3] = {9.0f, 9.0f, 9.0f};

    Action::getNeutralPoint(robot, output);

    const std::vector<float> expected = Action::getNeutralPoint(robot);
    EXPECT_FLOAT_EQ(output[0], expected[0]);
    EXPECT_FLOAT_EQ(output[1], expected[1]);
    EXPECT_FLOAT_EQ(output[2], expected[2]);
}
