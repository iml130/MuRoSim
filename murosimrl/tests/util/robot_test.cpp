// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/util/robot.hpp"
#include "murosim/robot/state.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace
{
using murosim::math::PI_2;
using murosim::actuator::Actuator;
using murosim::navigation::Node;
using murosim::robot::Robot;
using murosim::robot::State;
using murosimrl::util::robot::axesForActuatorType;
using murosimrl::util::robot::checkRobotHasSameActuator;
using murosimrl::util::robot::checkRobotHasStates;
using murosimrl::util::robot::distanceToGoalFromState;
using murosimrl::util::robot::orientationToGoalFromState;
using murosimrl::util::robot::requireConsistentLidars;
using fixtures::EPSILON;

/// State of a robot at `pose` with the goal at `goal`.
State stateWithGoal(const Eigen::Affine2f &pose, const Node &goal)
{
    const std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    robot->setPose(pose);
    robot->setGoal(goal);
    robot->makeState();
    return robot->getStateAt(0);
}
} // namespace

// ============================================================================
// Actuator Check Tests
// ============================================================================

TEST(UtilRobotActuator, SameActuatorTypeIsAccepted)
{
    EXPECT_TRUE(checkRobotHasSameActuator({fixtures::holonomicRobot("a"), fixtures::holonomicRobot("b")}));
    EXPECT_TRUE(checkRobotHasSameActuator({fixtures::differentialRobot("a")}));
    EXPECT_TRUE(checkRobotHasSameActuator({}));
}

TEST(UtilRobotActuator, MixedActuatorTypesAreRejected)
{
    EXPECT_FALSE(checkRobotHasSameActuator({fixtures::holonomicRobot("a"), fixtures::differentialRobot("b")}));
}

TEST(UtilRobotActuator, AxesForActuatorType)
{
    std::array<size_t, 3> axes;

    EXPECT_EQ(axesForActuatorType(Actuator::TYPE::X_Y_ANGULAR, axes), static_cast<size_t>(3));
    EXPECT_EQ(axes, (std::array<size_t, 3>{0, 1, 2}));

    EXPECT_EQ(axesForActuatorType(Actuator::TYPE::X_ANGULAR, axes), static_cast<size_t>(2));
    EXPECT_EQ(axes[0], static_cast<size_t>(0));
    EXPECT_EQ(axes[1], static_cast<size_t>(2)); // angular is the z component

    EXPECT_EQ(axesForActuatorType(Actuator::TYPE::X_Y, axes), static_cast<size_t>(2));
    EXPECT_EQ(axes[0], static_cast<size_t>(0));
    EXPECT_EQ(axes[1], static_cast<size_t>(1));
}

// ============================================================================
// State Check Tests
// ============================================================================

TEST(UtilRobotStates, RobotsWithStatesAreAccepted)
{
    const fixtures::RobotList robots = {fixtures::holonomicRobot("a"), fixtures::holonomicRobot("b")};
    robots[0]->makeState();
    robots[1]->makeState();

    EXPECT_TRUE(checkRobotHasStates(robots));
    EXPECT_TRUE(checkRobotHasStates({}));
}

TEST(UtilRobotStates, RobotWithoutStateIsRejected)
{
    const fixtures::RobotList robots = {fixtures::holonomicRobot("a"), fixtures::holonomicRobot("b")};
    robots[0]->makeState();

    EXPECT_FALSE(checkRobotHasStates(robots));
}

// ============================================================================
// Lidar Check Tests
// ============================================================================

TEST(UtilRobotLidars, ConsistentLidarsAreAccepted)
{
    const fixtures::RobotList robots = {fixtures::holonomicRobot("a", true), fixtures::differentialRobot("b", true)};

    EXPECT_NO_THROW(requireConsistentLidars(robots, 0));
}

TEST(UtilRobotLidars, LidarIndexOutOfRangeThrows)
{
    const fixtures::RobotList robots = {fixtures::holonomicRobot("a", true), fixtures::holonomicRobot("b", false)};

    EXPECT_THROW(requireConsistentLidars(robots, 0), std::runtime_error); // robot b has none
    EXPECT_THROW(requireConsistentLidars({robots[0]}, 1), std::runtime_error);
}

TEST(UtilRobotLidars, DifferentRayCountsThrow)
{
    const std::unique_ptr<Actuator> actuator = std::make_unique<murosim::actuator::HolonomicSimple>();
    const std::vector<murosim::sensor::Lidar> coarse = {
        murosim::sensor::Lidar(Eigen::Affine2f::Identity(), fixtures::LIDAR_FOV, fixtures::LIDAR_RAYS / 2,
                               fixtures::LIDAR_RANGE_MIN, fixtures::LIDAR_RANGE_MAX)};
    const fixtures::RobotList robots = {
        fixtures::holonomicRobot("fine", true),
        std::make_shared<Robot>("coarse", actuator, murosim::geometry::primitives::square(fixtures::BODY_SIZE),
                                murosim::geometry::Shape(), coarse)};

    EXPECT_THROW(requireConsistentLidars(robots, 0), std::runtime_error);
}

// ============================================================================
// Goal From State Tests
// ============================================================================

TEST(UtilRobotGoal, DistanceToGoalFromState)
{
    const State state = stateWithGoal(murosim::geometry::primitives::pose(1.0f, 1.0f, 0.0f), Node(4.0f, 5.0f, 0.5f));

    EXPECT_NEAR(distanceToGoalFromState(state), 5.0f, EPSILON);
}

TEST(UtilRobotGoal, OrientationToGoalFromStateIsInRobotFrame)
{
    // Robot faces +y, goal straight ahead in the world: dead ahead locally
    const State state = stateWithGoal(murosim::geometry::primitives::pose(0.0f, 0.0f, PI_2), Node(0.0f, 5.0f, 0.5f));

    const Eigen::Vector2f orientation = orientationToGoalFromState(state);

    EXPECT_NEAR(orientation.x(), 1.0f, EPSILON);
    EXPECT_NEAR(orientation.y(), 0.0f, EPSILON);
}

TEST(UtilRobotGoal, OrientationOnGoalIsZero)
{
    const State state = stateWithGoal(murosim::geometry::primitives::pose(2.0f, 2.0f, 0.0f), Node(2.0f, 2.0f, 0.5f));

    EXPECT_TRUE(orientationToGoalFromState(state).isZero());
}
