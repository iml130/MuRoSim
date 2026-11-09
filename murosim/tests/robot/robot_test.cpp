// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/robot/robot.hpp"
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::actuator::Actuator;
using murosim::actuator::CONTROL_MODE;
using murosim::actuator::HolonomicSimple;
using murosim::geometry::Circle;
using murosim::geometry::Shape;
using murosim::navigation::Node;
using murosim::robot::Robot;
using murosim::robot::State;
using murosim::sensor::Lidar;
using murosim::visualization::Color;

constexpr float EPSILON = 1e-5f;
constexpr float TIME_STEP = 0.1f;

// Limits of the fixture actuator. Min and max differ on purpose so a test
// that confuses them fails. The acceleration limits are needed for the
// acceleration command tests: without a limit every acceleration command
// is clamped to zero.
constexpr float VELOCITY_X_MIN = -0.5f;
constexpr float VELOCITY_X_MAX = 2.0f;
constexpr float VELOCITY_Y_MAX = 1.0f;
constexpr float VELOCITY_ANGULAR_MAX = PI_2;
constexpr float ACCELERATION_LINEAR = 0.5f;
constexpr float ACCELERATION_ANGULAR = PI_4;

constexpr float BODY_SIZE = 0.5f; // side length of the square body
constexpr size_t LIDAR_RAYS = 100;
constexpr float LIDAR_FOV = PI;
constexpr float LIDAR_RANGE_MAX = 10.0f;

// Color normalises 8-bit channels by 1/256, a power of two, so the stored
// float is exact
constexpr float COLOR_SCALE = 1.0f / 256.0f;

Lidar frontLidar(const size_t rays, const float fov, const float range_max)
{
    return Lidar(Eigen::Affine2f::Identity(), fov, rays, 0.0f, range_max);
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class RobotTest : public ::testing::Test
{
protected:
    std::unique_ptr<Actuator> actuator = std::make_unique<HolonomicSimple>(
        VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX, ACCELERATION_LINEAR,
        ACCELERATION_ANGULAR);
    Shape shape_robot = murosim::geometry::primitives::square(BODY_SIZE);
    Shape shape_lidar;
    std::vector<Lidar> lidars = {frontLidar(LIDAR_RAYS, LIDAR_FOV, LIDAR_RANGE_MAX)};
    Robot robot{"test_robot", actuator, shape_robot, shape_lidar, lidars};
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(RobotTest, ConstructorStoresNameAndLidars)
{
    EXPECT_EQ(robot.getName(), "test_robot");
    EXPECT_TRUE(robot.hasLidar());
    EXPECT_EQ(robot.getLidarSize(), static_cast<size_t>(1));
}

TEST_F(RobotTest, ConstructorWithoutLidars)
{
    const std::vector<Lidar> no_lidars;
    const Robot blind("blind", actuator, shape_robot, shape_lidar, no_lidars);

    EXPECT_FALSE(blind.hasLidar());
    EXPECT_EQ(blind.getLidarSize(), static_cast<size_t>(0));
}

TEST_F(RobotTest, ConstructorStartsAtIdentityPose)
{
    EXPECT_FLOAT_EQ(robot.getPosition().x(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getRotationAngle(), 0.0f);
}

TEST_F(RobotTest, ConstructorStartsIdleWithoutCollision)
{
    EXPECT_EQ(robot.getStatusState(), Robot::STATUS_STATE::IDLE);
    EXPECT_EQ(robot.getStatusCollision(), Robot::STATUS_COLLISION::NONE);
    EXPECT_FALSE(robot.isFinished());
    EXPECT_FALSE(robot.hasStates());
    EXPECT_FALSE(robot.hasPathPoints());
    EXPECT_EQ(robot.getSimulatedStepCount(), static_cast<size_t>(0));
    EXPECT_FLOAT_EQ(robot.getTimeOnGoal(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getTraveledDistance(), 0.0f);
}

TEST_F(RobotTest, ConstructorClonesActuator)
{
    // The robot owns its own actuator: commanding the robot leaves the
    // template untouched
    robot.setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(actuator->getCommandVelocity().x(), 0.0f);
}

TEST_F(RobotTest, CopyConstructorCopiesPoseAndName)
{
    robot.setPose(1.0f, 2.0f, 0.5f);

    const Robot copy(robot);

    EXPECT_EQ(copy.getName(), "test_robot");
    EXPECT_FLOAT_EQ(copy.getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(copy.getPosition().y(), 2.0f);
    EXPECT_NEAR(copy.getRotationAngle(), 0.5f, EPSILON);
}

TEST_F(RobotTest, CopyIsIndependentOfOriginal)
{
    Robot copy(robot);

    copy.setPose(3.0f, 4.0f, 0.0f);
    copy.setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getActuatorCommandVelocity().x(), 0.0f);
}

TEST_F(RobotTest, MoveConstructorKeepsPoseAndName)
{
    robot.setPose(3.0f, 4.0f, 1.0f);

    const Robot moved(std::move(robot));

    EXPECT_EQ(moved.getName(), "test_robot");
    EXPECT_FLOAT_EQ(moved.getPosition().x(), 3.0f);
    EXPECT_FLOAT_EQ(moved.getPosition().y(), 4.0f);
    EXPECT_NEAR(moved.getRotationAngle(), 1.0f, EPSILON);
}

TEST_F(RobotTest, CopyAssignmentCopiesPose)
{
    robot.setPose(1.0f, 2.0f, 0.0f);
    Robot other("other", actuator, shape_robot, shape_lidar, lidars);

    other = robot;

    EXPECT_EQ(other.getName(), "test_robot");
    EXPECT_FLOAT_EQ(other.getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(other.getPosition().y(), 2.0f);
}

// ============================================================================
// Pose Tests
// ============================================================================

TEST_F(RobotTest, SetPoseFloats)
{
    robot.setPose(1.0f, 2.0f, PI_2);

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 2.0f);
    EXPECT_NEAR(robot.getRotationAngle(), PI_2, EPSILON);
}

TEST_F(RobotTest, SetPoseAffine)
{
    const Eigen::Affine2f pose = murosim::geometry::primitives::pose(5.0f, 6.0f, PI_4);

    robot.setPose(pose);

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 5.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 6.0f);
    EXPECT_NEAR(robot.getRotationAngle(), PI_4, EPSILON);
    EXPECT_TRUE(robot.getPose().isApprox(pose));
}

TEST_F(RobotTest, SetPoseVectorAndRotation)
{
    robot.setPose(Eigen::Vector2f(3.0f, 4.0f), Eigen::Rotation2Df(PI_4));

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 3.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 4.0f);
    EXPECT_NEAR(robot.getRotationAngle(), PI_4, EPSILON);
}

TEST_F(RobotTest, SetPoseFromNodePlacesAtStartAngle)
{
    const Node node(2.0f, 3.0f, 1.0f, PI_2); // x, y, radius, start angle

    robot.setPoseFromNode(node);

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 2.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 3.0f);
    EXPECT_NEAR(robot.getRotationAngle(), PI_2, EPSILON);
}

TEST_F(RobotTest, SetPositionFloatsKeepsRotation)
{
    robot.setPose(0.0f, 0.0f, PI_2);

    robot.setPosition(7.0f, 8.0f);

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 7.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 8.0f);
    EXPECT_NEAR(robot.getRotationAngle(), PI_2, EPSILON);
}

TEST_F(RobotTest, SetPositionVectorKeepsRotation)
{
    robot.setPose(0.0f, 0.0f, PI_2);

    robot.setPosition(Eigen::Vector2f(9.0f, 10.0f));

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 9.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 10.0f);
    EXPECT_NEAR(robot.getRotationAngle(), PI_2, EPSILON);
}

TEST_F(RobotTest, SetRotationFloatKeepsPosition)
{
    robot.setPose(1.0f, 2.0f, 0.0f);

    robot.setRotation(PI_2);

    EXPECT_NEAR(robot.getRotationAngle(), PI_2, EPSILON);
    EXPECT_FLOAT_EQ(robot.getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 2.0f);
}

TEST_F(RobotTest, SetRotationMatrixKeepsPosition)
{
    robot.setPose(1.0f, 2.0f, 0.0f);

    robot.setRotation(Eigen::Rotation2Df(PI_4));

    EXPECT_NEAR(robot.getRotationAngle(), PI_4, EPSILON);
    EXPECT_NEAR(robot.getRotationMatrix().angle(), PI_4, EPSILON);
    EXPECT_FLOAT_EQ(robot.getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 2.0f);
}

TEST_F(RobotTest, GetCollisionRadiusEnclosesBody)
{
    // Smallest circle around a centred square: half the diagonal
    EXPECT_NEAR(robot.getCollisionRadius(), BODY_SIZE / 2.0f * std::sqrt(2.0f), EPSILON);
}

// ============================================================================
// Actuator Command Tests
// ============================================================================

TEST_F(RobotTest, SetActuatorCommandVelocityFloats)
{
    robot.setActuatorCommandVelocity(0.5f, 0.3f, 0.1f);

    const Eigen::Vector3f command = robot.getActuatorCommandVelocity();
    EXPECT_FLOAT_EQ(command.x(), 0.5f);
    EXPECT_FLOAT_EQ(command.y(), 0.3f);
    EXPECT_FLOAT_EQ(command.z(), 0.1f);
}

TEST_F(RobotTest, SetActuatorCommandVelocityVector)
{
    robot.setActuatorCommandVelocity(Eigen::Vector3f(0.2f, 0.4f, 0.6f));

    const Eigen::Vector3f command = robot.getActuatorCommandVelocity();
    EXPECT_FLOAT_EQ(command.x(), 0.2f);
    EXPECT_FLOAT_EQ(command.y(), 0.4f);
    EXPECT_FLOAT_EQ(command.z(), 0.6f);
}

TEST_F(RobotTest, SetActuatorCommandVelocityClampedToLimits)
{
    robot.setActuatorCommandVelocity(-10.0f, 10.0f, 10.0f);

    const Eigen::Vector3f command = robot.getActuatorCommandVelocity();
    EXPECT_FLOAT_EQ(command.x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(command.y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(command.z(), VELOCITY_ANGULAR_MAX);
}

TEST_F(RobotTest, SetActuatorCommandAccelerationFloats)
{
    robot.setActuatorCommandAcceleration(0.1f, 0.2f, 0.3f);

    const Eigen::Vector3f command = robot.getActuatorCommandAcceleration();
    EXPECT_FLOAT_EQ(command.x(), 0.1f);
    EXPECT_FLOAT_EQ(command.y(), 0.2f);
    EXPECT_FLOAT_EQ(command.z(), 0.3f);
}

TEST_F(RobotTest, SetActuatorCommandAccelerationVector)
{
    robot.setActuatorCommandAcceleration(Eigen::Vector3f(0.3f, 0.2f, 0.1f));

    const Eigen::Vector3f command = robot.getActuatorCommandAcceleration();
    EXPECT_FLOAT_EQ(command.x(), 0.3f);
    EXPECT_FLOAT_EQ(command.y(), 0.2f);
    EXPECT_FLOAT_EQ(command.z(), 0.1f);
}

TEST_F(RobotTest, SetActuatorCommandAccelerationClampedToLimits)
{
    robot.setActuatorCommandAcceleration(1.0f, -2.0f, 3.0f);

    const Eigen::Vector3f command = robot.getActuatorCommandAcceleration();
    EXPECT_FLOAT_EQ(command.x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(command.y(), -ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(command.z(), ACCELERATION_ANGULAR);
}

TEST_F(RobotTest, ControlModeDefaultsToVelocity)
{
    EXPECT_EQ(robot.getActuatorControlMode(), CONTROL_MODE::VELOCITY);
}

TEST_F(RobotTest, SetActuatorControlMode)
{
    robot.setActuatorControlMode(CONTROL_MODE::ACCELERATION);

    EXPECT_EQ(robot.getActuatorControlMode(), CONTROL_MODE::ACCELERATION);
}

// ============================================================================
// Actuator Info Tests
// ============================================================================

TEST_F(RobotTest, ActuatorVelocityIsZeroInitially)
{
    const Eigen::Vector3f velocity = robot.getActuatorVelocity();

    EXPECT_FLOAT_EQ(velocity.x(), 0.0f);
    EXPECT_FLOAT_EQ(velocity.y(), 0.0f);
    EXPECT_FLOAT_EQ(velocity.z(), 0.0f);
}

TEST_F(RobotTest, GetActuatorVelocityLimits)
{
    const Eigen::Vector3f velocity_max = robot.getActuatorVelocityMax();
    const Eigen::Vector3f velocity_min = robot.getActuatorVelocityMin();

    EXPECT_FLOAT_EQ(velocity_max.x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(velocity_max.y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(velocity_max.z(), VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(velocity_min.x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(velocity_min.y(), -VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(velocity_min.z(), -VELOCITY_ANGULAR_MAX);
}

TEST_F(RobotTest, GetActuatorAccelerationMax)
{
    const Eigen::Vector3f acceleration_max = robot.getActuatorAccelerationMax();

    EXPECT_FLOAT_EQ(acceleration_max.x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(acceleration_max.y(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(acceleration_max.z(), ACCELERATION_ANGULAR);
}

TEST_F(RobotTest, GetActuatorType)
{
    EXPECT_EQ(robot.getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
}

// ============================================================================
// Goal Tests
// ============================================================================

TEST_F(RobotTest, SetGoalNodeKeepsNodeRadius)
{
    const Node goal(5.0f, 6.0f, 1.0f);

    robot.setGoal(goal);

    EXPECT_FLOAT_EQ(robot.getGoal().getX(), 5.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getY(), 6.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getRadius(), 1.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getGoalRadius(), 1.0f); // falls back to the node radius
}

TEST_F(RobotTest, SetGoalNodeWithRadiusOverridesGoalRadius)
{
    const Node goal(5.0f, 6.0f, 1.0f);

    robot.setGoal(goal, 2.0f);

    EXPECT_FLOAT_EQ(robot.getGoal().getRadius(), 1.0f); // node circle unchanged
    EXPECT_FLOAT_EQ(robot.getGoal().getGoalRadius(), 2.0f);
}

// Regression: setGoal(x, y, radius) left the goal radius at the -1 sentinel,
// so checkGoalStatus could never report the goal as reached.
TEST_F(RobotTest, SetGoalFloatsSetsPositionAndGoalRadius)
{
    robot.setGoal(3.0f, 4.0f, 0.5f);

    EXPECT_FLOAT_EQ(robot.getGoal().getX(), 3.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getY(), 4.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getGoalRadius(), 0.5f);
}

// Regression: same -1 sentinel as above for the circle overload.
TEST_F(RobotTest, SetGoalCircleSetsPositionAndGoalRadius)
{
    robot.setGoal(Circle(7.0f, 8.0f, 1.5f));

    EXPECT_FLOAT_EQ(robot.getGoal().getX(), 7.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getY(), 8.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getGoalRadius(), 1.5f);
}

TEST_F(RobotTest, GetGroupIsGoalNodeGroup)
{
    const Node goal(1.0f, 1.0f, 0.5f, 0.0f, 0.0f, 7, 3); // x, y, radius, angle, bias, id, group

    robot.setGoal(goal);

    EXPECT_EQ(robot.getGroup(), static_cast<size_t>(3));
}

// ============================================================================
// Status Tests
// ============================================================================

TEST_F(RobotTest, StatusCollisionRoundTrip)
{
    robot.setStatusCollision(Robot::STATUS_COLLISION::WORLD);
    EXPECT_EQ(robot.getStatusCollision(), Robot::STATUS_COLLISION::WORLD);

    robot.setStatusCollision(Robot::STATUS_COLLISION::ROBOT);
    EXPECT_EQ(robot.getStatusCollision(), Robot::STATUS_COLLISION::ROBOT);

    robot.setStatusCollision(Robot::STATUS_COLLISION::NONE);
    EXPECT_EQ(robot.getStatusCollision(), Robot::STATUS_COLLISION::NONE);
}

TEST_F(RobotTest, StatusStateRoundTrip)
{
    robot.setStatusState(Robot::STATUS_STATE::SEARCH_GOAL);
    EXPECT_EQ(robot.getStatusState(), Robot::STATUS_STATE::SEARCH_GOAL);

    robot.setStatusState(Robot::STATUS_STATE::ON_GOAL);
    EXPECT_EQ(robot.getStatusState(), Robot::STATUS_STATE::ON_GOAL);

    robot.setStatusState(Robot::STATUS_STATE::IDLE);
    EXPECT_EQ(robot.getStatusState(), Robot::STATUS_STATE::IDLE);
}

TEST_F(RobotTest, StatusCollisionAsStringNamesEveryValue)
{
    EXPECT_EQ(Robot::getStatusCollisionAsString(Robot::STATUS_COLLISION::NONE), "NONE");
    EXPECT_EQ(Robot::getStatusCollisionAsString(Robot::STATUS_COLLISION::WORLD), "WORLD");
    EXPECT_EQ(Robot::getStatusCollisionAsString(Robot::STATUS_COLLISION::DYNAMIC_ENTITY), "DYNAMIC_ENTITY");
    EXPECT_EQ(Robot::getStatusCollisionAsString(Robot::STATUS_COLLISION::ROBOT), "ROBOT");

    robot.setStatusCollision(Robot::STATUS_COLLISION::ROBOT);
    EXPECT_EQ(robot.getStatusCollisionAsString(), "ROBOT");
}

TEST_F(RobotTest, StatusStateAsStringNamesEveryValue)
{
    EXPECT_EQ(Robot::getStatusStateAsString(Robot::STATUS_STATE::IDLE), "IDLE");
    EXPECT_EQ(Robot::getStatusStateAsString(Robot::STATUS_STATE::SEARCH_GOAL), "SEARCH_GOAL");
    EXPECT_EQ(Robot::getStatusStateAsString(Robot::STATUS_STATE::ON_GOAL), "ON_GOAL");
    EXPECT_EQ(Robot::getStatusStateAsString(Robot::STATUS_STATE::REACHED_GOAL), "REACHED_GOAL");
    EXPECT_EQ(Robot::getStatusStateAsString(Robot::STATUS_STATE::UNUSED), "UNUSED");

    robot.setStatusState(Robot::STATUS_STATE::ON_GOAL);
    EXPECT_EQ(robot.getStatusStateAsString(), "ON_GOAL");
}

// ============================================================================
// Lidar Tests
// ============================================================================

TEST_F(RobotTest, GetLidarsHoldsMountedLidars)
{
    ASSERT_EQ(robot.getLidars().size(), static_cast<size_t>(1));
    EXPECT_EQ(robot.getLidars()[0].getLaserSize(), LIDAR_RAYS);
}

TEST_F(RobotTest, GetLidarAtConst)
{
    const Robot &const_robot = robot;

    const Lidar &lidar = const_robot.getLidarAt(0);

    EXPECT_EQ(lidar.getLaserSize(), LIDAR_RAYS);
    EXPECT_FLOAT_EQ(lidar.getRangeMax(), LIDAR_RANGE_MAX);
}

TEST_F(RobotTest, GetLidarAtMutableModifiesMountedLidar)
{
    robot.getLidarAt(0).getArrayLaserDistanceBuffer()[0] = 1.5f;
    robot.getLidarAt(0).publishScan();

    const Lidar &lidar = robot.getLidarAt(0);
    EXPECT_FLOAT_EQ(lidar.getLaserDistance(0), 1.5f);
    EXPECT_EQ(lidar.getLaserDistance(LIDAR_RAYS - 1), std::numeric_limits<float>::infinity());
}

TEST_F(RobotTest, GetLidarsMutableModifiesMountedLidar)
{
    robot.getLidars()[0].getArrayLaserDistanceBuffer()[0] = 1.5f;
    robot.getLidars()[0].publishScan();

    EXPECT_FLOAT_EQ(robot.getLidarAt(0).getLaserDistance(0), 1.5f);
}

TEST_F(RobotTest, MultipleLidarsKeepMountingOrder)
{
    const std::vector<Lidar> three_lidars = {frontLidar(100, PI, 10.0f), frontLidar(50, PI_2, 5.0f),
                                             frontLidar(25, PI_4, 3.0f)};
    const Robot multi("multi", actuator, shape_robot, shape_lidar, three_lidars);

    ASSERT_EQ(multi.getLidarSize(), static_cast<size_t>(3));
    EXPECT_EQ(multi.getLidarAt(0).getLaserSize(), static_cast<size_t>(100));
    EXPECT_EQ(multi.getLidarAt(1).getLaserSize(), static_cast<size_t>(50));
    EXPECT_EQ(multi.getLidarAt(2).getLaserSize(), static_cast<size_t>(25));
}

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(RobotTest, GetShapeRobotIsTheSquareBody)
{
    EXPECT_EQ(robot.getShapeRobot().getLineSize(), static_cast<size_t>(4));
    EXPECT_EQ(robot.getShapeRobot().getCircleSize(), static_cast<size_t>(0));
}

TEST_F(RobotTest, GetShapeLidarIsEmpty)
{
    EXPECT_EQ(robot.getShapeLidar().getLineSize(), static_cast<size_t>(0));
    EXPECT_EQ(robot.getShapeLidar().getCircleSize(), static_cast<size_t>(0));
}

// ============================================================================
// Path Tests
// ============================================================================

TEST_F(RobotTest, NoPathPointsInitially)
{
    EXPECT_FALSE(robot.hasPathPoints());
    EXPECT_EQ(robot.getPathPointSize(), static_cast<size_t>(0));
    EXPECT_TRUE(robot.getPathPoints().empty());
}

TEST_F(RobotTest, SetPathPointsStoresAllPoints)
{
    const std::vector<Eigen::Vector2f> path = {Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 2.0f),
                                               Eigen::Vector2f(3.0f, 4.0f)};

    robot.setPathPoints(path);

    EXPECT_TRUE(robot.hasPathPoints());
    ASSERT_EQ(robot.getPathPointSize(), static_cast<size_t>(3));
    EXPECT_EQ(robot.getPathPoints().size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(robot.getPathPointAt(0).x(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getPathPointAt(0).y(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getPathPointAt(1).x(), 1.0f);
    EXPECT_FLOAT_EQ(robot.getPathPointAt(1).y(), 2.0f);
    EXPECT_FLOAT_EQ(robot.getPathPointAt(2).x(), 3.0f);
    EXPECT_FLOAT_EQ(robot.getPathPointAt(2).y(), 4.0f);
}

TEST_F(RobotTest, SetPathPointsReplacesPreviousPath)
{
    robot.setPathPoints({Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 1.0f)});

    robot.setPathPoints({Eigen::Vector2f(5.0f, 5.0f)});

    ASSERT_EQ(robot.getPathPointSize(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(robot.getPathPointAt(0).x(), 5.0f);
}

// ============================================================================
// State Tests
// ============================================================================

TEST_F(RobotTest, NoStatesInitially)
{
    EXPECT_FALSE(robot.hasStates());
    EXPECT_EQ(robot.getStateSize(), static_cast<size_t>(0));
}

TEST_F(RobotTest, MakeStateRecordsCurrentPose)
{
    robot.setPose(5.0f, 6.0f, PI_4);

    robot.makeState();

    EXPECT_TRUE(robot.hasStates());
    ASSERT_EQ(robot.getStateSize(), static_cast<size_t>(1));
    const State &state = robot.getStateAt(0);
    EXPECT_FLOAT_EQ(state.getPose().translation().x(), 5.0f);
    EXPECT_FLOAT_EQ(state.getPose().translation().y(), 6.0f);
    EXPECT_NEAR(Eigen::Rotation2Df(state.getPose().rotation()).angle(), PI_4, EPSILON);
}

TEST_F(RobotTest, MakeStateRecordsCommandAndGoal)
{
    robot.setActuatorCommandVelocity(0.5f, 0.3f, 0.1f);
    robot.setGoal(3.0f, 4.0f, 0.5f);

    robot.makeState();

    const State &state = robot.getStateAt(0);
    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().y(), 0.3f);
    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().z(), 0.1f);
    EXPECT_FLOAT_EQ(state.getGoal().getX(), 3.0f);
    EXPECT_FLOAT_EQ(state.getGoal().getY(), 4.0f);
    ASSERT_EQ(state.getLidarDistanceSize(), static_cast<size_t>(1));
    EXPECT_EQ(state.getLidarDistanceAt(0).size(), LIDAR_RAYS);
}

TEST_F(RobotTest, GetStateAtZeroIsMostRecent)
{
    robot.setPose(1.0f, 0.0f, 0.0f);
    robot.makeState();
    robot.setPose(2.0f, 0.0f, 0.0f);
    robot.makeState();

    ASSERT_EQ(robot.getStateSize(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(robot.getStateAt(0).getPose().translation().x(), 2.0f);
    EXPECT_FLOAT_EQ(robot.getStateAt(1).getPose().translation().x(), 1.0f);
}

TEST_F(RobotTest, GetStateAtOrOldestClampsToOldestState)
{
    robot.setPose(1.0f, 0.0f, 0.0f);
    robot.makeState();
    robot.setPose(2.0f, 0.0f, 0.0f);
    robot.makeState();

    EXPECT_FLOAT_EQ(robot.getStateAtOrOldest(0).getPose().translation().x(), 2.0f);
    EXPECT_FLOAT_EQ(robot.getStateAtOrOldest(1).getPose().translation().x(), 1.0f);
    EXPECT_FLOAT_EQ(robot.getStateAtOrOldest(100).getPose().translation().x(), 1.0f); // past the history
}

TEST_F(RobotTest, SetStateBufferSizeDiscardsRecordedStates)
{
    robot.makeState();
    robot.makeState();

    robot.setStateBufferSize(16);

    EXPECT_FALSE(robot.hasStates());
    EXPECT_EQ(robot.getStateSize(), static_cast<size_t>(0));
}

// ============================================================================
// Color Tests
// ============================================================================

TEST_F(RobotTest, DefaultColorsAreBlueGreenBlue)
{
    const Color color_robot = robot.getColorRobot();
    const Color color_lidar = robot.getColorLidar();
    const Color color_path = robot.getColorPath();

    EXPECT_FLOAT_EQ(color_robot.r(), 0.0f);
    EXPECT_FLOAT_EQ(color_robot.g(), 0.0f);
    EXPECT_FLOAT_EQ(color_robot.b(), 1.0f);
    EXPECT_FLOAT_EQ(color_lidar.r(), 0.0f);
    EXPECT_FLOAT_EQ(color_lidar.g(), 1.0f);
    EXPECT_FLOAT_EQ(color_lidar.b(), 0.0f);
    EXPECT_FLOAT_EQ(color_path.r(), 0.0f);
    EXPECT_FLOAT_EQ(color_path.g(), 0.0f);
    EXPECT_FLOAT_EQ(color_path.b(), 1.0f);
}

TEST_F(RobotTest, SetColorRobotFromBytes)
{
    robot.setColorRobot(static_cast<uint8_t>(255), static_cast<uint8_t>(128), static_cast<uint8_t>(64));

    const Color color = robot.getColorRobot();
    EXPECT_FLOAT_EQ(color.r(), 255.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.g(), 128.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.b(), 64.0f * COLOR_SCALE);
}

TEST_F(RobotTest, SetColorRobotFromFloats)
{
    robot.setColorRobot(1.0f, 0.5f, 0.25f);

    const Color color = robot.getColorRobot();
    EXPECT_FLOAT_EQ(color.r(), 1.0f);
    EXPECT_FLOAT_EQ(color.g(), 0.5f);
    EXPECT_FLOAT_EQ(color.b(), 0.25f);
}

TEST_F(RobotTest, SetColorLidarFromBytes)
{
    robot.setColorLidar(static_cast<uint8_t>(100), static_cast<uint8_t>(150), static_cast<uint8_t>(200));

    const Color color = robot.getColorLidar();
    EXPECT_FLOAT_EQ(color.r(), 100.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.g(), 150.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.b(), 200.0f * COLOR_SCALE);
}

TEST_F(RobotTest, SetColorPathFromBytes)
{
    robot.setColorPath(static_cast<uint8_t>(50), static_cast<uint8_t>(100), static_cast<uint8_t>(150));

    const Color color = robot.getColorPath();
    EXPECT_FLOAT_EQ(color.r(), 50.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.g(), 100.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.b(), 150.0f * COLOR_SCALE);
}

// ============================================================================
// Move Tests
// ============================================================================

TEST_F(RobotTest, MoveIntegratesActuatorVelocity)
{
    robot.setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);

    robot.move(TIME_STEP);

    // The actuator ramps from rest at ACCELERATION_LINEAR, so after one step
    // the velocity is a * dt and the robot has travelled a * dt * dt
    const float velocity = ACCELERATION_LINEAR * TIME_STEP;
    EXPECT_NEAR(robot.getActuatorVelocity().x(), velocity, EPSILON);
    EXPECT_NEAR(robot.getPosition().x(), velocity * TIME_STEP, EPSILON);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getRotationAngle(), 0.0f);
}

TEST_F(RobotTest, MoveAppliesVelocityInRobotFrame)
{
    robot.setPose(0.0f, 0.0f, PI_2); // facing +y
    robot.setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);

    robot.move(TIME_STEP);

    const float displacement = ACCELERATION_LINEAR * TIME_STEP * TIME_STEP;
    EXPECT_NEAR(robot.getPosition().x(), 0.0f, EPSILON);
    EXPECT_NEAR(robot.getPosition().y(), displacement, EPSILON);
}

TEST_F(RobotTest, MoveAccumulatesTraveledDistance)
{
    robot.setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);

    robot.move(TIME_STEP);
    robot.move(TIME_STEP);

    // Velocities after the two steps: a*dt and 2*a*dt
    const float distance = 3.0f * ACCELERATION_LINEAR * TIME_STEP * TIME_STEP;
    EXPECT_NEAR(robot.getTraveledDistance(), distance, EPSILON);
    EXPECT_NEAR(robot.getPosition().x(), distance, EPSILON);
}

TEST_F(RobotTest, MoveZeroDtLeavesPose)
{
    robot.setPose(1.0f, 2.0f, 0.5f);
    robot.setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);

    robot.move(0.0f);

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 2.0f);
    EXPECT_FLOAT_EQ(robot.getTraveledDistance(), 0.0f);
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(RobotTest, ResetZeroesActuatorAndTraveledDistance)
{
    robot.setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);
    robot.move(TIME_STEP);

    robot.reset();

    EXPECT_FLOAT_EQ(robot.getActuatorVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getActuatorCommandVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(robot.getTraveledDistance(), 0.0f);
}

// A reset may teleport the robot, so its lidars must not keep the old scan
TEST_F(RobotTest, ResetClearsTheLidars)
{
    robot.getLidarAt(0).getArrayLaserDistanceBuffer()[0] = 1.5f;
    robot.getLidarAt(0).publishScan();

    robot.reset();

    EXPECT_EQ(robot.getLidarAt(0).getLaserDistance(0), std::numeric_limits<float>::infinity());
    EXPECT_EQ(robot.getLidarAt(0).getArrayLaserDistanceBuffer()[0], std::numeric_limits<float>::infinity());
}

TEST_F(RobotTest, ResetClearsStatusStatesAndCounters)
{
    robot.setStatusCollision(Robot::STATUS_COLLISION::WORLD);
    robot.setStatusState(Robot::STATUS_STATE::ON_GOAL);
    robot.setSimulatedStepCount(100);
    robot.makeState();
    robot.finished();

    robot.reset();

    EXPECT_EQ(robot.getStatusCollision(), Robot::STATUS_COLLISION::NONE);
    EXPECT_EQ(robot.getStatusState(), Robot::STATUS_STATE::IDLE);
    EXPECT_EQ(robot.getSimulatedStepCount(), static_cast<size_t>(0));
    EXPECT_FALSE(robot.hasStates());
    EXPECT_FALSE(robot.isFinished());
}

// Regression: reset() seeded the timer with -1 s, which silently extended
// the required dwell time by one second for robots that start on their goal
// and broke the persistent-timer latch.
TEST_F(RobotTest, ResetInitialisesTimeOnGoalToZero)
{
    robot.setTimeOnGoal(5.0f);

    robot.reset();

    EXPECT_FLOAT_EQ(robot.getTimeOnGoal(), 0.0f);
}

TEST_F(RobotTest, ResetKeepsPoseAndGoal)
{
    robot.setPose(5.0f, 5.0f, 1.0f);
    robot.setGoal(3.0f, 4.0f, 0.5f);

    robot.reset();

    EXPECT_FLOAT_EQ(robot.getPosition().x(), 5.0f);
    EXPECT_FLOAT_EQ(robot.getPosition().y(), 5.0f);
    EXPECT_NEAR(robot.getRotationAngle(), 1.0f, EPSILON);
    EXPECT_FLOAT_EQ(robot.getGoal().getX(), 3.0f);
    EXPECT_FLOAT_EQ(robot.getGoal().getY(), 4.0f);
}

// ============================================================================
// Counter Tests
// ============================================================================

TEST_F(RobotTest, SetSimulatedStepCount)
{
    robot.setSimulatedStepCount(100);

    EXPECT_EQ(robot.getSimulatedStepCount(), static_cast<size_t>(100));
}

TEST_F(RobotTest, SetTimeOnGoal)
{
    robot.setTimeOnGoal(5.0f);

    EXPECT_FLOAT_EQ(robot.getTimeOnGoal(), 5.0f);
}

TEST_F(RobotTest, FinishedMarksRobot)
{
    robot.finished();

    EXPECT_TRUE(robot.isFinished());
}

// ============================================================================
// Stream Operator Tests
// ============================================================================

TEST_F(RobotTest, StreamOperatorPrintsPoseGoalAndStatus)
{
    robot.setPose(1.0f, 2.0f, 0.0f);
    robot.setGoal(3.0f, 4.0f, 0.5f);
    robot.setStatusState(Robot::STATUS_STATE::SEARCH_GOAL);
    robot.setStatusCollision(Robot::STATUS_COLLISION::WORLD);

    std::ostringstream stream;
    stream << robot;

    const std::string output = stream.str();
    EXPECT_NE(output.find("robot:"), std::string::npos);
    EXPECT_NE(output.find("position x:      1  y:      2"), std::string::npos);
    EXPECT_NE(output.find("goal x:      3  y:      4  r:    0.5"), std::string::npos);
    EXPECT_NE(output.find("ss: SEARCH_GOAL"), std::string::npos);
    EXPECT_NE(output.find("sc: WORLD"), std::string::npos);
}
