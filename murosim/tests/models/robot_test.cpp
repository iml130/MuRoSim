// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/models/robot.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <memory>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::actuator::Actuator;
using murosim::robot::Robot;
using murosim::sensor::Lidar;

constexpr float EPSILON = 1e-5f;

// Limits requested from the factories; all within the platforms' ranges
constexpr float VELOCITY_X_MIN = -0.2f;
constexpr float VELOCITY_X_MAX = 1.0f;
constexpr float VELOCITY_Y_MAX = 0.8f;
constexpr float VELOCITY_ANGULAR_MAX = 1.0f;
constexpr float ACCELERATION_LINEAR = 1.0f;
constexpr float ACCELERATION_ANGULAR = 1.0f;

// MiR100 platform limits the factory clamps to
constexpr float MIR_VELOCITY_LIMIT = 1.5f;
constexpr float MIR_REVERSE_LIMIT = 0.3f;
constexpr float MIR_ANGULAR_LIMIT = PI_2;
constexpr float MIR_ACCELERATION_LINEAR = 1.5f;
constexpr float MIR_ACCELERATION_ANGULAR = PI_2;
constexpr size_t MIR_LIDAR_RAYS = 541;
constexpr float MIR_LIDAR_RANGE_MAX = 29.0f;
constexpr float MIR_LIDAR_FREQUENCY = 12.5f;
constexpr float MIR_LIDAR_NOISE = 0.009f; // SICK S300 random error, 1 sigma

// RoboMaster body: 0.32 m x 0.24 m
constexpr float RM_HALF_LENGTH = 0.16f;
constexpr float RM_HALF_WIDTH = 0.12f;
constexpr float RM_LIDAR_X = 0.1335f;
constexpr size_t RM_LIDAR_RAYS = 1200;
constexpr float RM_LIDAR_RANGE_MAX = 30.0f;
constexpr float RM_LIDAR_FREQUENCY = 10.0f;
constexpr float RM_LIDAR_NOISE = 0.0125f;

constexpr float LIDAR_FOV = 1.5f * PI; // 270 degrees on both platforms
constexpr float LIDAR_RANGE_MIN = 0.05f;

std::shared_ptr<Robot> robomaster(const bool holonomic)
{
    return murosim::models::robot::robomaster(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                                              ACCELERATION_LINEAR, ACCELERATION_ANGULAR, holonomic);
}
} // namespace

// ============================================================================
// RoboMaster Tests
// ============================================================================

TEST(ModelsRobotRoboMaster, HolonomicUsesMecanumActuator)
{
    const std::shared_ptr<Robot> robot = robomaster(true);

    ASSERT_NE(robot, nullptr);
    EXPECT_EQ(robot->getName(), "robomaster");
    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
}

TEST(ModelsRobotRoboMaster, NonHolonomicUsesDifferentialActuator)
{
    const std::shared_ptr<Robot> robot = robomaster(false);

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_ANGULAR);
}

TEST(ModelsRobotRoboMaster, AppliesRequestedLimits)
{
    const std::shared_ptr<Robot> robot = robomaster(true);

    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().z(), VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().z(), ACCELERATION_ANGULAR);
}

TEST(ModelsRobotRoboMaster, BodyIsTheRectangle)
{
    const std::shared_ptr<Robot> robot = robomaster(true);

    EXPECT_EQ(robot->getShapeRobot().getLineSize(), static_cast<size_t>(4));
    EXPECT_EQ(robot->getShapeRobot().getCircleSize(), static_cast<size_t>(0));
    EXPECT_NEAR(robot->getCollisionRadius(), std::hypot(RM_HALF_LENGTH, RM_HALF_WIDTH), EPSILON);
}

TEST(ModelsRobotRoboMaster, LidarShapeIsHousingAndSensorDisk)
{
    const std::shared_ptr<Robot> robot = robomaster(true);

    EXPECT_EQ(robot->getShapeLidar().getLineSize(), static_cast<size_t>(4));
    ASSERT_EQ(robot->getShapeLidar().getCircleSize(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(robot->getShapeLidar().getCircleAt(0).getX(), RM_LIDAR_X);
}

TEST(ModelsRobotRoboMaster, HasOneFrontLidar)
{
    const std::shared_ptr<Robot> robot = robomaster(true);

    ASSERT_EQ(robot->getLidarSize(), static_cast<size_t>(1));
    const Lidar &lidar = robot->getLidarAt(0);
    EXPECT_EQ(lidar.getLaserSize(), RM_LIDAR_RAYS);
    EXPECT_FLOAT_EQ(lidar.getRangeMin(), LIDAR_RANGE_MIN);
    EXPECT_FLOAT_EQ(lidar.getRangeMax(), RM_LIDAR_RANGE_MAX);
    EXPECT_FLOAT_EQ(lidar.getFrequency(), RM_LIDAR_FREQUENCY);
    EXPECT_EQ(lidar.getNoiseType(), Lidar::NOISE_TYPE::NORMAL);
    EXPECT_FLOAT_EQ(lidar.getNoise(), RM_LIDAR_NOISE);
    EXPECT_TRUE(lidar.getRotationClockwise());
    EXPECT_FLOAT_EQ(lidar.getPose().translation().x(), RM_LIDAR_X);
    EXPECT_FLOAT_EQ(lidar.getPose().translation().y(), 0.0f);
    EXPECT_NEAR(std::abs(lidar.getAngleMin() - lidar.getAngleMax()), LIDAR_FOV, EPSILON);
}

TEST(ModelsRobotRoboMaster, EveryCallBuildsANewRobot)
{
    const std::shared_ptr<Robot> first = robomaster(true);
    const std::shared_ptr<Robot> second = robomaster(true);

    EXPECT_NE(first, second);
}

// ============================================================================
// MiR100 Tests
// ============================================================================

TEST(ModelsRobotMir100, DefaultsToPlatformLimits)
{
    const std::shared_ptr<Robot> robot = murosim::models::robot::mir100();

    ASSERT_NE(robot, nullptr);
    EXPECT_EQ(robot->getName(), "mir100");
    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().x(), -MIR_REVERSE_LIMIT);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), MIR_VELOCITY_LIMIT);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().z(), MIR_ANGULAR_LIMIT);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().x(), MIR_ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().z(), MIR_ACCELERATION_ANGULAR);
}

TEST(ModelsRobotMir100, AppliesLimitsWithinPlatformRange)
{
    const std::shared_ptr<Robot> robot =
        murosim::models::robot::mir100(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().z(), VELOCITY_ANGULAR_MAX);
}

TEST(ModelsRobotMir100, ClampsLimitsToPlatformRange)
{
    const std::shared_ptr<Robot> robot = murosim::models::robot::mir100(-5.0f, 5.0f, 10.0f);

    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().x(), -MIR_REVERSE_LIMIT);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), MIR_VELOCITY_LIMIT);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().z(), MIR_ANGULAR_LIMIT);
}

TEST(ModelsRobotMir100, NegativeAngularLimitUsesItsMagnitude)
{
    const std::shared_ptr<Robot> robot = murosim::models::robot::mir100(-1.0f, 1.0f, -1.0f);

    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().z(), 1.0f);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().z(), -1.0f);
}

TEST(ModelsRobotMir100, BodyIsTheRectangleWithHeadingMarker)
{
    const std::shared_ptr<Robot> robot = murosim::models::robot::mir100();

    EXPECT_EQ(robot->getShapeRobot().getLineSize(), static_cast<size_t>(6)); // four sides plus the arrow
    EXPECT_EQ(robot->getShapeRobot().getCircleSize(), static_cast<size_t>(0));
    EXPECT_NEAR(robot->getCollisionRadius(), std::hypot(0.45f, 0.30f), EPSILON);
}

TEST(ModelsRobotMir100, LidarShapeHasTwoSensorDisks)
{
    const std::shared_ptr<Robot> robot = murosim::models::robot::mir100();

    EXPECT_EQ(robot->getShapeLidar().getLineSize(), static_cast<size_t>(4));
    EXPECT_EQ(robot->getShapeLidar().getCircleSize(), static_cast<size_t>(2));
}

TEST(ModelsRobotMir100, HasTwoDiagonallyOpposedLidars)
{
    const std::shared_ptr<Robot> robot = murosim::models::robot::mir100();

    ASSERT_EQ(robot->getLidarSize(), static_cast<size_t>(2));
    const Lidar &front = robot->getLidarAt(0);
    const Lidar &rear = robot->getLidarAt(1);

    // Front left corner looking outwards at 45 degrees, rear right corner
    // looking the opposite way
    EXPECT_FLOAT_EQ(front.getPose().translation().x(), 0.32f);
    EXPECT_FLOAT_EQ(front.getPose().translation().y(), 0.21f);
    EXPECT_NEAR(Eigen::Rotation2Df(front.getPose().rotation()).angle(), PI_4, EPSILON);
    EXPECT_FLOAT_EQ(rear.getPose().translation().x(), -0.32f);
    EXPECT_FLOAT_EQ(rear.getPose().translation().y(), -0.21f);
    EXPECT_NEAR(Eigen::Rotation2Df(rear.getPose().rotation()).angle(), PI_4 - PI, EPSILON); // 225 wrapped

    for (size_t i = 0; i < robot->getLidarSize(); i++)
    {
        const Lidar &lidar = robot->getLidarAt(i);
        EXPECT_EQ(lidar.getLaserSize(), MIR_LIDAR_RAYS) << "lidar " << i;
        EXPECT_FLOAT_EQ(lidar.getRangeMin(), LIDAR_RANGE_MIN) << "lidar " << i;
        EXPECT_FLOAT_EQ(lidar.getRangeMax(), MIR_LIDAR_RANGE_MAX) << "lidar " << i;
        EXPECT_FLOAT_EQ(lidar.getFrequency(), MIR_LIDAR_FREQUENCY) << "lidar " << i;
        EXPECT_EQ(lidar.getNoiseType(), Lidar::NOISE_TYPE::NORMAL) << "lidar " << i;
        EXPECT_FLOAT_EQ(lidar.getNoise(), MIR_LIDAR_NOISE) << "lidar " << i;
        EXPECT_FALSE(lidar.getRotationClockwise()) << "lidar " << i;
        EXPECT_NEAR(lidar.getAngleMax() - lidar.getAngleMin(), LIDAR_FOV, EPSILON) << "lidar " << i;
    }
}
