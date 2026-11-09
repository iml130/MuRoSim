// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/robot/robot_builder.hpp"
#include "murosim/robot/robot.hpp"
#include "murosim/math/constants.hpp"
#include <memory>
#include <stdexcept>
#include <utility>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::actuator::Actuator;
using murosim::actuator::DifferentialSimple;
using murosim::geometry::Shape;
using murosim::robot::Robot;
using murosim::robot::RobotBuilder;
using murosim::sensor::Lidar;

constexpr float EPSILON = 1e-5f;

// Explicit actuator limits, distinct from the class defaults (1 m/s, pi rad/s)
// so a test that gets the defaults instead fails
constexpr float VELOCITY_X_MIN = -0.5f;
constexpr float VELOCITY_X_MAX = 2.0f;
constexpr float VELOCITY_Y_MAX = 0.5f;
constexpr float VELOCITY_ANGULAR_MAX = PI_2;
constexpr float ACCELERATION_LINEAR = 3.0f;
constexpr float ACCELERATION_ANGULAR = 4.0f;

constexpr float BODY_WIDTH = 0.4f;
constexpr float BODY_HEIGHT = 0.6f;
constexpr float BODY_RADIUS = 0.15f;

constexpr float LIDAR_FOV = 1.5f * PI;
constexpr size_t LIDAR_RAYS = 360;
constexpr float LIDAR_RANGE_MIN = 0.05f;
constexpr float LIDAR_RANGE_MAX = 10.0f;
constexpr float LIDAR_FREQUENCY = 20.0f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class RobotBuilderTest : public ::testing::Test
{
protected:
    RobotBuilder builder;
};

// ============================================================================
// Build Validation Tests
// ============================================================================

TEST_F(RobotBuilderTest, BuildThrowsWhenEmpty)
{
    EXPECT_THROW(builder.build(), std::runtime_error);
}

TEST_F(RobotBuilderTest, BuildThrowsWithoutActuator)
{
    builder.setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    EXPECT_THROW(builder.build(), std::runtime_error);
}

TEST_F(RobotBuilderTest, BuildThrowsWithoutShape)
{
    builder.setActuatorDifferentialSimple();

    EXPECT_THROW(builder.build(), std::runtime_error);
}

// ============================================================================
// Build Tests
// ============================================================================

TEST_F(RobotBuilderTest, BuildWithMinimalConfigReturnsRobot)
{
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    ASSERT_NE(robot, nullptr);
    EXPECT_EQ(robot->getName(), ""); // no name set
    EXPECT_FALSE(robot->hasLidar());
}

TEST_F(RobotBuilderTest, BuildSetsName)
{
    builder.setName("my_robot").setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getName(), "my_robot");
}

TEST_F(RobotBuilderTest, BuildTwiceYieldsDistinctRobots)
{
    builder.setName("bot").setActuatorHolonomicSimple().setShapeRobotCircle(BODY_RADIUS);

    const std::shared_ptr<Robot> robot1 = builder.build();
    const std::shared_ptr<Robot> robot2 = builder.build();

    ASSERT_NE(robot1, nullptr);
    ASSERT_NE(robot2, nullptr);
    EXPECT_NE(robot1.get(), robot2.get());
    EXPECT_EQ(robot1->getName(), "bot");
    EXPECT_EQ(robot2->getName(), "bot");
}

TEST_F(RobotBuilderTest, BuiltRobotsAreIndependent)
{
    builder.setActuatorHolonomicSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);
    const std::shared_ptr<Robot> robot1 = builder.build();
    const std::shared_ptr<Robot> robot2 = builder.build();

    robot1->setPose(5.0f, 6.0f, 0.0f);
    robot2->setPose(10.0f, 20.0f, 1.0f);

    EXPECT_FLOAT_EQ(robot1->getPosition().x(), 5.0f);
    EXPECT_FLOAT_EQ(robot1->getPosition().y(), 6.0f);
    EXPECT_FLOAT_EQ(robot2->getPosition().x(), 10.0f);
    EXPECT_FLOAT_EQ(robot2->getPosition().y(), 20.0f);
}

// ============================================================================
// Actuator Tests
// ============================================================================

TEST_F(RobotBuilderTest, SetActuatorDifferentialSimpleDefault)
{
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().x(), -1.0f); // class defaults
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), 1.0f);
    EXPECT_NEAR(robot->getActuatorVelocityMax().z(), PI, EPSILON);
}

TEST_F(RobotBuilderTest, SetActuatorDifferentialSimpleWithVelocity)
{
    builder.setActuatorDifferentialSimple(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX)
        .setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().z(), VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().x(), 0.0f); // no limit
}

TEST_F(RobotBuilderTest, SetActuatorDifferentialSimpleWithAcceleration)
{
    builder
        .setActuatorDifferentialSimple(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX, ACCELERATION_LINEAR,
                                       ACCELERATION_ANGULAR)
        .setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().z(), ACCELERATION_ANGULAR);
}

TEST_F(RobotBuilderTest, SetActuatorHolonomicSimpleDefault)
{
    builder.setActuatorHolonomicSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), 1.0f); // class defaults
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().y(), 1.0f);
    EXPECT_NEAR(robot->getActuatorVelocityMax().z(), PI, EPSILON);
}

TEST_F(RobotBuilderTest, SetActuatorHolonomicSimpleWithVelocity)
{
    builder.setActuatorHolonomicSimple(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX)
        .setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().z(), VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().x(), 0.0f); // no limit
}

TEST_F(RobotBuilderTest, SetActuatorHolonomicSimpleWithAcceleration)
{
    builder
        .setActuatorHolonomicSimple(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                                    ACCELERATION_LINEAR, ACCELERATION_ANGULAR)
        .setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().y(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(robot->getActuatorAccelerationMax().z(), ACCELERATION_ANGULAR);
}

TEST_F(RobotBuilderTest, SetActuatorTakesOwnership)
{
    std::unique_ptr<Actuator> actuator =
        std::make_unique<DifferentialSimple>(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX);

    builder.setActuator(std::move(actuator)).setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();
    EXPECT_EQ(actuator, nullptr); // moved into the builder
    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_ANGULAR);
    EXPECT_FLOAT_EQ(robot->getActuatorVelocityMax().x(), VELOCITY_X_MAX);
}

TEST_F(RobotBuilderTest, LastActuatorWins)
{
    builder.setActuatorDifferentialSimple().setActuatorHolonomicSimple().setShapeRobotRectangle(BODY_WIDTH,
                                                                                                 BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
}

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(RobotBuilderTest, SetShapeRobotRectangle)
{
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    const Shape &shape = robot->getShapeRobot();
    EXPECT_EQ(shape.getLineSize(), static_cast<size_t>(4));
    EXPECT_EQ(shape.getCircleSize(), static_cast<size_t>(0));
}

TEST_F(RobotBuilderTest, SetShapeRobotCircle)
{
    builder.setActuatorDifferentialSimple().setShapeRobotCircle(BODY_RADIUS);

    const std::shared_ptr<Robot> robot = builder.build();

    const Shape &shape = robot->getShapeRobot();
    EXPECT_EQ(shape.getLineSize(), static_cast<size_t>(0));
    ASSERT_EQ(shape.getCircleSize(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(shape.getCircleAt(0).getRadius(), BODY_RADIUS);
}

TEST_F(RobotBuilderTest, SetShapeRobotCopiesCustomShape)
{
    Shape triangle;
    triangle.addLine(0.0f, 0.0f, 1.0f, 0.0f);
    triangle.addLine(1.0f, 0.0f, 0.5f, 1.0f);
    triangle.addLine(0.5f, 1.0f, 0.0f, 0.0f);

    builder.setActuatorDifferentialSimple().setShapeRobot(triangle);

    const std::shared_ptr<Robot> robot = builder.build();
    EXPECT_EQ(robot->getShapeRobot().getLineSize(), static_cast<size_t>(3));
    EXPECT_EQ(triangle.getLineSize(), static_cast<size_t>(3)); // source untouched
}

TEST_F(RobotBuilderTest, SetShapeRobotMovesCustomShape)
{
    Shape segment;
    segment.addLine(0.0f, 0.0f, 1.0f, 0.0f);

    builder.setActuatorDifferentialSimple().setShapeRobot(std::move(segment));

    const std::shared_ptr<Robot> robot = builder.build();
    EXPECT_EQ(robot->getShapeRobot().getLineSize(), static_cast<size_t>(1));
}

TEST_F(RobotBuilderTest, SetShapeLidar)
{
    Shape lidar_shape;
    lidar_shape.addCircle(0.0f, 0.0f, 0.05f);

    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT).setShapeLidar(lidar_shape);

    const std::shared_ptr<Robot> robot = builder.build();
    EXPECT_EQ(robot->getShapeLidar().getCircleSize(), static_cast<size_t>(1));
}

TEST_F(RobotBuilderTest, ShapeLidarDefaultsToRobotShape)
{
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    ASSERT_EQ(robot->getShapeLidar().getLineSize(), static_cast<size_t>(4));
    EXPECT_EQ(robot->getShapeLidar().getCircleSize(), static_cast<size_t>(0));
    for (size_t i = 0; i < robot->getShapeLidar().getLineSize(); ++i)
    {
        EXPECT_EQ(robot->getShapeLidar().getLineAt(i), robot->getShapeRobot().getLineAt(i));
    }
}

TEST_F(RobotBuilderTest, ExplicitlyEmptyShapeLidarStaysEmpty)
{
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT).setShapeLidar(Shape());

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_EQ(robot->getShapeLidar().getLineSize(), static_cast<size_t>(0));
    EXPECT_EQ(robot->getShapeLidar().getCircleSize(), static_cast<size_t>(0));
}

TEST_F(RobotBuilderTest, ResetRestoresTheShapeLidarDefault)
{
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT).setShapeLidar(Shape());
    builder.reset();
    builder.setActuatorDifferentialSimple().setShapeRobotCircle(BODY_RADIUS);

    EXPECT_EQ(builder.build()->getShapeLidar().getCircleSize(), static_cast<size_t>(1));
}

// ============================================================================
// Lidar Tests
// ============================================================================

TEST_F(RobotBuilderTest, BuildWithoutLidar)
{
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_FALSE(robot->hasLidar());
    EXPECT_EQ(robot->getLidarSize(), static_cast<size_t>(0));
}

TEST_F(RobotBuilderTest, AddLidarByParameters)
{
    builder.setActuatorDifferentialSimple()
        .setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT)
        .addLidar(Eigen::Affine2f::Identity(), LIDAR_FOV, LIDAR_RAYS, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX,
                  LIDAR_FREQUENCY);

    const std::shared_ptr<Robot> robot = builder.build();

    EXPECT_TRUE(robot->hasLidar());
    ASSERT_EQ(robot->getLidarSize(), static_cast<size_t>(1));
    const Lidar &lidar = robot->getLidarAt(0);
    EXPECT_EQ(lidar.getLaserSize(), LIDAR_RAYS);
    EXPECT_FLOAT_EQ(lidar.getRangeMin(), LIDAR_RANGE_MIN);
    EXPECT_FLOAT_EQ(lidar.getRangeMax(), LIDAR_RANGE_MAX);
    EXPECT_FLOAT_EQ(lidar.getFrequency(), LIDAR_FREQUENCY);
}

TEST_F(RobotBuilderTest, AddLidarByObject)
{
    const Lidar lidar(Eigen::Affine2f::Identity(), PI, 100, 0.0f, 5.0f, LIDAR_FREQUENCY);

    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT).addLidar(lidar);

    const std::shared_ptr<Robot> robot = builder.build();
    ASSERT_EQ(robot->getLidarSize(), static_cast<size_t>(1));
    EXPECT_EQ(robot->getLidarAt(0).getLaserSize(), static_cast<size_t>(100));
    EXPECT_FLOAT_EQ(robot->getLidarAt(0).getRangeMax(), 5.0f);
}

TEST_F(RobotBuilderTest, AddMultipleLidarsKeepsOrder)
{
    builder.setActuatorDifferentialSimple()
        .setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT)
        .addLidar(Eigen::Affine2f::Identity(), PI, 100, 0.0f, 10.0f, LIDAR_FREQUENCY)
        .addLidar(Eigen::Affine2f::Identity(), PI_2, 50, 0.0f, 5.0f, LIDAR_FREQUENCY);

    const std::shared_ptr<Robot> robot = builder.build();

    ASSERT_EQ(robot->getLidarSize(), static_cast<size_t>(2));
    EXPECT_EQ(robot->getLidarAt(0).getLaserSize(), static_cast<size_t>(100));
    EXPECT_EQ(robot->getLidarAt(1).getLaserSize(), static_cast<size_t>(50));
}

// ============================================================================
// Chaining Tests
// ============================================================================

TEST_F(RobotBuilderTest, FluentChainingBuildsRobot)
{
    const std::shared_ptr<Robot> robot =
        builder.setName("chained")
            .setActuatorHolonomicSimple(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX)
            .setShapeRobotCircle(BODY_RADIUS)
            .addLidar(Eigen::Affine2f::Identity(), LIDAR_FOV, LIDAR_RAYS, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX,
                      LIDAR_FREQUENCY)
            .build();

    EXPECT_EQ(robot->getName(), "chained");
    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
    EXPECT_EQ(robot->getShapeRobot().getCircleSize(), static_cast<size_t>(1));
    EXPECT_EQ(robot->getLidarSize(), static_cast<size_t>(1));
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(RobotBuilderTest, ResetClearsConfiguration)
{
    builder.setName("before_reset")
        .setActuatorDifferentialSimple()
        .setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT)
        .addLidar(Eigen::Affine2f::Identity(), PI, 100, 0.0f, 10.0f, LIDAR_FREQUENCY);
    EXPECT_NO_THROW(builder.build());

    builder.reset();

    EXPECT_THROW(builder.build(), std::runtime_error); // actuator and shape gone
}

TEST_F(RobotBuilderTest, ResetAllowsReconfiguration)
{
    builder.setName("first").setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);
    const std::shared_ptr<Robot> robot1 = builder.build();

    builder.reset();
    builder.setName("second").setActuatorHolonomicSimple().setShapeRobotCircle(BODY_RADIUS);
    const std::shared_ptr<Robot> robot2 = builder.build();

    EXPECT_EQ(robot1->getName(), "first");
    EXPECT_EQ(robot1->getActuatorType(), Actuator::TYPE::X_ANGULAR);
    EXPECT_EQ(robot1->getShapeRobot().getLineSize(), static_cast<size_t>(4));
    EXPECT_EQ(robot2->getName(), "second");
    EXPECT_EQ(robot2->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
    EXPECT_EQ(robot2->getShapeRobot().getCircleSize(), static_cast<size_t>(1));
}

TEST_F(RobotBuilderTest, ResetClearsName)
{
    builder.setName("named").setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    builder.reset();
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    EXPECT_EQ(builder.build()->getName(), "");
}

// ============================================================================
// Move Semantics Tests
// ============================================================================

TEST_F(RobotBuilderTest, MoveConstructorKeepsConfiguration)
{
    builder.setName("movable").setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_WIDTH, BODY_HEIGHT);

    const RobotBuilder moved(std::move(builder));

    const std::shared_ptr<Robot> robot = moved.build();
    EXPECT_EQ(robot->getName(), "movable");
    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_ANGULAR);
}

TEST_F(RobotBuilderTest, MoveAssignmentKeepsConfiguration)
{
    builder.setName("assigned").setActuatorHolonomicSimple().setShapeRobotCircle(BODY_RADIUS);
    RobotBuilder other;

    other = std::move(builder);

    const std::shared_ptr<Robot> robot = other.build();
    EXPECT_EQ(robot->getName(), "assigned");
    EXPECT_EQ(robot->getActuatorType(), Actuator::TYPE::X_Y_ANGULAR);
}
