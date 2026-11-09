// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/math/constants.hpp"
#include <limits>
#include <stdexcept>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
constexpr float EPSILON = 1e-5f;
constexpr float TIME_STEP = 0.1f;

// Limits of the fixture actuator. Min and max differ on purpose so a test
// that confuses them fails.
constexpr float VELOCITY_X_MIN = -0.5f;
constexpr float VELOCITY_X_MAX = 2.0f;
constexpr float VELOCITY_Y_MAX = 1.0f;
constexpr float VELOCITY_ANGULAR_MAX = PI_2;
constexpr float ACCELERATION_LINEAR = 0.5f;
constexpr float ACCELERATION_ANGULAR = PI_4;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(HolonomicSimpleConstructor, DefaultConstructor)
{
    murosim::actuator::HolonomicSimple actuator;

    // Class defaults: 1 m/s linear on both axes, pi rad/s angular
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), -1.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), 1.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), -1.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), 1.0f);
    EXPECT_NEAR(actuator.getVelocityMin().z(), -PI, EPSILON);
    EXPECT_NEAR(actuator.getVelocityMax().z(), PI, EPSILON);

    // Acceleration defaults to zero (no limit)
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);
}

TEST(HolonomicSimpleConstructor, VelocityLimitsConstructor)
{
    murosim::actuator::HolonomicSimple actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), -VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), VELOCITY_ANGULAR_MAX);

    // Acceleration defaults to zero (no limit)
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);
}

TEST(HolonomicSimpleConstructor, FullConstructor)
{
    murosim::actuator::HolonomicSimple actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                                                ACCELERATION_LINEAR, ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), -VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), VELOCITY_ANGULAR_MAX);

    // Both x and y use the same acceleration_linear parameter
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().y(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), ACCELERATION_ANGULAR);
}

TEST(HolonomicSimpleConstructor, NegativeAngularVelocityIsClamped)
{
    // Negative angular velocity max should be treated as zero
    murosim::actuator::HolonomicSimple actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, -1.0f);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), 0.0f);
}

TEST(HolonomicSimpleConstructor, NegativeAccelerationIsClamped)
{
    // Negative acceleration should be treated as zero
    murosim::actuator::HolonomicSimple actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                                                -1.0f, -1.0f);

    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);
}

// ============================================================================
// Test Fixture
// ============================================================================

class HolonomicSimpleTest : public ::testing::Test
{
protected:
    murosim::actuator::HolonomicSimple actuator{VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                                                ACCELERATION_LINEAR, ACCELERATION_ANGULAR};
};

// ============================================================================
// Type Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, TypeIsXYAngular)
{
    EXPECT_EQ(actuator.getType(), murosim::actuator::Actuator::TYPE::X_Y_ANGULAR);
    EXPECT_EQ(actuator.getTypeAsString(), "X_Y_ANGULAR");
}

// ============================================================================
// Initial State Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, InitialVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(HolonomicSimpleTest, InitialCommandVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), 0.0f);
}

TEST_F(HolonomicSimpleTest, InitialCommandAccelerationIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.0f);
}

// ============================================================================
// Command Velocity Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, SetCommandVelocityWithinLimits)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

TEST_F(HolonomicSimpleTest, SetCommandVelocityClampedToMax)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MAX, 2.0f * VELOCITY_Y_MAX, 2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), VELOCITY_ANGULAR_MAX);
}

TEST_F(HolonomicSimpleTest, SetCommandVelocityClampedToMin)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MIN, -2.0f * VELOCITY_Y_MAX, -2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), -VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), -VELOCITY_ANGULAR_MAX);
}

TEST_F(HolonomicSimpleTest, SetCommandVelocityWithVector)
{
    actuator.setCommandVelocity(Eigen::Vector3f(0.5f, 0.5f, PI_4));

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

// ============================================================================
// Command Acceleration Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, SetCommandAccelerationWithinLimits)
{
    actuator.setCommandAcceleration(0.3f, 0.3f, 0.5f);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.5f);
}

TEST_F(HolonomicSimpleTest, SetCommandAccelerationClampedToMax)
{
    actuator.setCommandAcceleration(2.0f * ACCELERATION_LINEAR, 2.0f * ACCELERATION_LINEAR,
                                    2.0f * ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), ACCELERATION_ANGULAR);
}

TEST_F(HolonomicSimpleTest, SetCommandAccelerationClampedToNegativeMax)
{
    actuator.setCommandAcceleration(-2.0f * ACCELERATION_LINEAR, -2.0f * ACCELERATION_LINEAR,
                                    -2.0f * ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), -ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), -ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), -ACCELERATION_ANGULAR);
}

TEST_F(HolonomicSimpleTest, SetCommandAccelerationWithVector)
{
    actuator.setCommandAcceleration(Eigen::Vector3f(0.3f, 0.3f, 0.5f));

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.5f);
}

// ============================================================================
// Step Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, StepAcceleratesLinearXVelocity)
{
    actuator.setCommandVelocity(VELOCITY_X_MAX, 0.0f, 0.0f);

    // Velocity ramps linearly with the acceleration limit
    for (int i = 1; i <= 10; ++i)
    {
        actuator.step(TIME_STEP);
        const float expected_velocity = static_cast<float>(i) * TIME_STEP * ACCELERATION_LINEAR;

        EXPECT_NEAR(actuator.getVelocity().x(), expected_velocity, EPSILON) << "step " << i;
        EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
        EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
    }
}

TEST_F(HolonomicSimpleTest, StepAcceleratesLinearYVelocity)
{
    actuator.setCommandVelocity(0.0f, VELOCITY_Y_MAX, 0.0f);

    // Velocity ramps linearly with the acceleration limit
    for (int i = 1; i <= 10; ++i)
    {
        actuator.step(TIME_STEP);
        const float expected_velocity = static_cast<float>(i) * TIME_STEP * ACCELERATION_LINEAR;

        EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
        EXPECT_NEAR(actuator.getVelocity().y(), expected_velocity, EPSILON) << "step " << i;
        EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
    }
}

TEST_F(HolonomicSimpleTest, StepAcceleratesAngularVelocity)
{
    actuator.setCommandVelocity(0.0f, 0.0f, VELOCITY_ANGULAR_MAX);

    // Velocity ramps linearly with the acceleration limit
    for (int i = 1; i <= 10; ++i)
    {
        actuator.step(TIME_STEP);
        const float expected_angular = static_cast<float>(i) * TIME_STEP * ACCELERATION_ANGULAR;

        EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
        EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
        EXPECT_NEAR(actuator.getVelocity().z(), expected_angular, EPSILON) << "step " << i;
    }
}

TEST_F(HolonomicSimpleTest, StepDeceleratesVelocity)
{
    // Reach a steady state on all axes (10 steps of ramp suffice, 20 are safe)
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);
    for (int i = 0; i < 20; ++i)
    {
        actuator.step(TIME_STEP);
    }
    ASSERT_NEAR(actuator.getVelocity().y(), 0.5f, EPSILON);

    // Reverse the y command: y decelerates with the same limit, x and angular keep their values
    actuator.setCommandVelocity(0.5f, -VELOCITY_Y_MAX, PI_4);

    for (int i = 1; i <= 10; ++i)
    {
        actuator.step(TIME_STEP);
        const float expected_y = 0.5f - static_cast<float>(i) * TIME_STEP * ACCELERATION_LINEAR;

        EXPECT_NEAR(actuator.getVelocity().x(), 0.5f, EPSILON);
        EXPECT_NEAR(actuator.getVelocity().y(), expected_y, EPSILON) << "step " << i;
        EXPECT_NEAR(actuator.getVelocity().z(), PI_4, EPSILON);
    }
}

TEST_F(HolonomicSimpleTest, StepReturnsTransformation)
{
    // Both axes ramp with the same limit; the y command is reached exactly at step 10
    actuator.setCommandVelocity(VELOCITY_X_MAX, 0.5f, 0.0f);

    Eigen::Affine2f pose = Eigen::Affine2f::Identity();
    float expected_distance = 0.0f;

    for (int i = 1; i <= 10; ++i)
    {
        pose = pose * actuator.step(TIME_STEP);

        // Each step translates by the velocity reached in that step times dt
        expected_distance += static_cast<float>(i) * TIME_STEP * ACCELERATION_LINEAR * TIME_STEP;
    }

    // Diagonal drive without rotation
    EXPECT_NEAR(pose.translation().x(), expected_distance, EPSILON);
    EXPECT_NEAR(pose.translation().y(), expected_distance, EPSILON);
    EXPECT_NEAR(Eigen::Rotation2Df(pose.rotation()).angle(), 0.0f, EPSILON);
}

TEST_F(HolonomicSimpleTest, StepWithoutAccelerationLimit)
{
    // Without acceleration limits the command velocity is reached in one step
    murosim::actuator::HolonomicSimple no_accel_actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX,
                                                         VELOCITY_ANGULAR_MAX);

    no_accel_actuator.setCommandVelocity(1.0f, 0.5f, PI_4);
    no_accel_actuator.step(TIME_STEP);

    EXPECT_FLOAT_EQ(no_accel_actuator.getVelocity().x(), 1.0f);
    EXPECT_FLOAT_EQ(no_accel_actuator.getVelocity().y(), 0.5f);
    EXPECT_FLOAT_EQ(no_accel_actuator.getVelocity().z(), PI_4);
}

TEST_F(HolonomicSimpleTest, StepZeroDtIsIdentityNoOp)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    const Eigen::Affine2f movement = actuator.step(0.0f);

    // No time elapsed: identity transform and velocity untouched
    EXPECT_TRUE(movement.isApprox(Eigen::Affine2f::Identity()));
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(HolonomicSimpleTest, StepNegativeOrNaNDtThrows)
{
    EXPECT_THROW(actuator.step(-0.1f), std::invalid_argument);
    EXPECT_THROW(actuator.step(std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, ResetClearsAllState)
{
    // Set some state
    actuator.setCommandVelocity(1.0f, 0.5f, PI_4);
    actuator.setCommandAcceleration(0.3f, 0.3f, 0.5f);
    actuator.step(TIME_STEP);
    actuator.step(TIME_STEP);

    actuator.reset();

    // All should be zero
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), 0.0f);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.0f);
}

// ============================================================================
// Clone Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, CloneCreatesIndependentCopy)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);
    actuator.step(TIME_STEP);

    auto clone = actuator.clone();

    // Clone should have same state
    EXPECT_FLOAT_EQ(clone->getVelocity().x(), actuator.getVelocity().x());
    EXPECT_FLOAT_EQ(clone->getVelocity().y(), actuator.getVelocity().y());
    EXPECT_FLOAT_EQ(clone->getVelocity().z(), actuator.getVelocity().z());
    EXPECT_EQ(clone->getType(), actuator.getType());

    // Modifying original should not affect clone
    actuator.reset();
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_NE(clone->getVelocity().x(), 0.0f);
}

// ============================================================================
// Getter Overload Tests (float& version)
// ============================================================================

TEST_F(HolonomicSimpleTest, GetVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);
    actuator.step(TIME_STEP);

    float x, y, angular;
    actuator.getVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, actuator.getVelocity().x());
    EXPECT_FLOAT_EQ(y, actuator.getVelocity().y());
    EXPECT_FLOAT_EQ(angular, actuator.getVelocity().z());
}

TEST_F(HolonomicSimpleTest, GetCommandVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    float x, y, angular;
    actuator.getCommandVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, 0.5f);
    EXPECT_FLOAT_EQ(y, 0.5f);
    EXPECT_FLOAT_EQ(angular, PI_4);
}

TEST_F(HolonomicSimpleTest, GetVelocityMinMaxFloatOverload)
{
    float x, y, angular;

    actuator.getVelocityMin(x, y, angular);
    EXPECT_FLOAT_EQ(x, VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(y, -VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(angular, -VELOCITY_ANGULAR_MAX);

    actuator.getVelocityMax(x, y, angular);
    EXPECT_FLOAT_EQ(x, VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(y, VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(angular, VELOCITY_ANGULAR_MAX);
}

TEST_F(HolonomicSimpleTest, GetAccelerationMaxFloatOverload)
{
    float x, y, angular;
    actuator.getAccelerationMax(x, y, angular);

    EXPECT_FLOAT_EQ(x, ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(y, ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(angular, ACCELERATION_ANGULAR);
}

// ============================================================================
// Control Mode Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, DefaultControlModeIsVelocity)
{
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::VELOCITY);
    EXPECT_EQ(actuator.getControlModeAsString(), "VELOCITY");
}

TEST_F(HolonomicSimpleTest, SetControlModeAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);

    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
    EXPECT_EQ(actuator.getControlModeAsString(), "ACCELERATION");
}

TEST_F(HolonomicSimpleTest, ControlModeFromString)
{
    using murosim::actuator::Actuator;
    using murosim::actuator::CONTROL_MODE;

    EXPECT_EQ(Actuator::getControlModeFromString("VELOCITY"), CONTROL_MODE::VELOCITY);
    EXPECT_EQ(Actuator::getControlModeFromString("velocity"), CONTROL_MODE::VELOCITY);
    EXPECT_EQ(Actuator::getControlModeFromString("ACCELERATION"), CONTROL_MODE::ACCELERATION);
    EXPECT_EQ(Actuator::getControlModeFromString("acceleration"), CONTROL_MODE::ACCELERATION);
}

TEST_F(HolonomicSimpleTest, ControlModeFromStringThrowsOnInvalid)
{
    using murosim::actuator::Actuator;

    EXPECT_THROW(Actuator::getControlModeFromString("INVALID"), std::runtime_error);
    // The removed VELOCITY_ACCELERATION mode must not silently parse anymore
    EXPECT_THROW(Actuator::getControlModeFromString("VELOCITY_ACCELERATION"), std::runtime_error);
}

TEST_F(HolonomicSimpleTest, ControlModePersistsAfterReset)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.reset();

    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
}

// ============================================================================
// Acceleration Control Mode Tests
// ============================================================================

TEST_F(HolonomicSimpleTest, AccelerationModeAppliesDirectAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandAcceleration(0.5f, 0.3f, 0.2f);

    // First step: velocity increases by acceleration * time
    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.05f, EPSILON); // 0.5 * 0.1
    EXPECT_NEAR(actuator.getVelocity().y(), 0.03f, EPSILON); // 0.3 * 0.1
    EXPECT_NEAR(actuator.getVelocity().z(), 0.02f, EPSILON); // 0.2 * 0.1

    // Second step: velocity increases further
    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.10f, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().y(), 0.06f, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), 0.04f, EPSILON);
}

TEST_F(HolonomicSimpleTest, AccelerationModeNegativeAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);

    // Accelerate forward for 0.5 s
    actuator.setCommandAcceleration(0.5f, 0.0f, 0.0f);
    for (int i = 0; i < 5; ++i)
    {
        actuator.step(TIME_STEP);
    }
    EXPECT_NEAR(actuator.getVelocity().x(), 0.25f, EPSILON);

    // Decelerate with the opposite acceleration for the same time
    actuator.setCommandAcceleration(-0.5f, 0.0f, 0.0f);
    for (int i = 0; i < 5; ++i)
    {
        actuator.step(TIME_STEP);
    }
    EXPECT_NEAR(actuator.getVelocity().x(), 0.0f, EPSILON);
}

TEST_F(HolonomicSimpleTest, AccelerationModeClampsVelocityToLimits)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandAcceleration(ACCELERATION_LINEAR, ACCELERATION_LINEAR, ACCELERATION_ANGULAR);

    // 10 s of constant acceleration would exceed every limit several times
    for (int i = 0; i < 100; ++i)
    {
        actuator.step(TIME_STEP);
    }

    EXPECT_NEAR(actuator.getVelocity().x(), VELOCITY_X_MAX, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().y(), VELOCITY_Y_MAX, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), VELOCITY_ANGULAR_MAX, EPSILON);
}

TEST_F(HolonomicSimpleTest, AccelerationModeClampsVelocityToMinLimits)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandAcceleration(-ACCELERATION_LINEAR, -ACCELERATION_LINEAR, -ACCELERATION_ANGULAR);

    // 10 s of constant acceleration would exceed every limit several times
    for (int i = 0; i < 100; ++i)
    {
        actuator.step(TIME_STEP);
    }

    EXPECT_NEAR(actuator.getVelocity().x(), VELOCITY_X_MIN, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().y(), -VELOCITY_Y_MAX, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), -VELOCITY_ANGULAR_MAX, EPSILON);
}

TEST_F(HolonomicSimpleTest, AccelerationModeIgnoresCommandVelocity)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandVelocity(1.0f, 1.0f, 1.0f);
    actuator.setCommandAcceleration(0.0f, 0.0f, 0.0f);

    actuator.step(TIME_STEP);

    // Velocity should remain zero since acceleration is zero
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}
