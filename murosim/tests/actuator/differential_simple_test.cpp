// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/actuator/differential_simple.hpp"
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
constexpr float VELOCITY_X_MAX = 1.0f;
constexpr float VELOCITY_ANGULAR_MAX = PI_2;
constexpr float ACCELERATION_LINEAR = 0.5f;
constexpr float ACCELERATION_ANGULAR = PI_4;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(DifferentialSimpleConstructor, DefaultConstructor)
{
    murosim::actuator::DifferentialSimple actuator;

    // Class defaults: 1 m/s linear, pi rad/s angular
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), -1.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), 1.0f);
    EXPECT_NEAR(actuator.getVelocityMin().z(), -PI, EPSILON);
    EXPECT_NEAR(actuator.getVelocityMax().z(), PI, EPSILON);

    // Y should be zero for differential drive
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), 0.0f);

    // Acceleration defaults to zero (no limit)
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);
}

TEST(DifferentialSimpleConstructor, VelocityLimitsConstructor)
{
    murosim::actuator::DifferentialSimple actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), VELOCITY_ANGULAR_MAX);

    // Acceleration defaults to zero (no limit)
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);
}

TEST(DifferentialSimpleConstructor, FullConstructor)
{
    murosim::actuator::DifferentialSimple actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX,
                                                   ACCELERATION_LINEAR, ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), ACCELERATION_ANGULAR);
}

TEST(DifferentialSimpleConstructor, NegativeAngularVelocityIsClamped)
{
    // Negative angular velocity max should be treated as zero
    murosim::actuator::DifferentialSimple actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, -1.0f);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), 0.0f);
}

// ============================================================================
// Test Fixture
// ============================================================================

class DifferentialSimpleTest : public ::testing::Test
{
protected:
    murosim::actuator::DifferentialSimple actuator{VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX,
                                                   ACCELERATION_LINEAR, ACCELERATION_ANGULAR};
};

// ============================================================================
// Type Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, TypeIsXAngular)
{
    EXPECT_EQ(actuator.getType(), murosim::actuator::Actuator::TYPE::X_ANGULAR);
    EXPECT_EQ(actuator.getTypeAsString(), "X_ANGULAR");
}

// ============================================================================
// Initial State Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, InitialVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(DifferentialSimpleTest, InitialCommandVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), 0.0f);
}

TEST_F(DifferentialSimpleTest, InitialCommandAccelerationIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.0f);
}

// ============================================================================
// Command Velocity Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, SetCommandVelocityWithinLimits)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

TEST_F(DifferentialSimpleTest, SetCommandVelocityClampedToMax)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MAX, 1.0f, 2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), VELOCITY_ANGULAR_MAX);
}

TEST_F(DifferentialSimpleTest, SetCommandVelocityClampedToMin)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MIN, -1.0f, -2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), -VELOCITY_ANGULAR_MAX);
}

TEST_F(DifferentialSimpleTest, SetCommandVelocityWithVector)
{
    actuator.setCommandVelocity(Eigen::Vector3f(0.5f, 0.5f, PI_4));

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

// ============================================================================
// Command Acceleration Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, SetCommandAccelerationWithinLimits)
{
    actuator.setCommandAcceleration(0.3f, 0.0f, 0.5f);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.5f);
}

TEST_F(DifferentialSimpleTest, SetCommandAccelerationClampedToMax)
{
    actuator.setCommandAcceleration(2.0f * ACCELERATION_LINEAR, 1.0f, 2.0f * ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), ACCELERATION_ANGULAR);
}

TEST_F(DifferentialSimpleTest, SetCommandAccelerationClampedToNegativeMax)
{
    actuator.setCommandAcceleration(-2.0f * ACCELERATION_LINEAR, -1.0f, -2.0f * ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), -ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), -ACCELERATION_ANGULAR);
}

TEST_F(DifferentialSimpleTest, SetCommandAccelerationWithVector)
{
    actuator.setCommandAcceleration(Eigen::Vector3f(0.3f, 0.3f, 0.5f));

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.5f);
}

// ============================================================================
// Step Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, StepAcceleratesLinearVelocity)
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

TEST_F(DifferentialSimpleTest, StepAcceleratesAngularVelocity)
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

TEST_F(DifferentialSimpleTest, StepReturnsTransformation)
{
    actuator.setCommandVelocity(VELOCITY_X_MAX, 0.0f, 0.0f);

    Eigen::Affine2f pose = Eigen::Affine2f::Identity();
    float expected_distance = 0.0f;

    for (int i = 1; i <= 10; ++i)
    {
        pose = pose * actuator.step(TIME_STEP);

        // Each step translates by the velocity reached in that step times dt
        expected_distance += static_cast<float>(i) * TIME_STEP * ACCELERATION_LINEAR * TIME_STEP;
    }

    // Straight forward: no rotation, no sideways motion
    EXPECT_NEAR(pose.translation().x(), expected_distance, EPSILON);
    EXPECT_NEAR(pose.translation().y(), 0.0f, EPSILON);
    EXPECT_NEAR(Eigen::Rotation2Df(pose.rotation()).angle(), 0.0f, EPSILON);
}

TEST_F(DifferentialSimpleTest, StepZeroDtIsIdentityNoOp)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);

    const Eigen::Affine2f movement = actuator.step(0.0f);

    // No time elapsed: identity transform and velocity untouched
    EXPECT_TRUE(movement.isApprox(Eigen::Affine2f::Identity()));
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(DifferentialSimpleTest, StepNegativeOrNaNDtThrows)
{
    EXPECT_THROW(actuator.step(-0.1f), std::invalid_argument);
    EXPECT_THROW(actuator.step(std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, ResetClearsAllState)
{
    // Set some state
    actuator.setCommandVelocity(1.0f, 0.0f, PI_4);
    actuator.setCommandAcceleration(0.3f, 0.0f, 0.5f);
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

TEST_F(DifferentialSimpleTest, CloneCreatesIndependentCopy)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);
    actuator.step(TIME_STEP);

    auto clone = actuator.clone();

    // Clone should have same state
    EXPECT_FLOAT_EQ(clone->getVelocity().x(), actuator.getVelocity().x());
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

TEST_F(DifferentialSimpleTest, GetVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);
    actuator.step(TIME_STEP);

    float x, y, angular;
    actuator.getVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, actuator.getVelocity().x());
    EXPECT_FLOAT_EQ(y, actuator.getVelocity().y());
    EXPECT_FLOAT_EQ(angular, actuator.getVelocity().z());
}

TEST_F(DifferentialSimpleTest, GetCommandVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);

    float x, y, angular;
    actuator.getCommandVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, 0.5f);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(angular, PI_4);
}

TEST_F(DifferentialSimpleTest, GetVelocityMinMaxFloatOverload)
{
    float x, y, angular;

    actuator.getVelocityMin(x, y, angular);
    EXPECT_FLOAT_EQ(x, VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(angular, -VELOCITY_ANGULAR_MAX);

    actuator.getVelocityMax(x, y, angular);
    EXPECT_FLOAT_EQ(x, VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(angular, VELOCITY_ANGULAR_MAX);
}

TEST_F(DifferentialSimpleTest, GetAccelerationMaxFloatOverload)
{
    float x, y, angular;
    actuator.getAccelerationMax(x, y, angular);

    EXPECT_FLOAT_EQ(x, ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(angular, ACCELERATION_ANGULAR);
}

// ============================================================================
// Control Mode Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, DefaultControlModeIsVelocity)
{
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::VELOCITY);
}

TEST_F(DifferentialSimpleTest, SetControlModeAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
}

TEST_F(DifferentialSimpleTest, ControlModePersistsAfterReset)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.reset();
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
}

// ============================================================================
// Acceleration Control Mode Tests
// ============================================================================

TEST_F(DifferentialSimpleTest, AccelerationModeAppliesDirectAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandAcceleration(0.5f, 0.0f, 0.2f);

    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.05f, EPSILON); // 0.5 * 0.1
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);       // Y always zero for differential
    EXPECT_NEAR(actuator.getVelocity().z(), 0.02f, EPSILON); // 0.2 * 0.1

    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.10f, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), 0.04f, EPSILON);
}

TEST_F(DifferentialSimpleTest, AccelerationModeClampsVelocityToLimits)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandAcceleration(ACCELERATION_LINEAR, 0.0f, ACCELERATION_ANGULAR);

    // 10 s of constant acceleration would exceed both limits several times
    for (int i = 0; i < 100; ++i)
    {
        actuator.step(TIME_STEP);
    }

    EXPECT_NEAR(actuator.getVelocity().x(), VELOCITY_X_MAX, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), VELOCITY_ANGULAR_MAX, EPSILON);
}

TEST_F(DifferentialSimpleTest, AccelerationModeIgnoresCommandVelocity)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandVelocity(1.0f, 0.0f, 1.0f);
    actuator.setCommandAcceleration(0.0f, 0.0f, 0.0f);

    actuator.step(TIME_STEP);

    // Velocity should remain zero since acceleration is zero
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}
