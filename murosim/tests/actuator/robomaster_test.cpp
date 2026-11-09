// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/actuator/robomaster.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::actuator::Actuator;
using murosim::actuator::RoboMaster;

constexpr float EPSILON = 1e-5f;
constexpr float TIME_STEP = 0.1f;

// Limits of the fixture actuators. Min and max differ on purpose so a test
// that confuses them fails. The acceleration limits are far below the wheel
// slew-rate limit of the hardware, so ramps are governed by these values.
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

TEST(RoboMasterConstructor, HardwareLimitsMatchRoboMasterSpec)
{
    EXPECT_FLOAT_EQ(RoboMaster::VELOCITY_LINEAR_MAX, 3.0f);
    EXPECT_FLOAT_EQ(RoboMaster::VELOCITY_ANGULAR_MAX, 2.0f);
    EXPECT_FLOAT_EQ(RoboMaster::WHEEL_RPM_MAX, 1000.0f);
    EXPECT_FLOAT_EQ(RoboMaster::TIME_TO_MAX_RPM, 5.0f / 3.0f);
    EXPECT_FLOAT_EQ(RoboMaster::SAMPLE_TIME, 0.02f);
}

TEST(RoboMasterConstructor, DefaultConstructor)
{
    RoboMaster actuator;

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), -RoboMaster::VELOCITY_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), RoboMaster::VELOCITY_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), -RoboMaster::VELOCITY_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), RoboMaster::VELOCITY_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -RoboMaster::VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), RoboMaster::VELOCITY_ANGULAR_MAX);

    // Acceleration defaults to zero (no limit)
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);

    // Default type is X_Y_ANGULAR
    EXPECT_EQ(actuator.getType(), Actuator::TYPE::X_Y_ANGULAR);
}

TEST(RoboMasterConstructor, VelocityMaxConstructor)
{
    RoboMaster actuator(VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX);

    // Symmetric limits on x and y
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), -VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), -VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), VELOCITY_ANGULAR_MAX);

    // Acceleration defaults to zero (no limit)
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);
}

TEST(RoboMasterConstructor, VelocityLimitsConstructor)
{
    RoboMaster actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX);

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

TEST(RoboMasterConstructor, FullConstructor)
{
    RoboMaster actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
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

TEST(RoboMasterConstructor, TypeXAngularZeroesYVelocity)
{
    RoboMaster actuator(VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX, Actuator::TYPE::X_ANGULAR);

    EXPECT_EQ(actuator.getType(), Actuator::TYPE::X_ANGULAR);

    // Y velocity limits are forced to zero for a differential-like type
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), 0.0f);
}

TEST(RoboMasterConstructor, TypeXYZeroesAngularVelocity)
{
    RoboMaster actuator(VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX, Actuator::TYPE::X_Y);

    EXPECT_EQ(actuator.getType(), Actuator::TYPE::X_Y);

    // Angular velocity limits are forced to zero for a translation-only type
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), 0.0f);
}

TEST(RoboMasterConstructor, NegativeAccelerationIsClamped)
{
    // Negative acceleration should be treated as zero
    RoboMaster actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX, -1.0f, -1.0f);

    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 0.0f);
}

// ============================================================================
// Test Fixtures
// ============================================================================

class RoboMasterXYAngularTest : public ::testing::Test
{
protected:
    RoboMaster actuator{VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                        ACCELERATION_LINEAR, ACCELERATION_ANGULAR, Actuator::TYPE::X_Y_ANGULAR};
};

class RoboMasterXAngularTest : public ::testing::Test
{
protected:
    RoboMaster actuator{VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                        ACCELERATION_LINEAR, ACCELERATION_ANGULAR, Actuator::TYPE::X_ANGULAR};
};

class RoboMasterXYTest : public ::testing::Test
{
protected:
    RoboMaster actuator{VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_Y_MAX, VELOCITY_ANGULAR_MAX,
                        ACCELERATION_LINEAR, ACCELERATION_ANGULAR, Actuator::TYPE::X_Y};
};

// ============================================================================
// Type Tests
// ============================================================================

TEST_F(RoboMasterXYAngularTest, TypeIsXYAngular)
{
    EXPECT_EQ(actuator.getType(), Actuator::TYPE::X_Y_ANGULAR);
    EXPECT_EQ(actuator.getTypeAsString(), "X_Y_ANGULAR");
}

TEST_F(RoboMasterXAngularTest, TypeIsXAngular)
{
    EXPECT_EQ(actuator.getType(), Actuator::TYPE::X_ANGULAR);
    EXPECT_EQ(actuator.getTypeAsString(), "X_ANGULAR");
}

TEST_F(RoboMasterXYTest, TypeIsXY)
{
    EXPECT_EQ(actuator.getType(), Actuator::TYPE::X_Y);
    EXPECT_EQ(actuator.getTypeAsString(), "X_Y");
}

// ============================================================================
// Initial State Tests
// ============================================================================

TEST_F(RoboMasterXYAngularTest, InitialVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(RoboMasterXYAngularTest, InitialCommandVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), 0.0f);
}

TEST_F(RoboMasterXYAngularTest, InitialCommandAccelerationIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.0f);
}

// ============================================================================
// Command Velocity Tests - X_Y_ANGULAR
// ============================================================================

TEST_F(RoboMasterXYAngularTest, SetCommandVelocityWithinLimits)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

TEST_F(RoboMasterXYAngularTest, SetCommandVelocityClampedToMax)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MAX, 2.0f * VELOCITY_Y_MAX, 2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), VELOCITY_ANGULAR_MAX);
}

TEST_F(RoboMasterXYAngularTest, SetCommandVelocityClampedToMin)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MIN, -2.0f * VELOCITY_Y_MAX, -2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), -VELOCITY_Y_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), -VELOCITY_ANGULAR_MAX);
}

TEST_F(RoboMasterXYAngularTest, SetCommandVelocityWithVector)
{
    actuator.setCommandVelocity(Eigen::Vector3f(0.5f, 0.5f, PI_4));

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

// ============================================================================
// Command Velocity Tests - X_ANGULAR
// ============================================================================

TEST_F(RoboMasterXAngularTest, SetCommandVelocityForcesYToZero)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for X_ANGULAR
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

// ============================================================================
// Command Velocity Tests - X_Y
// ============================================================================

TEST_F(RoboMasterXYTest, SetCommandVelocityForcesAngularToZero)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), 0.0f); // Angular always zero for X_Y
}

// ============================================================================
// Command Acceleration Tests
// ============================================================================

TEST_F(RoboMasterXYAngularTest, SetCommandAccelerationWithinLimits)
{
    actuator.setCommandAcceleration(0.3f, 0.3f, 0.5f);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.5f);
}

TEST_F(RoboMasterXYAngularTest, SetCommandAccelerationClampedToMax)
{
    actuator.setCommandAcceleration(2.0f * ACCELERATION_LINEAR, 2.0f * ACCELERATION_LINEAR,
                                    2.0f * ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), ACCELERATION_ANGULAR);
}

TEST_F(RoboMasterXYAngularTest, SetCommandAccelerationClampedToNegativeMax)
{
    actuator.setCommandAcceleration(-2.0f * ACCELERATION_LINEAR, -2.0f * ACCELERATION_LINEAR,
                                    -2.0f * ACCELERATION_ANGULAR);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), -ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), -ACCELERATION_LINEAR);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), -ACCELERATION_ANGULAR);
}

TEST_F(RoboMasterXYAngularTest, SetCommandAccelerationWithVector)
{
    actuator.setCommandAcceleration(Eigen::Vector3f(0.3f, 0.3f, 0.5f));

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.3f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.5f);
}

// ============================================================================
// Step Tests
// ============================================================================

TEST_F(RoboMasterXYAngularTest, StepAcceleratesLinearXVelocity)
{
    actuator.setCommandVelocity(VELOCITY_X_MAX, 0.0f, 0.0f);

    // Velocity ramps linearly with the acceleration limit; the internal 50 Hz
    // sub-steps do not change the value reached at the end of each step
    for (int i = 1; i <= 10; ++i)
    {
        actuator.step(TIME_STEP);
        const float expected_velocity = static_cast<float>(i) * TIME_STEP * ACCELERATION_LINEAR;

        EXPECT_NEAR(actuator.getVelocity().x(), expected_velocity, EPSILON) << "step " << i;
        EXPECT_NEAR(actuator.getVelocity().y(), 0.0f, EPSILON);
        EXPECT_NEAR(actuator.getVelocity().z(), 0.0f, EPSILON);
    }
}

TEST_F(RoboMasterXYAngularTest, StepAcceleratesLinearYVelocity)
{
    actuator.setCommandVelocity(0.0f, VELOCITY_Y_MAX, 0.0f);

    for (int i = 1; i <= 10; ++i)
    {
        actuator.step(TIME_STEP);
        const float expected_velocity = static_cast<float>(i) * TIME_STEP * ACCELERATION_LINEAR;

        EXPECT_NEAR(actuator.getVelocity().x(), 0.0f, EPSILON);
        EXPECT_NEAR(actuator.getVelocity().y(), expected_velocity, EPSILON) << "step " << i;
        EXPECT_NEAR(actuator.getVelocity().z(), 0.0f, EPSILON);
    }
}

TEST_F(RoboMasterXYAngularTest, StepAcceleratesAngularVelocity)
{
    actuator.setCommandVelocity(0.0f, 0.0f, VELOCITY_ANGULAR_MAX);

    for (int i = 1; i <= 10; ++i)
    {
        actuator.step(TIME_STEP);
        const float expected_angular = static_cast<float>(i) * TIME_STEP * ACCELERATION_ANGULAR;

        EXPECT_NEAR(actuator.getVelocity().x(), 0.0f, EPSILON);
        EXPECT_NEAR(actuator.getVelocity().y(), 0.0f, EPSILON);
        EXPECT_NEAR(actuator.getVelocity().z(), expected_angular, EPSILON) << "step " << i;
    }
}

TEST_F(RoboMasterXYAngularTest, StepReturnsTransformation)
{
    // Ramp to steady state: x needs 20 steps at 0.05 m/s per step, y needs 10
    const float command_x = 1.0f;
    const float command_y = 0.5f;
    actuator.setCommandVelocity(command_x, command_y, 0.0f);
    for (int i = 0; i < 30; ++i)
    {
        actuator.step(TIME_STEP);
    }
    ASSERT_NEAR(actuator.getVelocity().x(), command_x, EPSILON);
    ASSERT_NEAR(actuator.getVelocity().y(), command_y, EPSILON);

    // At constant velocity one step translates by exactly velocity * dt
    const Eigen::Affine2f movement = actuator.step(TIME_STEP);

    EXPECT_NEAR(movement.translation().x(), command_x * TIME_STEP, EPSILON);
    EXPECT_NEAR(movement.translation().y(), command_y * TIME_STEP, EPSILON);
    EXPECT_NEAR(Eigen::Rotation2Df(movement.rotation()).angle(), 0.0f, EPSILON);
}

TEST_F(RoboMasterXYAngularTest, StepCombinedMotionRotatesAndTranslates)
{
    // Ramp to steady state on all three axes (10 steps suffice)
    const float angular = PI_4;
    actuator.setCommandVelocity(0.5f, 0.5f, angular);
    for (int i = 0; i < 30; ++i)
    {
        actuator.step(TIME_STEP);
    }
    ASSERT_NEAR(actuator.getVelocity().z(), angular, EPSILON);

    const Eigen::Affine2f movement = actuator.step(TIME_STEP);

    // Rotations of the 50 Hz sub-steps add up exactly
    EXPECT_NEAR(Eigen::Rotation2Df(movement.rotation()).angle(), angular * TIME_STEP, EPSILON);

    // The translation is composed from sub-steps that are each rotated by the
    // sub-steps before, so it is not exactly velocity * dt; it does move
    // forward and sideways though
    EXPECT_GT(movement.translation().x(), 0.0f);
    EXPECT_GT(movement.translation().y(), 0.0f);
}

TEST_F(RoboMasterXAngularTest, StepRespectsDifferentialConstraints)
{
    // The y command is ignored for X_ANGULAR
    actuator.setCommandVelocity(0.5f, 1.0f, PI_4);

    for (int i = 0; i < 50; ++i)
    {
        actuator.step(TIME_STEP);
    }

    EXPECT_NEAR(actuator.getVelocity().y(), 0.0f, EPSILON);
    EXPECT_GT(actuator.getVelocity().x(), 0.0f);
    EXPECT_GT(actuator.getVelocity().z(), 0.0f);
}

TEST_F(RoboMasterXYTest, StepRespectsHolonomicConstraints)
{
    // The angular command is ignored for X_Y
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    for (int i = 0; i < 50; ++i)
    {
        actuator.step(TIME_STEP);
    }

    EXPECT_NEAR(actuator.getVelocity().z(), 0.0f, EPSILON);
    EXPECT_GT(actuator.getVelocity().x(), 0.0f);
    EXPECT_GT(actuator.getVelocity().y(), 0.0f);
}

TEST_F(RoboMasterXYAngularTest, StepZeroDtIsIdentityNoOp)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    const Eigen::Affine2f movement = actuator.step(0.0f);

    // No time elapsed: identity transform and velocity untouched
    EXPECT_TRUE(movement.isApprox(Eigen::Affine2f::Identity()));
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(RoboMasterXYAngularTest, StepNegativeOrNaNDtThrows)
{
    EXPECT_THROW(actuator.step(-0.1f), std::invalid_argument);
    EXPECT_THROW(actuator.step(std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(RoboMasterXYAngularTest, ResetClearsAllState)
{
    // Set some state
    actuator.setCommandVelocity(1.0f, 0.5f, PI_4);
    actuator.setCommandAcceleration(0.3f, 0.3f, 0.5f);
    for (int i = 0; i < 10; ++i)
    {
        actuator.step(TIME_STEP);
    }

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

TEST_F(RoboMasterXYAngularTest, CloneCreatesIndependentCopy)
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

TEST_F(RoboMasterXYAngularTest, GetVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);
    actuator.step(TIME_STEP);

    float x, y, angular;
    actuator.getVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, actuator.getVelocity().x());
    EXPECT_FLOAT_EQ(y, actuator.getVelocity().y());
    EXPECT_FLOAT_EQ(angular, actuator.getVelocity().z());
}

TEST_F(RoboMasterXYAngularTest, GetCommandVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.5f, PI_4);

    float x, y, angular;
    actuator.getCommandVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, 0.5f);
    EXPECT_FLOAT_EQ(y, 0.5f);
    EXPECT_FLOAT_EQ(angular, PI_4);
}

TEST_F(RoboMasterXYAngularTest, GetVelocityMinMaxFloatOverload)
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

TEST_F(RoboMasterXYAngularTest, GetAccelerationMaxFloatOverload)
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

TEST_F(RoboMasterXYAngularTest, DefaultControlModeIsVelocity)
{
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::VELOCITY);
}

TEST_F(RoboMasterXYAngularTest, SetControlModeAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
}

TEST_F(RoboMasterXYAngularTest, ControlModePersistsAfterReset)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.reset();
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
}

// ============================================================================
// Acceleration Control Mode Tests
// ============================================================================

TEST_F(RoboMasterXYAngularTest, AccelerationModeAppliesDirectAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandAcceleration(0.5f, 0.3f, 0.2f);

    // These accelerations stay far below the wheel slew-rate limit
    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.05f, EPSILON); // 0.5 * 0.1
    EXPECT_NEAR(actuator.getVelocity().y(), 0.03f, EPSILON); // 0.3 * 0.1
    EXPECT_NEAR(actuator.getVelocity().z(), 0.02f, EPSILON); // 0.2 * 0.1

    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.10f, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().y(), 0.06f, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), 0.04f, EPSILON);
}

TEST_F(RoboMasterXYAngularTest, AccelerationModeClampsVelocityToLimits)
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

TEST_F(RoboMasterXYAngularTest, AccelerationModeIgnoresCommandVelocity)
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
