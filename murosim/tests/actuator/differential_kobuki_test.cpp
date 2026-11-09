// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/actuator/differential_kobuki.hpp"
#include "murosim/math/constants.hpp"
#include <limits>
#include <stdexcept>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::actuator::DifferentialKobuki;

constexpr float EPSILON = 1e-5f;
constexpr float TIME_STEP = 0.1f;

// Limits of the fixture actuator. Min and max differ on purpose so a test
// that confuses them fails. The acceleration limits are the Kobuki defaults
// (DifferentialKobuki::ACCELERATION_*_MAX). Note that VELOCITY_X_MAX lies
// above DifferentialKobuki::WHEEL_SPEED_MAX, so forward driving is capped by
// the wheels, not by this limit.
constexpr float VELOCITY_X_MIN = -0.5f;
constexpr float VELOCITY_X_MAX = 1.0f;
constexpr float VELOCITY_ANGULAR_MAX = PI_2;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(DifferentialKobukiConstructor, HardwareLimitsMatchKobukiSpec)
{
    EXPECT_FLOAT_EQ(DifferentialKobuki::VELOCITY_LINEAR_MAX, 0.7f);
    EXPECT_FLOAT_EQ(DifferentialKobuki::VELOCITY_ANGULAR_MAX, PI);
    EXPECT_FLOAT_EQ(DifferentialKobuki::ACCELERATION_LINEAR_MAX, 2.5f);
    EXPECT_FLOAT_EQ(DifferentialKobuki::ACCELERATION_ANGULAR_MAX, 32.0f);
    EXPECT_FLOAT_EQ(DifferentialKobuki::WHEEL_SPEED_MAX, 0.7f);
}

TEST(DifferentialKobukiConstructor, DefaultConstructor)
{
    DifferentialKobuki actuator;

    // Default velocity limits from Kobuki specs
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), -DifferentialKobuki::VELOCITY_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), DifferentialKobuki::VELOCITY_LINEAR_MAX);
    EXPECT_NEAR(actuator.getVelocityMin().z(), -DifferentialKobuki::VELOCITY_ANGULAR_MAX, EPSILON);
    EXPECT_NEAR(actuator.getVelocityMax().z(), DifferentialKobuki::VELOCITY_ANGULAR_MAX, EPSILON);

    // Y should be zero for differential drive
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().y(), 0.0f);

    // Default acceleration limits from Kobuki specs
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), DifferentialKobuki::ACCELERATION_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), DifferentialKobuki::ACCELERATION_ANGULAR_MAX);
}

TEST(DifferentialKobukiConstructor, VelocityLimitsConstructor)
{
    DifferentialKobuki actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), VELOCITY_ANGULAR_MAX);

    // Uses default Kobuki acceleration limits
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), DifferentialKobuki::ACCELERATION_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), DifferentialKobuki::ACCELERATION_ANGULAR_MAX);
}

TEST(DifferentialKobukiConstructor, FullConstructor)
{
    DifferentialKobuki actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX, 1.0f, 10.0f);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), -VELOCITY_ANGULAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), VELOCITY_ANGULAR_MAX);

    // Custom acceleration below the Kobuki max is kept as given
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), 1.0f);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), 10.0f);
}

TEST(DifferentialKobukiConstructor, AccelerationClampedToKobukiMax)
{
    // Request acceleration higher than Kobuki supports
    DifferentialKobuki actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX,
                                4.0f * DifferentialKobuki::ACCELERATION_LINEAR_MAX,
                                4.0f * DifferentialKobuki::ACCELERATION_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().x(), DifferentialKobuki::ACCELERATION_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getAccelerationMax().z(), DifferentialKobuki::ACCELERATION_ANGULAR_MAX);
}

TEST(DifferentialKobukiConstructor, NegativeAngularVelocityIsClamped)
{
    // Negative angular velocity max should be treated as zero
    DifferentialKobuki actuator(VELOCITY_X_MIN, VELOCITY_X_MAX, -1.0f);

    EXPECT_FLOAT_EQ(actuator.getVelocityMin().z(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocityMax().z(), 0.0f);
}

// ============================================================================
// Test Fixture
// ============================================================================

class DifferentialKobukiTest : public ::testing::Test
{
protected:
    DifferentialKobuki actuator{VELOCITY_X_MIN, VELOCITY_X_MAX, VELOCITY_ANGULAR_MAX};
};

// ============================================================================
// Type Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, TypeIsXAngular)
{
    EXPECT_EQ(actuator.getType(), murosim::actuator::Actuator::TYPE::X_ANGULAR);
    EXPECT_EQ(actuator.getTypeAsString(), "X_ANGULAR");
}

// ============================================================================
// Initial State Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, InitialVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(DifferentialKobukiTest, InitialCommandVelocityIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), 0.0f);
}

TEST_F(DifferentialKobukiTest, InitialCommandAccelerationIsZero)
{
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 0.0f);
}

// ============================================================================
// Command Velocity Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, SetCommandVelocityWithinLimits)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

TEST_F(DifferentialKobukiTest, SetCommandVelocityClampedToMax)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MAX, 1.0f, 2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), VELOCITY_ANGULAR_MAX);
}

TEST_F(DifferentialKobukiTest, SetCommandVelocityClampedToMin)
{
    actuator.setCommandVelocity(2.0f * VELOCITY_X_MIN, -1.0f, -2.0f * VELOCITY_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), VELOCITY_X_MIN);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), -VELOCITY_ANGULAR_MAX);
}

TEST_F(DifferentialKobukiTest, SetCommandVelocityWithVector)
{
    actuator.setCommandVelocity(Eigen::Vector3f(0.5f, 0.5f, PI_4));

    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandVelocity().z(), PI_4);
}

// ============================================================================
// Command Acceleration Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, SetCommandAccelerationWithinLimits)
{
    actuator.setCommandAcceleration(1.0f, 0.0f, 10.0f);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 1.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 10.0f);
}

TEST_F(DifferentialKobukiTest, SetCommandAccelerationClampedToMax)
{
    actuator.setCommandAcceleration(4.0f * DifferentialKobuki::ACCELERATION_LINEAR_MAX, 1.0f,
                                    4.0f * DifferentialKobuki::ACCELERATION_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), DifferentialKobuki::ACCELERATION_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), DifferentialKobuki::ACCELERATION_ANGULAR_MAX);
}

TEST_F(DifferentialKobukiTest, SetCommandAccelerationClampedToNegativeMax)
{
    actuator.setCommandAcceleration(-4.0f * DifferentialKobuki::ACCELERATION_LINEAR_MAX, -1.0f,
                                    -4.0f * DifferentialKobuki::ACCELERATION_ANGULAR_MAX);

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), -DifferentialKobuki::ACCELERATION_LINEAR_MAX);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), -DifferentialKobuki::ACCELERATION_ANGULAR_MAX);
}

TEST_F(DifferentialKobukiTest, SetCommandAccelerationWithVector)
{
    actuator.setCommandAcceleration(Eigen::Vector3f(1.0f, 1.0f, 10.0f));

    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().x(), 1.0f);
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().y(), 0.0f); // Y always zero for differential
    EXPECT_FLOAT_EQ(actuator.getCommandAcceleration().z(), 10.0f);
}

// ============================================================================
// Step Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, StepAcceleratesLinearVelocity)
{
    actuator.setCommandVelocity(VELOCITY_X_MAX, 0.0f, 0.0f);

    // Velocity ramps with the Kobuki acceleration limit: 0.25 m/s per step
    constexpr float ramp = DifferentialKobuki::ACCELERATION_LINEAR_MAX * TIME_STEP;

    actuator.step(TIME_STEP);
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), ramp);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);

    actuator.step(TIME_STEP);
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 2.0f * ramp);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);

    // Third step would reach 0.75 m/s, but the wheels cap it at WHEEL_SPEED_MAX
    actuator.step(TIME_STEP);
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), DifferentialKobuki::WHEEL_SPEED_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(DifferentialKobukiTest, StepWheelSpeedLimitsVelocity)
{
    // Reach the wheel speed limit driving straight
    actuator.setCommandVelocity(VELOCITY_X_MAX, 0.0f, 0.0f);
    for (int i = 0; i < 5; ++i)
    {
        actuator.step(TIME_STEP);
    }
    ASSERT_FLOAT_EQ(actuator.getVelocity().x(), DifferentialKobuki::WHEEL_SPEED_MAX);

    // Both wheels already spin at their maximum, so there is no room for a
    // speed difference between them: the angular command cannot be realised
    actuator.setCommandVelocity(VELOCITY_X_MAX, 0.0f, VELOCITY_ANGULAR_MAX);
    actuator.step(TIME_STEP);

    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), DifferentialKobuki::WHEEL_SPEED_MAX);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(DifferentialKobukiTest, StepAngularVelocity)
{
    // The Kobuki angular acceleration limit (32 rad/s^2) covers the whole
    // angular range within one step, so the command is reached immediately
    static_assert(DifferentialKobuki::ACCELERATION_ANGULAR_MAX * TIME_STEP > VELOCITY_ANGULAR_MAX);

    actuator.setCommandVelocity(0.0f, 0.0f, VELOCITY_ANGULAR_MAX);
    actuator.step(TIME_STEP);
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), VELOCITY_ANGULAR_MAX);

    // Reverse direction
    actuator.setCommandVelocity(0.0f, 0.0f, -VELOCITY_ANGULAR_MAX);
    actuator.step(TIME_STEP);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), -VELOCITY_ANGULAR_MAX);

    // Stop
    actuator.setCommandVelocity(0.0f, 0.0f, 0.0f);
    actuator.step(TIME_STEP);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(DifferentialKobukiTest, StepReturnsTransformation)
{
    // 0.5 m/s is reached after two steps (0.25 m/s ramp), then held
    const float command = 0.5f;
    actuator.setCommandVelocity(command, 0.0f, 0.0f);

    Eigen::Affine2f pose = Eigen::Affine2f::Identity();
    for (int i = 0; i < 10; ++i)
    {
        pose = pose * actuator.step(TIME_STEP);
    }

    // Each step translates by the velocity reached in that step times dt
    const float ramp = DifferentialKobuki::ACCELERATION_LINEAR_MAX * TIME_STEP;
    const float expected_distance = (ramp + 9.0f * command) * TIME_STEP;

    // Straight forward: no rotation, no sideways motion
    EXPECT_NEAR(pose.translation().x(), expected_distance, EPSILON);
    EXPECT_NEAR(pose.translation().y(), 0.0f, EPSILON);
    EXPECT_NEAR(Eigen::Rotation2Df(pose.rotation()).angle(), 0.0f, EPSILON);
}

// Regression: the per-step transform composes translation THEN rotation, so the
// step's body-frame displacement (v*dt, 0) is NOT rotated by that same step's
// yaw. This matches the upstream ecl_navigation pose update
// (pose_update.translation(ds, 0); pose_update.rotation(domega);) and the other
// actuators. The previous rotate-then-translate order would rotate the
// translation by omega*dt, yielding a nonzero y component.
TEST_F(DifferentialKobukiTest, StepCombinedMotionTranslatesBeforeRotating)
{
    const float linear = 0.3f;
    const float angular = PI_4;

    // Ramp to steady state so velocity equals the command exactly (these values
    // stay below the wheel-speed limit, so no clamping reduces them).
    actuator.setCommandVelocity(linear, 0.0f, angular);
    for (int i = 0; i < 10; ++i)
    {
        actuator.step(TIME_STEP);
    }
    ASSERT_FLOAT_EQ(actuator.getVelocity().x(), linear);
    ASSERT_FLOAT_EQ(actuator.getVelocity().z(), angular);

    // One more step at steady state: inspect the incremental transform.
    const Eigen::Affine2f movement = actuator.step(TIME_STEP);

    // Translation is the unrotated body-frame displacement.
    EXPECT_NEAR(movement.translation().x(), linear * TIME_STEP, EPSILON);
    EXPECT_NEAR(movement.translation().y(), 0.0f, EPSILON);

    // Rotation is the angular velocity integrated over the step.
    const float angle = Eigen::Rotation2Df(movement.rotation()).angle();
    EXPECT_NEAR(angle, angular * TIME_STEP, EPSILON);
}

TEST_F(DifferentialKobukiTest, StepZeroDtIsIdentityNoOp)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);

    const Eigen::Affine2f movement = actuator.step(0.0f);

    // No time elapsed: identity transform and velocity untouched
    EXPECT_TRUE(movement.isApprox(Eigen::Affine2f::Identity()));
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}

TEST_F(DifferentialKobukiTest, StepNegativeOrNaNDtThrows)
{
    EXPECT_THROW(actuator.step(-0.1f), std::invalid_argument);
    EXPECT_THROW(actuator.step(std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, ResetClearsAllState)
{
    // Set some state
    actuator.setCommandVelocity(1.0f, 0.0f, PI_4);
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

TEST_F(DifferentialKobukiTest, CloneCreatesIndependentCopy)
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

TEST_F(DifferentialKobukiTest, GetVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.0f, 0.0f);
    actuator.step(TIME_STEP);

    float x, y, angular;
    actuator.getVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, actuator.getVelocity().x());
    EXPECT_FLOAT_EQ(y, actuator.getVelocity().y());
    EXPECT_FLOAT_EQ(angular, actuator.getVelocity().z());
}

TEST_F(DifferentialKobukiTest, GetCommandVelocityFloatOverload)
{
    actuator.setCommandVelocity(0.5f, 0.0f, PI_4);

    float x, y, angular;
    actuator.getCommandVelocity(x, y, angular);

    EXPECT_FLOAT_EQ(x, 0.5f);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(angular, PI_4);
}

TEST_F(DifferentialKobukiTest, GetVelocityMinMaxFloatOverload)
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

TEST_F(DifferentialKobukiTest, GetAccelerationMaxFloatOverload)
{
    float x, y, angular;
    actuator.getAccelerationMax(x, y, angular);

    EXPECT_FLOAT_EQ(x, DifferentialKobuki::ACCELERATION_LINEAR_MAX);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(angular, DifferentialKobuki::ACCELERATION_ANGULAR_MAX);
}

// ============================================================================
// Control Mode Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, DefaultControlModeIsVelocity)
{
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::VELOCITY);
}

TEST_F(DifferentialKobukiTest, SetControlModeAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
}

TEST_F(DifferentialKobukiTest, ControlModePersistsAfterReset)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.reset();
    EXPECT_EQ(actuator.getControlMode(), murosim::actuator::CONTROL_MODE::ACCELERATION);
}

// ============================================================================
// Acceleration Control Mode Tests
// ============================================================================

TEST_F(DifferentialKobukiTest, AccelerationModeAppliesDirectAcceleration)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandAcceleration(0.5f, 0.0f, 0.2f);

    // These velocities stay far below the wheel speed limit
    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.05f, EPSILON); // 0.5 * 0.1
    EXPECT_FLOAT_EQ(actuator.getVelocity().y(), 0.0f);       // Y always zero for differential
    EXPECT_NEAR(actuator.getVelocity().z(), 0.02f, EPSILON); // 0.2 * 0.1

    actuator.step(TIME_STEP);
    EXPECT_NEAR(actuator.getVelocity().x(), 0.10f, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), 0.04f, EPSILON);
}

TEST_F(DifferentialKobukiTest, AccelerationModeClampsVelocityToLimits)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);

    // Accelerate backwards: VELOCITY_X_MIN is reachable, whereas the positive
    // limit lies above the wheel speed limit and could not be observed
    actuator.setCommandAcceleration(-1.0f, 0.0f, 10.0f);

    // 10 s of constant acceleration would exceed both limits several times
    for (int i = 0; i < 100; ++i)
    {
        actuator.step(TIME_STEP);
    }

    EXPECT_NEAR(actuator.getVelocity().x(), VELOCITY_X_MIN, EPSILON);
    EXPECT_NEAR(actuator.getVelocity().z(), VELOCITY_ANGULAR_MAX, EPSILON);
}

TEST_F(DifferentialKobukiTest, AccelerationModeIgnoresCommandVelocity)
{
    actuator.setControlMode(murosim::actuator::CONTROL_MODE::ACCELERATION);
    actuator.setCommandVelocity(1.0f, 0.0f, 1.0f);
    actuator.setCommandAcceleration(0.0f, 0.0f, 0.0f);

    actuator.step(TIME_STEP);

    // Velocity should remain zero since acceleration is zero
    EXPECT_FLOAT_EQ(actuator.getVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(actuator.getVelocity().z(), 0.0f);
}
