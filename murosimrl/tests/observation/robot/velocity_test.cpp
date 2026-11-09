// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/velocity.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::robot::Robot;
using murosim::simulation::Simulation;
using murosimrl::observation::robot::Velocity;
using fixtures::EPSILON;

constexpr bool RAW = false;
constexpr bool NORMALIZED = true;
constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// The fixture actuator has no acceleration limit, so after one step the
// actual velocity equals the command.
class VelocityTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setActuatorCommandVelocity(0.5f, -0.25f, 0.1f);
        step();
    }
};

// ============================================================================
// Value Tests
// ============================================================================

TEST_F(VelocityTest, HolonomicShapeIsBatchByThree)
{
    const Velocity observation(RAW, NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 3}));
    EXPECT_EQ(observation.name(), "velocity");
}

TEST_F(VelocityTest, RawValuesAreTheActuatorVelocity)
{
    const Velocity observation(RAW, NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(output[0], 0.5f);
    EXPECT_FLOAT_EQ(output[1], -0.25f);
    EXPECT_FLOAT_EQ(output[2], 0.1f);
}

TEST_F(VelocityTest, ActualVelocityLagsBehindCommandWithAccelerationLimit)
{
    const std::shared_ptr<Simulation> own_sim = fixtures::makeSimulation("limited");
    const std::shared_ptr<Robot> limited = fixtures::holonomicRobot("limited", false, true);
    own_sim->addRobot(limited);
    own_sim->reset();
    limited->setActuatorCommandVelocity(1.0f, 0.0f, 0.0f);
    own_sim->simulate();
    const Velocity observation(RAW, NO_STACK);

    const std::vector<float> output = observation(fixtures::robotList(limited));

    // ramped for one step from rest, not the commanded 1 m/s
    EXPECT_NEAR(output[0], fixtures::ACCELERATION_LINEAR * fixtures::TIME_STEP, EPSILON);
}

TEST_F(VelocityTest, NormalizedBySymmetricLimits)
{
    const Velocity observation(NORMALIZED, NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_FLOAT_EQ(output[0], 0.5f);
    EXPECT_FLOAT_EQ(output[1], -0.25f);
    EXPECT_FLOAT_EQ(output[2], 0.1f);
}

TEST_F(VelocityTest, DifferentialRobotSkipsTheYAxis)
{
    const std::shared_ptr<Simulation> own_sim = fixtures::makeSimulation("differential");
    const std::shared_ptr<Robot> differential = fixtures::differentialRobot("differential");
    own_sim->addRobot(differential);
    own_sim->reset();
    differential->setActuatorCommandVelocity(0.5f, 0.0f, 0.25f);
    own_sim->simulate();
    const Velocity observation(RAW, NO_STACK);

    const std::vector<float> output = observation(fixtures::robotList(differential));

    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(output[0], 0.5f);
    EXPECT_FLOAT_EQ(output[1], 0.25f);
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(VelocityTest, StackIsNewestFirst)
{
    robot->setActuatorCommandVelocity(0.0f, 0.0f, 0.0f);
    step();
    const Velocity observation(RAW, 2);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(6));
    EXPECT_FLOAT_EQ(output[0], 0.0f); // stopped
    EXPECT_FLOAT_EQ(output[3], 0.5f); // still moving one step back
}

// ============================================================================
// Contract Tests
// ============================================================================

TEST_F(VelocityTest, MixedActuatorTypesThrow)
{
    const std::shared_ptr<Robot> differential = fixtures::differentialRobot("differential");
    differential->makeState();
    const Velocity observation(RAW, NO_STACK);

    EXPECT_THROW(observation(fixtures::RobotList{robot, differential}), std::runtime_error);
}
