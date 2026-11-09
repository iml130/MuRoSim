// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/command_velocity.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::robot::Robot;
using murosim::simulation::Simulation;
using murosimrl::observation::robot::CommandVelocity;

constexpr bool RAW = false;
constexpr bool NORMALIZED = true;
constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class CommandVelocityTest : public fixtures::SingleRobotTest
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

TEST_F(CommandVelocityTest, HolonomicShapeIsBatchByThree)
{
    const CommandVelocity observation(RAW, NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 3}));
    EXPECT_EQ(observation.name(), "command_velocity");
    EXPECT_FALSE(observation.normalize());
}

TEST_F(CommandVelocityTest, RawValuesInAxisOrder)
{
    const CommandVelocity observation(RAW, NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(output[0], 0.5f);   // x
    EXPECT_FLOAT_EQ(output[1], -0.25f); // y
    EXPECT_FLOAT_EQ(output[2], 0.1f);   // angular
}

TEST_F(CommandVelocityTest, NormalizedBySymmetricLimits)
{
    const CommandVelocity observation(NORMALIZED, NO_STACK);

    const std::vector<float> output = observation(robots());

    // limits are [-1, 1], so the normalized value equals the command
    EXPECT_FLOAT_EQ(output[0], 0.5f);
    EXPECT_FLOAT_EQ(output[1], -0.25f);
    EXPECT_FLOAT_EQ(output[2], 0.1f);
    EXPECT_TRUE(observation.normalize());
}

TEST_F(CommandVelocityTest, NormalizedByTheRobotsOwnAsymmetricLimits)
{
    const std::shared_ptr<Simulation> own_sim = fixtures::makeSimulation("asymmetric");
    const std::shared_ptr<Robot> asymmetric = fixtures::holonomicRobot("asymmetric", false, false, 0.0f); // x in [0, 1]
    own_sim->addRobot(asymmetric);
    own_sim->reset();
    asymmetric->setActuatorCommandVelocity(0.5f, 0.0f, 0.0f);
    own_sim->simulate();
    const CommandVelocity observation(NORMALIZED, NO_STACK);

    const std::vector<float> output = observation(fixtures::robotList(asymmetric));

    EXPECT_FLOAT_EQ(output[0], 0.0f); // the middle of [0, 1]
}

TEST_F(CommandVelocityTest, DifferentialRobotSkipsTheYAxis)
{
    const std::shared_ptr<Simulation> own_sim = fixtures::makeSimulation("differential");
    const std::shared_ptr<Robot> differential = fixtures::differentialRobot("differential");
    own_sim->addRobot(differential);
    own_sim->reset();
    differential->setActuatorCommandVelocity(0.5f, 0.0f, 0.25f);
    own_sim->simulate();
    const CommandVelocity observation(RAW, NO_STACK);

    const std::vector<float> output = observation(fixtures::robotList(differential));

    EXPECT_EQ(observation.shape(fixtures::robotList(differential)), (std::vector<size_t>{1, 2}));
    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(output[0], 0.5f);  // x
    EXPECT_FLOAT_EQ(output[1], 0.25f); // angular
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(CommandVelocityTest, StackIsNewestFirst)
{
    robot->setActuatorCommandVelocity(-0.5f, 0.0f, 0.0f);
    step();
    const CommandVelocity observation(RAW, 2);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 2, 3}));
    ASSERT_EQ(output.size(), static_cast<size_t>(6));
    EXPECT_FLOAT_EQ(output[0], -0.5f); // newest command
    EXPECT_FLOAT_EQ(output[3], 0.5f);  // the command one step back
    EXPECT_FLOAT_EQ(output[4], -0.25f);
}

// ============================================================================
// Contract Tests
// ============================================================================

TEST_F(CommandVelocityTest, MixedActuatorTypesThrow)
{
    const std::shared_ptr<Robot> differential = fixtures::differentialRobot("differential");
    differential->makeState();
    const CommandVelocity observation(RAW, NO_STACK);

    EXPECT_THROW(observation(fixtures::RobotList{robot, differential}), std::runtime_error);
}
