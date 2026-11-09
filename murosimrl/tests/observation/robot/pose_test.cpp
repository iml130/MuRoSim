// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/pose.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosimrl::observation::robot::Pose;
using fixtures::EPSILON;

constexpr bool AS_ANGLE = false;
constexpr bool AS_UNIT_VECTOR = true;
constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class PoseTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(1.5f, -2.0f, 0.0f);
        step();
    }
};

// ============================================================================
// Angle Tests
// ============================================================================

TEST_F(PoseTest, AngleShapeIsBatchByThree)
{
    const Pose observation(AS_ANGLE, NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 3}));
    EXPECT_FALSE(observation.orientationAsUnitVector());
}

TEST_F(PoseTest, AngleValues)
{
    const Pose observation(AS_ANGLE, NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(output[0], 1.5f);
    EXPECT_FLOAT_EQ(output[1], -2.0f);
    EXPECT_NEAR(output[2], 0.0f, EPSILON);
}

TEST_F(PoseTest, AngleFollowsRotation)
{
    robot->setPose(1.5f, -2.0f, PI_4);
    step();
    const Pose observation(AS_ANGLE, NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[2], PI_4, EPSILON);
}

// ============================================================================
// Unit Vector Tests
// ============================================================================

TEST_F(PoseTest, UnitVectorShapeIsBatchByFour)
{
    const Pose observation(AS_UNIT_VECTOR, NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 4}));
    EXPECT_TRUE(observation.orientationAsUnitVector());
}

TEST_F(PoseTest, UnitVectorValues)
{
    robot->setPose(1.5f, -2.0f, PI_2);
    step();
    const Pose observation(AS_UNIT_VECTOR, NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(4));
    EXPECT_FLOAT_EQ(output[0], 1.5f);
    EXPECT_FLOAT_EQ(output[1], -2.0f);
    EXPECT_NEAR(output[2], 0.0f, EPSILON); // cos(pi/2)
    EXPECT_NEAR(output[3], 1.0f, EPSILON); // sin(pi/2)
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(PoseTest, StackShapeCarriesTheStackDimension)
{
    const Pose observation(AS_ANGLE, 2);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 2, 3}));
    EXPECT_EQ(observation.stackSize(), static_cast<size_t>(2));
}

// State index 0 is the current step, higher indices reach further into the
// past: the stacked layout is newest first.
TEST_F(PoseTest, StackIsNewestFirst)
{
    robot->setPose(0.0f, 0.0f, 0.0f);
    step();
    robot->setPose(1.0f, 0.0f, 0.0f);
    step();
    const Pose observation(AS_ANGLE, 2);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(6));
    EXPECT_FLOAT_EQ(output[0], 1.0f); // newest
    EXPECT_FLOAT_EQ(output[1], 0.0f);
    EXPECT_FLOAT_EQ(output[3], 0.0f); // one step back
    EXPECT_FLOAT_EQ(output[4], 0.0f);
}

// A stack size larger than the recorded history clamps to the oldest state
// (getStateAtOrOldest) instead of reading out of bounds.
TEST_F(PoseTest, StackClampsToOldestState)
{
    ASSERT_EQ(robot->getStateSize(), static_cast<size_t>(2)); // the reset and the fixture pose
    const Pose observation(AS_ANGLE, 5);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(15));
    EXPECT_FLOAT_EQ(output[0], 1.5f); // current
    EXPECT_FLOAT_EQ(output[1], -2.0f);
    EXPECT_FLOAT_EQ(output[3], 0.0f); // the reset state at the origin
    EXPECT_FLOAT_EQ(output[4], 0.0f);
    for (size_t j = 2; j < 5; j++)
    {
        EXPECT_FLOAT_EQ(output[j * 3 + 0], output[3]) << "stack " << j;
        EXPECT_FLOAT_EQ(output[j * 3 + 1], output[4]) << "stack " << j;
        EXPECT_FLOAT_EQ(output[j * 3 + 2], output[5]) << "stack " << j;
    }
}

// A stack size of 1 reads the same single state as no stacking, but the
// shape carries the stack dimension (only 0 leaves it out).
TEST_F(PoseTest, StackSizeOneAddsTheStackDimension)
{
    const Pose stacked(AS_ANGLE, 1);
    const Pose plain(AS_ANGLE, NO_STACK);

    EXPECT_EQ(stacked.shape(robots()), (std::vector<size_t>{1, 1, 3}));
    EXPECT_EQ(plain.shape(robots()), (std::vector<size_t>{1, 3}));
    EXPECT_EQ(stacked(robots()), plain(robots()));
}

// ============================================================================
// Contract Tests
// ============================================================================

// The checks below live in ObservationOf/ObservationRobot and hold for every
// robot observation; Pose is the simplest one to exercise them with.

TEST_F(PoseTest, EmptyInputThrows)
{
    const Pose observation(AS_ANGLE, NO_STACK);

    EXPECT_THROW(observation(fixtures::RobotList()), std::invalid_argument);
}

TEST_F(PoseTest, RobotWithoutStateThrows)
{
    const std::shared_ptr<murosim::robot::Robot> fresh = fixtures::holonomicRobot("fresh");
    const Pose observation(AS_ANGLE, NO_STACK);

    EXPECT_THROW(observation(fixtures::robotList(fresh)), std::runtime_error);
}

TEST_F(PoseTest, NameIsStored)
{
    EXPECT_EQ(Pose(AS_ANGLE, NO_STACK).name(), "pose");
}

TEST_F(PoseTest, PointerOverloadWritesTheSameValues)
{
    const Pose observation(AS_ANGLE, NO_STACK);
    std::vector<float> buffer(3, -1.0f);

    observation(robots(), buffer.data());

    EXPECT_EQ(buffer, observation(robots()));
}

// The concrete classes override shape(robots) and thereby hide the base
// class overloads for a single robot or for simulations; those are reached
// through an ObservationRobot reference.
TEST_F(PoseTest, SingleRobotAndSimulationOverloadsObserveTheSameRobots)
{
    const Pose observation(AS_ANGLE, NO_STACK);
    const murosimrl::observation::ObservationRobot &base = observation;

    EXPECT_EQ(base.shape(robot), observation.shape(robots()));
    EXPECT_EQ(base.shape(sim), observation.shape(robots()));
    EXPECT_EQ(base.shape(sims()), observation.shape(robots()));
    EXPECT_EQ(observation(robot), observation(robots()));
    EXPECT_EQ(observation(sim), observation(robots()));
    EXPECT_EQ(observation(sims()), observation(robots()));
}

TEST_F(PoseTest, SimulationWithoutRobotsThrows)
{
    const std::shared_ptr<murosim::simulation::Simulation> empty = fixtures::makeSimulation("empty");
    const Pose observation(AS_ANGLE, NO_STACK);

    EXPECT_THROW(observation(empty), std::invalid_argument);
    EXPECT_THROW(observation(fixtures::SimulationList{sim, empty}), std::invalid_argument);
}
