// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot_simulation/goal_normalized.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::simulation::Simulation;
using murosimrl::observation::robot_simulation::GoalNormalized;
using fixtures::EPSILON;

constexpr float NO_NORMALIZATION = 0.0f;
constexpr float NORMALIZE_DISTANCE = 10.0f;
constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// The square world is centred on the origin, so the goal offset from the
// world centre is the goal itself.
class GoalNormalizedTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setGoal(3.0f, 4.0f, 0.5f);
        step();
    }
};

// ============================================================================
// Value Tests
// ============================================================================

TEST_F(GoalNormalizedTest, ShapeIsRobotsByTwo)
{
    const GoalNormalized observation(NO_NORMALIZATION, NO_STACK);

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 2}));
    EXPECT_EQ(observation.name(), "goal_normalized");
}

TEST_F(GoalNormalizedTest, RawOffsetFromWorldCentre)
{
    const GoalNormalized observation(NO_NORMALIZATION, NO_STACK);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(output[0], 3.0f);
    EXPECT_FLOAT_EQ(output[1], 4.0f);
    EXPECT_FLOAT_EQ(observation.normalizeDistance(), NO_NORMALIZATION);
}

TEST_F(GoalNormalizedTest, NormalizedOffsetIsShiftedByHalf)
{
    const GoalNormalized observation(NORMALIZE_DISTANCE, NO_STACK);

    const std::vector<float> output = observation(sims());

    EXPECT_NEAR(output[0], 0.5f + 3.0f / NORMALIZE_DISTANCE, EPSILON);
    EXPECT_NEAR(output[1], 0.5f + 4.0f / NORMALIZE_DISTANCE, EPSILON);
}

TEST_F(GoalNormalizedTest, WorldCentreMapsToHalf)
{
    robot->setGoal(0.0f, 0.0f, 0.5f);
    step();
    const GoalNormalized observation(NORMALIZE_DISTANCE, NO_STACK);

    const std::vector<float> output = observation(sims());

    EXPECT_FLOAT_EQ(output[0], 0.5f);
    EXPECT_FLOAT_EQ(output[1], 0.5f);
}

TEST_F(GoalNormalizedTest, OffsetIsMeasuredFromTheWorldsOwnCentre)
{
    // A world whose AABB centre is not the origin: 10 m square from (0, 0) to (10, 10)
    const std::shared_ptr<Simulation> shifted = std::make_shared<Simulation>(
        murosim::geometry::World(murosim::geometry::primitives::square(
            fixtures::WORLD_SIZE, murosim::geometry::primitives::pose(5.0f, 5.0f, 0.0f))),
        fixtures::twoNodes(), "shifted", fixtures::GRID_SCALE, fixtures::BUFFER_SCALE,
        Simulation::INIT_MODE::MANUAL);
    const std::shared_ptr<murosim::robot::Robot> other = fixtures::holonomicRobot("other");
    other->setPose(5.0f, 5.0f, 0.0f); // inside the shifted world before the reset step
    shifted->addRobot(other);
    shifted->reset();
    other->setGoal(6.0f, 5.0f, 0.5f);
    shifted->simulate();
    const GoalNormalized observation(NO_NORMALIZATION, NO_STACK);

    const std::vector<float> output = observation(fixtures::SimulationList{shifted});

    EXPECT_NEAR(output[0], 1.0f, EPSILON); // one metre right of the centre (5, 5)
    EXPECT_NEAR(output[1], 0.0f, EPSILON);
}

// ============================================================================
// Batch Tests
// ============================================================================

TEST_F(GoalNormalizedTest, BatchCoversAllRobotsOfAllSimulations)
{
    const std::shared_ptr<Simulation> second = fixtures::makeSimulation("second");
    const std::shared_ptr<murosim::robot::Robot> a = fixtures::holonomicRobot("a");
    const std::shared_ptr<murosim::robot::Robot> b = fixtures::holonomicRobot("b");
    a->setPose(-2.0f, 0.0f, 0.0f); // apart, so the reset step records no collision
    b->setPose(2.0f, 0.0f, 0.0f);
    second->addRobot(a);
    second->addRobot(b);
    second->reset();
    a->setGoal(-1.0f, -1.0f, 0.5f);
    b->setGoal(2.0f, -2.0f, 0.5f);
    second->simulate();
    const GoalNormalized observation(NO_NORMALIZATION, NO_STACK);

    const std::vector<float> output = observation(fixtures::SimulationList{sim, second});

    EXPECT_EQ(observation.shape(fixtures::SimulationList{sim, second}), (std::vector<size_t>{3, 2}));
    ASSERT_EQ(output.size(), static_cast<size_t>(6));
    EXPECT_FLOAT_EQ(output[0], 3.0f); // fixture robot
    EXPECT_FLOAT_EQ(output[2], -1.0f); // a
    EXPECT_FLOAT_EQ(output[4], 2.0f);  // b
    EXPECT_FLOAT_EQ(output[5], -2.0f);
}

TEST_F(GoalNormalizedTest, StackIsNewestFirst)
{
    robot->setGoal(-2.0f, 1.0f, 0.5f);
    step();
    const GoalNormalized observation(NO_NORMALIZATION, 2);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 2, 2}));
    EXPECT_FLOAT_EQ(output[0], -2.0f);
    EXPECT_FLOAT_EQ(output[2], 3.0f);
}

// ============================================================================
// Contract Tests
// ============================================================================

TEST_F(GoalNormalizedTest, SimulationWithoutRobotsThrows)
{
    const std::shared_ptr<Simulation> empty = fixtures::makeSimulation("empty");
    const GoalNormalized observation(NO_NORMALIZATION, NO_STACK);
    float buffer[2] = {0.0f, 0.0f};

    // the vector overload sees a zero-sized result and returns before
    // validating; the buffer overload runs the check
    EXPECT_THROW(observation(fixtures::SimulationList{empty}, buffer), std::invalid_argument);
    EXPECT_TRUE(observation(fixtures::SimulationList{empty}).empty());
}

TEST_F(GoalNormalizedTest, RobotWithoutStateThrows)
{
    sim->addRobot(fixtures::holonomicRobot("fresh")); // added after the reset: no state yet
    const GoalNormalized observation(NO_NORMALIZATION, NO_STACK);

    EXPECT_THROW(observation(sims()), std::runtime_error);
}
