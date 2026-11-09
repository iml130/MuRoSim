// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/util/simulation.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>

namespace
{
using murosim::simulation::Simulation;
using murosimrl::util::simulation::checkRobotHasStates;
using murosimrl::util::simulation::getNumberOfRobots;
using murosimrl::util::simulation::requireGridMapFitSize;
using murosimrl::util::simulation::requireSimulationsHaveRobots;

constexpr size_t FIXTURE_MAP_SIZE = 101; // 10 m at 0.1 m per cell, plus one
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class UtilSimulationTest : public ::testing::Test
{
protected:
    std::shared_ptr<Simulation> first = fixtures::makeSimulation("first");
    std::shared_ptr<Simulation> second = fixtures::makeSimulation("second");
    std::shared_ptr<Simulation> empty = fixtures::makeSimulation("empty");

    void SetUp() override
    {
        first->addRobot(fixtures::holonomicRobot("a"));
        first->addRobot(fixtures::holonomicRobot("b"));
        second->addRobot(fixtures::holonomicRobot("c"));
    }
};

// ============================================================================
// Grid Map Fit Tests
// ============================================================================

TEST_F(UtilSimulationTest, MapThatFitsIsAccepted)
{
    ASSERT_EQ(first->getGridMap().sizeX(), FIXTURE_MAP_SIZE);

    EXPECT_NO_THROW(requireGridMapFitSize({first, second}, FIXTURE_MAP_SIZE, "test"));
    EXPECT_NO_THROW(requireGridMapFitSize({first}, 2 * FIXTURE_MAP_SIZE, "test"));
}

TEST_F(UtilSimulationTest, MapLargerThanBufferThrows)
{
    EXPECT_THROW(requireGridMapFitSize({first}, FIXTURE_MAP_SIZE - 1, "test"), std::runtime_error);
}

TEST_F(UtilSimulationTest, OneOversizedMapInTheListThrows)
{
    const std::shared_ptr<Simulation> large = fixtures::makeSimulation("large", 2.0f * fixtures::WORLD_SIZE);

    EXPECT_THROW(requireGridMapFitSize({first, large}, FIXTURE_MAP_SIZE, "test"), std::runtime_error);
}

// ============================================================================
// Robot Presence Tests
// ============================================================================

TEST_F(UtilSimulationTest, SimulationsWithRobotsAreAccepted)
{
    EXPECT_NO_THROW(requireSimulationsHaveRobots({first, second}));
    EXPECT_NO_THROW(requireSimulationsHaveRobots({}));
}

TEST_F(UtilSimulationTest, SimulationWithoutRobotsThrows)
{
    EXPECT_THROW(requireSimulationsHaveRobots({first, empty}), std::invalid_argument);
}

TEST_F(UtilSimulationTest, NumberOfRobotsSumsAllSimulations)
{
    EXPECT_EQ(getNumberOfRobots({first, second, empty}), static_cast<size_t>(3));
    EXPECT_EQ(getNumberOfRobots({}), static_cast<size_t>(0));
}

// ============================================================================
// State Check Tests
// ============================================================================

TEST_F(UtilSimulationTest, RobotsWithoutStatesAreRejected)
{
    EXPECT_FALSE(checkRobotHasStates({first}));
}

TEST_F(UtilSimulationTest, ResetRecordsAStateForEveryRobot)
{
    first->reset();
    second->reset();

    EXPECT_TRUE(checkRobotHasStates({first, second}));
    EXPECT_TRUE(checkRobotHasStates({empty})); // nothing to check
}
