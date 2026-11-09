// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/util/conversion.hpp"
#include "fixtures.hpp"
#include <memory>

namespace
{
using murosim::simulation::Simulation;
using murosimrl::util::conversion::fromVectorSimulationToVectorRobot;
} // namespace

// ============================================================================
// Simulation To Robot Tests
// ============================================================================

TEST(UtilConversion, FlattensSimulationsInOrder)
{
    const std::shared_ptr<Simulation> first = fixtures::makeSimulation("first");
    const std::shared_ptr<Simulation> second = fixtures::makeSimulation("second");
    first->addRobot(fixtures::holonomicRobot("a"));
    first->addRobot(fixtures::holonomicRobot("b"));
    second->addRobot(fixtures::holonomicRobot("c"));

    const fixtures::RobotList robots = fromVectorSimulationToVectorRobot({first, second});

    ASSERT_EQ(robots.size(), static_cast<size_t>(3));
    EXPECT_EQ(robots[0], first->getRobotPtrAt(0)); // the same objects, not copies
    EXPECT_EQ(robots[1], first->getRobotPtrAt(1));
    EXPECT_EQ(robots[2], second->getRobotPtrAt(0));
}

TEST(UtilConversion, EmptySimulationsContributeNothing)
{
    const std::shared_ptr<Simulation> empty = fixtures::makeSimulation("empty");
    const std::shared_ptr<Simulation> one = fixtures::makeSimulation("one");
    one->addRobot(fixtures::holonomicRobot("a"));

    EXPECT_EQ(fromVectorSimulationToVectorRobot({empty, one, empty}).size(), static_cast<size_t>(1));
    EXPECT_TRUE(fromVectorSimulationToVectorRobot({}).empty());
}
