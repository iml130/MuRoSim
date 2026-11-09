// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/done.hpp"
#include "fixtures.hpp"
#include <memory>
#include <vector>

namespace
{
using murosim::robot::Robot;
using murosim::simulation::Simulation;
using murosimrl::Done;
} // namespace

// ============================================================================
// Robot Tests
// ============================================================================

TEST(DoneRobot, UnfinishedRobotIsFalse)
{
    const std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    const Done done;

    EXPECT_EQ(done(robot), std::vector<bool>{false});
}

TEST(DoneRobot, FinishedRobotIsTrue)
{
    const std::shared_ptr<Robot> robot = fixtures::holonomicRobot("robot");
    robot->finished();
    const Done done;

    EXPECT_EQ(done(robot), std::vector<bool>{true});
}

TEST(DoneRobot, RobotListKeepsOrder)
{
    const fixtures::RobotList robots = {fixtures::holonomicRobot("a"), fixtures::holonomicRobot("b"),
                                        fixtures::holonomicRobot("c")};
    robots[1]->finished();
    const Done done;

    EXPECT_EQ(done(robots), (std::vector<bool>{false, true, false}));
}

TEST(DoneRobot, PointerOverloadsWriteTheSameFlags)
{
    const fixtures::RobotList robots = {fixtures::holonomicRobot("a"), fixtures::holonomicRobot("b")};
    robots[0]->finished();
    const Done done;
    bool single = false;
    bool list[2] = {true, true};

    done(robots[0], &single);
    done(robots, list);

    EXPECT_TRUE(single);
    EXPECT_TRUE(list[0]);
    EXPECT_FALSE(list[1]);
}

// ============================================================================
// Simulation Tests
// ============================================================================

TEST(DoneSimulation, FlagsFollowSimulationThenRobotOrder)
{
    const std::shared_ptr<Simulation> first = fixtures::makeSimulation("first");
    const std::shared_ptr<Simulation> second = fixtures::makeSimulation("second");
    first->addRobot(fixtures::holonomicRobot("a"));
    first->addRobot(fixtures::holonomicRobot("b"));
    second->addRobot(fixtures::holonomicRobot("c"));
    first->getRobotAt(1).finished();
    second->getRobotAt(0).finished();
    const Done done;

    EXPECT_EQ(done(first), (std::vector<bool>{false, true}));
    EXPECT_EQ(done(fixtures::SimulationList{first, second}), (std::vector<bool>{false, true, true}));
}

TEST(DoneSimulation, PointerOverloadsWriteTheSameFlags)
{
    const std::shared_ptr<Simulation> sim = fixtures::makeSimulation("sim");
    sim->addRobot(fixtures::holonomicRobot("a"));
    sim->addRobot(fixtures::holonomicRobot("b"));
    sim->getRobotAt(0).finished();
    const Done done;
    bool single[2] = {false, true};
    bool list[2] = {false, true};

    done(sim, single);
    done(fixtures::SimulationList{sim}, list);

    EXPECT_TRUE(single[0]);
    EXPECT_FALSE(single[1]);
    EXPECT_TRUE(list[0]);
    EXPECT_FALSE(list[1]);
}

// ============================================================================
// Shape Tests
// ============================================================================

TEST(DoneShape, OneFlagPerRobot)
{
    const std::shared_ptr<Simulation> sim = fixtures::makeSimulation("sim");
    sim->addRobot(fixtures::holonomicRobot("a"));
    sim->addRobot(fixtures::holonomicRobot("b"));
    const fixtures::RobotList robots = {fixtures::holonomicRobot("c"), fixtures::holonomicRobot("d"),
                                        fixtures::holonomicRobot("e")};

    EXPECT_EQ(Done::shape(robots[0]), (std::vector<size_t>{1}));
    EXPECT_EQ(Done::shape(robots), (std::vector<size_t>{3, 1}));
    EXPECT_EQ(Done::shape(sim), (std::vector<size_t>{2, 1}));
    EXPECT_EQ(Done::shape(fixtures::SimulationList{sim, sim}), (std::vector<size_t>{4, 1}));
}
