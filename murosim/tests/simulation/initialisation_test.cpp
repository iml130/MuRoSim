// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/simulation/initialisation.hpp"
#include "murosim/robot/robot_builder.hpp"
#include <memory>
#include <vector>

namespace
{
using murosim::geometry::Line;
using murosim::geometry::World;
using murosim::navigation::Container;
using murosim::navigation::Node;
using murosim::random::RandomGenerator;
using murosim::robot::Robot;
using murosim::robot::RobotBuilder;

constexpr float NODE_RADIUS = 0.5f;
constexpr float GOAL_RADIUS = 0.2f;
constexpr float BODY_SIZE = 0.3f;
constexpr uint64_t SEED = 42;
constexpr int ROUNDS = 20; // random draws per test, all must pick another node

std::shared_ptr<Robot> buildRobot()
{
    RobotBuilder builder;
    builder.setActuatorDifferentialSimple().setShapeRobotRectangle(BODY_SIZE, BODY_SIZE);
    return builder.build();
}

/// Three nodes far apart, all in group 0.
Container threeNodes()
{
    const std::vector<Node> nodes = {Node(0.0f, 0.0f, NODE_RADIUS, 0.0f, 0.0f, 0, 0),
                                     Node(5.0f, 5.0f, NODE_RADIUS, 0.0f, 0.0f, 1, 0),
                                     Node(10.0f, 0.0f, NODE_RADIUS, 0.0f, 0.0f, 2, 0)};
    return Container(nodes);
}

/// 20 m x 20 m empty world around the nodes, so robot placement always succeeds.
World openWorld()
{
    return World({Line(-5.0f, -5.0f, 15.0f, -5.0f), Line(15.0f, -5.0f, 15.0f, 15.0f), Line(15.0f, 15.0f, -5.0f, 15.0f),
                  Line(-5.0f, 15.0f, -5.0f, -5.0f)},
                 {});
}
} // namespace

// ============================================================================
// Select New Node Tests
// ============================================================================

// Regression: the resample condition used to be inverted, so the "new" goal
// was always the current node again (or the retry loop failed).
TEST(InitialisationSelectNewNode, SelectNewNodeAllPicksDifferentNode)
{
    const Container nodes = threeNodes();
    RandomGenerator random_generator(SEED);
    const std::shared_ptr<Robot> robot = buildRobot();

    for (int round = 0; round < ROUNDS; round++)
    {
        robot->setGoal(nodes.getNodes()[0], GOAL_RADIUS);
        const Eigen::Vector2f old_goal = robot->getGoal().getPoint();

        ASSERT_TRUE(murosim::simulation::initialisation::selectNewNodeAll(*robot, random_generator, nodes, GOAL_RADIUS))
            << "round " << round;

        // The new goal lies outside the old goal's circle and carries the requested radius
        ASSERT_GT((robot->getGoal().getPoint() - old_goal).norm(), NODE_RADIUS) << "round " << round;
        ASSERT_FLOAT_EQ(robot->getGoal().getGoalRadius(), GOAL_RADIUS) << "round " << round;
    }
}

TEST(InitialisationSelectNewNode, SelectNewNodeGroupPicksDifferentNode)
{
    const Container nodes = threeNodes();
    RandomGenerator random_generator(SEED);
    const std::shared_ptr<Robot> robot = buildRobot();

    for (int round = 0; round < ROUNDS; round++)
    {
        robot->setGoal(nodes.getNodes()[0], GOAL_RADIUS);
        const Eigen::Vector2f old_goal = robot->getGoal().getPoint();

        ASSERT_TRUE(
            murosim::simulation::initialisation::selectNewNodeGroup(*robot, random_generator, nodes, GOAL_RADIUS))
            << "round " << round;

        ASSERT_GT((robot->getGoal().getPoint() - old_goal).norm(), NODE_RADIUS) << "round " << round;
        ASSERT_EQ(robot->getGoal().getGroup(), static_cast<size_t>(0)) << "round " << round;
    }
}

// ============================================================================
// Surplus Robot Tests
// ============================================================================

// Regression: surplus robots were parked at FLT_MAX with status UNUSED but
// never marked finished, so they kept being simulated and the simulation
// could never report finished.
TEST(InitialisationSurplusRobots, SurplusRobotsAreFinished)
{
    const Container nodes = threeNodes();
    const World world = openWorld();
    RandomGenerator random_generator(SEED);
    std::vector<std::shared_ptr<Robot>> robots;
    for (int i = 0; i < 5; i++) // 5 robots, only 3 nodes
    {
        robots.push_back(buildRobot());
    }

    ASSERT_TRUE(
        murosim::simulation::initialisation::selectNodeAllRandom(robots, random_generator, nodes, world, GOAL_RADIUS));

    for (size_t i = 0; i < robots.size(); i++)
    {
        if (i < nodes.getNodes().size())
        {
            EXPECT_FALSE(robots[i]->isFinished()) << "active robot " << i;
            EXPECT_NE(robots[i]->getStatusState(), Robot::STATUS_STATE::UNUSED) << "active robot " << i;
        }
        else
        {
            EXPECT_TRUE(robots[i]->isFinished()) << "surplus robot " << i;
            EXPECT_EQ(robots[i]->getStatusState(), Robot::STATUS_STATE::UNUSED) << "surplus robot " << i;
        }
    }
}
