// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/simulation/simulation.hpp"
#include "murosim/simulation/simulation_config.hpp"
#include "murosim/simulation/initialisation.hpp"
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::actuator::Actuator;
using murosim::actuator::HolonomicSimple;
using murosim::geometry::Shape;
using murosim::geometry::World;
using murosim::navigation::Container;
using murosim::navigation::Node;
using murosim::random::RandomGenerator;
using murosim::robot::Robot;
using murosim::sensor::Lidar;
using murosim::simulation::Simulation;
using murosim::simulation::SimulationConfig;

constexpr float EPSILON = 1e-5f;

// Square world from -WORLD_HALF_SIZE to +WORLD_HALF_SIZE on both axes
constexpr float WORLD_HALF_SIZE = 5.0f;
constexpr float GRID_SCALE = 0.1f;
constexpr float BUFFER_SCALE = 0.2f;
constexpr size_t MAP_SIZE = 101; // 10 m / 0.1 m + 1 cell

// Four nodes in the corners of the world, all in group 0
constexpr float NODE_OFFSET = 3.0f;
constexpr float NODE_RADIUS = 0.5f;
constexpr size_t NODE_COUNT = 4;

constexpr float TIME_STEP = 0.1f;
constexpr float TIME_STEP_FINE = 0.01f;
constexpr float GOAL_RADIUS = 0.5f;
constexpr uint32_t SEED = 42;

// Robot: 0.3 m square body, holonomic actuator limited to 1 m/s and 1 rad/s,
// no acceleration limit, one lidar with 1 degree steps whose middle ray points
// straight ahead
constexpr float BODY_SIZE = 0.3f;
constexpr float VELOCITY_MAX = 1.0f;
constexpr float LIDAR_FOV = 1.5f * PI; // 270 degrees
constexpr size_t LIDAR_RAYS = 271;
constexpr size_t FORWARD_RAY = 135;
constexpr float LIDAR_RANGE_MIN = 0.05f;
constexpr float LIDAR_RANGE_MAX = 10.0f;
constexpr float NO_FREQUENCY = -1.0f;        // SYNCHRONIZED-only lidar
constexpr float SLOW_LIDAR_FREQUENCY = 2.0f; // one rotation per five time steps

World squareWorld(const float half_size)
{
    Shape shape;
    shape.addLine(-half_size, -half_size, half_size, -half_size); // bottom
    shape.addLine(half_size, -half_size, half_size, half_size);   // right
    shape.addLine(half_size, half_size, -half_size, half_size);   // top
    shape.addLine(-half_size, half_size, -half_size, -half_size); // left
    return World(shape);
}

Container cornerNodes()
{
    const std::vector<Node> nodes = {Node(-NODE_OFFSET, -NODE_OFFSET, NODE_RADIUS, 0.0f, 0.0f, 0, 0),
                                     Node(NODE_OFFSET, -NODE_OFFSET, NODE_RADIUS, 0.0f, 0.0f, 1, 0),
                                     Node(NODE_OFFSET, NODE_OFFSET, NODE_RADIUS, 0.0f, 0.0f, 2, 0),
                                     Node(-NODE_OFFSET, NODE_OFFSET, NODE_RADIUS, 0.0f, 0.0f, 3, 0)};
    return Container(nodes);
}

std::shared_ptr<Robot> makeRobot(const std::string &name, const float lidar_frequency = NO_FREQUENCY)
{
    const std::unique_ptr<Actuator> actuator =
        std::make_unique<HolonomicSimple>(-VELOCITY_MAX, VELOCITY_MAX, VELOCITY_MAX, VELOCITY_MAX);
    const std::vector<Lidar> lidars = {
        Lidar(Eigen::Affine2f::Identity(), LIDAR_FOV, LIDAR_RAYS, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX, lidar_frequency)};
    return std::make_shared<Robot>(name, actuator, murosim::geometry::primitives::square(BODY_SIZE), Shape(), lidars);
}

/// Distance from the centre of the square world to its wall along `angle`.
float wallDistance(const float angle)
{
    return WORLD_HALF_SIZE / std::max(std::abs(std::cos(angle)), std::abs(std::sin(angle)));
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class SimulationTest : public ::testing::Test
{
protected:
    World world = squareWorld(WORLD_HALF_SIZE);
    Container nodes = cornerNodes();
    Simulation sim{world, nodes, "test_sim", GRID_SCALE, BUFFER_SCALE, SimulationConfig::INIT_MODE::MANUAL};
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(SimulationTest, ConstructorStoresNameAndStartsEmpty)
{
    EXPECT_EQ(sim.getName(), "test_sim");
    EXPECT_EQ(sim.getRobotSize(), static_cast<size_t>(0));
    EXPECT_TRUE(sim.getRobots().empty());
    EXPECT_FALSE(sim.isFinished());
    EXPECT_FALSE(sim.isReady()); // ready only after reset()
    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(0));
}

TEST_F(SimulationTest, ConstructorStoresInitMode)
{
    const Simulation random_sim(world, nodes, "random", GRID_SCALE, BUFFER_SCALE,
                                SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);

    EXPECT_EQ(sim.getConfig().getInitMode(), SimulationConfig::INIT_MODE::MANUAL);
    EXPECT_EQ(random_sim.getConfig().getInitMode(), SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);
}

TEST_F(SimulationTest, ConstructorWithConfigAppliesValues)
{
    SimulationConfig config;
    config.setTimeStep(0.05f);
    config.setTimeIntervalCollision(0.001f);
    config.setTimeIntervalActuator(0.002f);
    config.setTimeIntervalSensor(0.003f);
    config.setGridScale(0.05f);
    config.setBufferScale(0.3f);
    config.setGoalRadius(1.5f);
    config.setInitMode(SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);
    config.setSensorUpdateMode(SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    config.setSeed(SEED);

    const Simulation configured(world, nodes, "configured", config);

    EXPECT_EQ(configured.getName(), "configured");
    EXPECT_FLOAT_EQ(configured.getConfig().getTimeStep(), 0.05f);
    EXPECT_FLOAT_EQ(configured.getConfig().getTimeIntervalCollision(), 0.001f);
    EXPECT_FLOAT_EQ(configured.getConfig().getTimeIntervalActuator(), 0.002f);
    EXPECT_FLOAT_EQ(configured.getConfig().getTimeIntervalSensor(), 0.003f);
    EXPECT_FLOAT_EQ(configured.getConfig().getGridScale(), 0.05f);
    EXPECT_FLOAT_EQ(configured.getConfig().getBufferScale(), 0.3f);
    EXPECT_FLOAT_EQ(configured.getConfig().getGoalRadius(), 1.5f);
    EXPECT_EQ(configured.getConfig().getInitMode(), SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);
    EXPECT_EQ(configured.getConfig().getSensorUpdateMode(), SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    EXPECT_EQ(configured.getConfig().getSeed(), SEED);
}

TEST_F(SimulationTest, ConstructorWithConfigUsesFinerGrid)
{
    SimulationConfig config;
    config.setGridScale(GRID_SCALE / 2.0f);

    const Simulation fine(world, nodes, "fine", config);

    EXPECT_EQ(fine.getGridMap().sizeX(), 2 * (MAP_SIZE - 1) + 1);
}

TEST_F(SimulationTest, CopyConstructorCopiesConfigAndRobots)
{
    SimulationConfig config = sim.getConfig();
    config.setTimeStep(TIME_STEP_FINE);
    config.setSensorUpdateMode(SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    sim.setConfig(config);
    sim.addRobot(makeRobot("robot"));

    const Simulation copy(sim);

    EXPECT_EQ(copy.getName(), "test_sim");
    EXPECT_FLOAT_EQ(copy.getConfig().getTimeStep(), TIME_STEP_FINE);
    EXPECT_EQ(copy.getConfig().getSensorUpdateMode(), SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    EXPECT_EQ(copy.getRobotSize(), static_cast<size_t>(1));
    EXPECT_EQ(copy.getNodes().getNodes().size(), NODE_COUNT);
}

TEST_F(SimulationTest, MoveConstructorKeepsName)
{
    Simulation source(world, nodes, "movable", GRID_SCALE, BUFFER_SCALE);
    source.addRobot(makeRobot("robot"));

    const Simulation moved(std::move(source));

    EXPECT_EQ(moved.getName(), "movable");
    EXPECT_EQ(moved.getRobotSize(), static_cast<size_t>(1));
}

TEST_F(SimulationTest, EmptyNodesGiveNoGroupsAndNoPossibleRobots)
{
    const Container no_nodes;

    const Simulation empty(world, no_nodes, "empty", GRID_SCALE, BUFFER_SCALE);

    EXPECT_EQ(empty.getGroupSize(), static_cast<size_t>(0));
    EXPECT_EQ(empty.getPossibleRobotSize(), static_cast<size_t>(0));
}

// ============================================================================
// Getter Tests
// ============================================================================

TEST_F(SimulationTest, GetWorldIsTheFourWalls)
{
    EXPECT_EQ(sim.getWorld().getLineSize(), static_cast<size_t>(4));
    EXPECT_EQ(sim.getWorld().getCircleSize(), static_cast<size_t>(0));
}

TEST_F(SimulationTest, GetNodesHoldsTheCornerNodes)
{
    EXPECT_EQ(sim.getNodes().getNodes().size(), NODE_COUNT);
    EXPECT_FLOAT_EQ(sim.getNodes().getNodes()[0].getX(), -NODE_OFFSET);
}

TEST_F(SimulationTest, GetGridMapCoversTheWorld)
{
    EXPECT_EQ(sim.getGridMap().sizeX(), MAP_SIZE);
    EXPECT_EQ(sim.getGridMap().sizeY(), MAP_SIZE);
    EXPECT_FLOAT_EQ(sim.getGridMap().getGridScale(), GRID_SCALE);
}

TEST_F(SimulationTest, GetPossibleRobotSizeIsTheNodeCount)
{
    EXPECT_EQ(sim.getPossibleRobotSize(), NODE_COUNT);
}

TEST_F(SimulationTest, GetGroupSizeCountsNodeGroups)
{
    EXPECT_EQ(sim.getGroupSize(), static_cast<size_t>(1)); // all nodes in group 0
}

// ============================================================================
// Config Tests
// ============================================================================

TEST_F(SimulationTest, SetConfigReplacesTimeParameters)
{
    SimulationConfig config = sim.getConfig();
    config.setTimeStep(0.02f);
    config.setTimeIntervalCollision(0.005f);
    config.setTimeIntervalActuator(0.01f);
    config.setTimeIntervalSensor(0.02f);

    sim.setConfig(config);

    EXPECT_FLOAT_EQ(sim.getConfig().getTimeStep(), 0.02f);
    EXPECT_FLOAT_EQ(sim.getConfig().getTimeIntervalCollision(), 0.005f);
    EXPECT_FLOAT_EQ(sim.getConfig().getTimeIntervalActuator(), 0.01f);
    EXPECT_FLOAT_EQ(sim.getConfig().getTimeIntervalSensor(), 0.02f);
}

TEST_F(SimulationTest, SetConfigReplacesGoalParameters)
{
    SimulationConfig config = sim.getConfig();
    config.setGoalRadius(1.5f);
    config.setTimeOnGoal(2.0f);
    config.setUseNewGoal(true);

    sim.setConfig(config);

    EXPECT_FLOAT_EQ(sim.getConfig().getGoalRadius(), 1.5f);
    EXPECT_FLOAT_EQ(sim.getConfig().getTimeOnGoal(), 2.0f);
    EXPECT_TRUE(sim.getConfig().getUseNewGoal());
}

TEST_F(SimulationTest, SetConfigReplacesInitMode)
{
    SimulationConfig config = sim.getConfig();
    config.setInitMode(SimulationConfig::INIT_MODE::FREE_ALL_RANDOM);
    sim.setConfig(config);
    EXPECT_EQ(sim.getConfig().getInitMode(), SimulationConfig::INIT_MODE::FREE_ALL_RANDOM);

    config.setInitMode(SimulationConfig::INIT_MODE::NODE_GROUP_RANDOM);
    sim.setConfig(config);
    EXPECT_EQ(sim.getConfig().getInitMode(), SimulationConfig::INIT_MODE::NODE_GROUP_RANDOM);
}

TEST_F(SimulationTest, SetConfigReplacesSensorUpdateMode)
{
    EXPECT_EQ(sim.getConfig().getSensorUpdateMode(), SimulationConfig::SENSOR_UPDATE_MODE::SYNCHRONIZED);

    SimulationConfig config = sim.getConfig();
    config.setSensorUpdateMode(SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    sim.setConfig(config);
    EXPECT_EQ(sim.getConfig().getSensorUpdateMode(), SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);

    config.setSensorUpdateMode(SimulationConfig::SENSOR_UPDATE_MODE::SYNCHRONIZED);
    sim.setConfig(config);
    EXPECT_EQ(sim.getConfig().getSensorUpdateMode(), SimulationConfig::SENSOR_UPDATE_MODE::SYNCHRONIZED);
}

// ============================================================================
// Robot Management Tests
// ============================================================================

TEST_F(SimulationTest, AddRobotAppendsAndReturnsTrue)
{
    const bool added = sim.addRobot(makeRobot("robot1"));

    EXPECT_TRUE(added);
    EXPECT_EQ(sim.getRobotSize(), static_cast<size_t>(1));
    EXPECT_EQ(sim.getRobots().size(), static_cast<size_t>(1));
}

TEST_F(SimulationTest, AddRobotKeepsInsertionOrder)
{
    sim.addRobot(makeRobot("alpha"));
    sim.addRobot(makeRobot("beta"));
    sim.addRobot(makeRobot("gamma"));

    ASSERT_EQ(sim.getRobotSize(), static_cast<size_t>(3));
    EXPECT_EQ(sim.getRobotAt(0).getName(), "alpha");
    EXPECT_EQ(sim.getRobotAt(1).getName(), "beta");
    EXPECT_EQ(sim.getRobotAt(2).getName(), "gamma");
}

TEST_F(SimulationTest, AddSameRobotTwiceIsRejected)
{
    const std::shared_ptr<Robot> robot = makeRobot("duplicate");

    const bool first_add = sim.addRobot(robot);
    const bool second_add = sim.addRobot(robot);

    EXPECT_TRUE(first_add);
    EXPECT_FALSE(second_add);
    EXPECT_EQ(sim.getRobotSize(), static_cast<size_t>(1));
}

TEST_F(SimulationTest, AddRobotMarksSimulationNotReady)
{
    sim.addRobot(makeRobot("robot"));
    sim.reset();
    ASSERT_TRUE(sim.isReady());

    sim.addRobot(makeRobot("another"));

    EXPECT_FALSE(sim.isReady());
}

TEST_F(SimulationTest, RemoveAllRobotsEmptiesSimulation)
{
    sim.addRobot(makeRobot("robot1"));
    sim.addRobot(makeRobot("robot2"));

    sim.removeAllRobots();

    EXPECT_EQ(sim.getRobotSize(), static_cast<size_t>(0));
    EXPECT_TRUE(sim.getRobots().empty());
}

// ============================================================================
// Robot Access Tests
// ============================================================================

TEST_F(SimulationTest, GetRobotAtConst)
{
    sim.addRobot(makeRobot("alpha"));
    sim.addRobot(makeRobot("beta"));
    const Simulation &const_sim = sim;

    EXPECT_EQ(const_sim.getRobotAt(0).getName(), "alpha");
    EXPECT_EQ(const_sim.getRobotAt(1).getName(), "beta");
}

TEST_F(SimulationTest, GetRobotAtMutableChangesTheStoredRobot)
{
    sim.addRobot(makeRobot("robot"));

    sim.getRobotAt(0).setPose(5.0f, 3.0f, PI_2);

    EXPECT_FLOAT_EQ(sim.getRobots()[0]->getPosition().x(), 5.0f);
    EXPECT_FLOAT_EQ(sim.getRobots()[0]->getPosition().y(), 3.0f);
    EXPECT_NEAR(sim.getRobots()[0]->getRotationAngle(), PI_2, EPSILON);
}

TEST_F(SimulationTest, GetRobotPtrAtSharesTheAddedRobot)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    sim.addRobot(robot);
    const Simulation &const_sim = sim;

    EXPECT_EQ(const_sim.getRobotPtrAt(0), robot);
    EXPECT_EQ(sim.getRobotPtrAt(0), robot);
    EXPECT_EQ(sim.getRobots()[0], robot);
}

TEST_F(SimulationTest, GetRobotPtrAtMutableChangesTheStoredRobot)
{
    sim.addRobot(makeRobot("robot"));

    sim.getRobotPtrAt(0)->setPose(1.0f, 2.0f, 0.0f);

    EXPECT_FLOAT_EQ(sim.getRobotAt(0).getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(sim.getRobotAt(0).getPosition().y(), 2.0f);
}

TEST_F(SimulationTest, GetRobotAtAgreesWithGetRobots)
{
    sim.addRobot(makeRobot("first"));
    sim.addRobot(makeRobot("second"));

    for (size_t i = 0; i < sim.getRobotSize(); i++)
    {
        EXPECT_EQ(&sim.getRobotAt(i), sim.getRobots()[i].get()) << "robot " << i;
    }
}

// ============================================================================
// Simulate Tests
// ============================================================================

TEST_F(SimulationTest, SimulateWithoutRobotsCountsSteps)
{
    sim.reset();

    sim.simulate();
    sim.simulate();

    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(2));
    EXPECT_TRUE(sim.isFinished()); // no robot left to simulate
}

TEST_F(SimulationTest, FirstSimulateOnUnreadySimulationOnlyResets)
{
    sim.addRobot(makeRobot("robot"));

    sim.simulate(); // not ready: performs the reset, which does not count as a step

    EXPECT_TRUE(sim.isReady());
    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(0));

    sim.simulate();

    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(1));
}

TEST_F(SimulationTest, ResetComputesInitialStateWithoutCountingSteps)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    sim.addRobot(robot);

    sim.reset();

    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(0));
    EXPECT_EQ(robot->getSimulatedStepCount(), static_cast<size_t>(0));
    EXPECT_TRUE(robot->hasStates()); // the initial observation is recorded
    EXPECT_FALSE(sim.isFinished());
}

TEST_F(SimulationTest, SimulateCountsStepsForSimulationAndRobots)
{
    SimulationConfig config = sim.getConfig();
    config.setTimeStep(TIME_STEP_FINE);
    sim.setConfig(config);
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    sim.addRobot(robot);
    sim.reset();

    for (int i = 0; i < 10; i++)
    {
        sim.simulate();
    }

    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(10));
    EXPECT_EQ(robot->getSimulatedStepCount(), static_cast<size_t>(10));
    EXPECT_EQ(robot->getStateSize(), static_cast<size_t>(11)); // initial state + 10 steps
}

TEST_F(SimulationTest, SimulateMovesRobotByCommandedVelocity)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(NODE_OFFSET, 0.0f, GOAL_RADIUS);
    sim.addRobot(robot);
    sim.reset();
    robot->setActuatorCommandVelocity(0.5f, 0.0f, 0.0f); // after reset, which clears commands

    sim.simulate();

    EXPECT_NEAR(robot->getPosition().x(), 0.5f * TIME_STEP, EPSILON);
    EXPECT_NEAR(robot->getPosition().y(), 0.0f, EPSILON);
}

TEST_F(SimulationTest, ResetRestartsStepCount)
{
    sim.addRobot(makeRobot("robot"));
    sim.reset();
    sim.simulate();
    sim.simulate();
    ASSERT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(2));

    sim.reset();

    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(0));
    EXPECT_FALSE(sim.isFinished());
}

TEST_F(SimulationTest, ResetClearsActuatorCommands)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);
    sim.addRobot(robot);

    sim.reset(); // resets every robot, including its actuator

    EXPECT_FLOAT_EQ(robot->getActuatorCommandVelocity().x(), 0.0f);
    EXPECT_FLOAT_EQ(robot->getPosition().x(), 0.0f); // the reset step did not move it
}

TEST_F(SimulationTest, VerySmallTimeStepCountsEveryStep)
{
    SimulationConfig config = sim.getConfig();
    config.setTimeStep(0.001f); // below the default actuator and collision intervals
    sim.setConfig(config);
    sim.addRobot(makeRobot("robot"));
    sim.reset();

    for (int i = 0; i < 100; i++)
    {
        sim.simulate();
    }

    EXPECT_EQ(sim.getSimulatedStepCount(), static_cast<size_t>(100));
}

TEST_F(SimulationTest, RobotOutsideWorldSeesNothingAndDoesNotCollide)
{
    const std::shared_ptr<Robot> robot = makeRobot("outside");
    robot->setPose(20.0f * WORLD_HALF_SIZE, 20.0f * WORLD_HALF_SIZE, 0.0f);
    sim.addRobot(robot);

    sim.reset();
    sim.simulate();

    EXPECT_EQ(robot->getStatusCollision(), Robot::STATUS_COLLISION::NONE);
    const Lidar &lidar = robot->getLidarAt(0);
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_EQ(lidar.getLaserDistance(i), std::numeric_limits<float>::infinity()) << "ray " << i; // no return
    }
}

// ============================================================================
// Lidar Tests
// ============================================================================

TEST_F(SimulationTest, LidarMeasuresDistanceToWalls)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(0.0f, 0.0f, 0.0f); // centre of the room, facing +x
    sim.addRobot(robot);

    sim.reset(); // performs the first sweep

    const Lidar &lidar = robot->getLidarAt(0);
    ASSERT_EQ(lidar.getLaserSize(), LIDAR_RAYS);
    EXPECT_NEAR(lidar.getLaserDistance(FORWARD_RAY), WORLD_HALF_SIZE, EPSILON);
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        const float angle = lidar.getAngleMin() + static_cast<float>(i) * lidar.getAngleStep();
        ASSERT_NEAR(lidar.getLaserDistance(i), wallDistance(angle), EPSILON) << "ray " << i;
    }
}

TEST_F(SimulationTest, LidarFollowsRobotRotation)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(0.0f, 0.0f, PI_2); // facing +y
    sim.addRobot(robot);

    sim.reset();

    const Lidar &lidar = robot->getLidarAt(0);
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        const float angle = PI_2 + lidar.getAngleMin() + static_cast<float>(i) * lidar.getAngleStep();
        ASSERT_NEAR(lidar.getLaserDistance(i), wallDistance(angle), EPSILON) << "ray " << i;
    }
}

// Regression: in CONTINUOUS mode a reset kept each lidar's scan phase and
// half-built scan, so a lidar slower than the time step still showed the
// previous episode's surroundings after the reset. A reset now starts the
// episode with a complete scan of the new pose.
TEST_F(SimulationTest, ContinuousResetStartsWithACompleteScanOfTheNewPose)
{
    SimulationConfig config = sim.getConfig();
    config.setSensorUpdateMode(SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    sim.setConfig(config);

    const std::shared_ptr<Robot> robot = makeRobot("robot", SLOW_LIDAR_FREQUENCY);
    robot->setPose(WORLD_HALF_SIZE - 1.0f, 0.0f, 0.0f); // first episode: near the right wall
    sim.addRobot(robot);
    sim.reset();
    for (int i = 0; i < 7; i++) // 0.7 s: the beam is in the middle of a rotation
    {
        sim.simulate();
    }

    robot->setPose(0.0f, 0.0f, 0.0f); // next episode: centre of the room
    sim.reset();

    const Lidar &lidar = robot->getLidarAt(0);
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        const float angle = lidar.getAngleMin() + static_cast<float>(i) * lidar.getAngleStep();
        ASSERT_NEAR(lidar.getLaserDistance(i), wallDistance(angle), EPSILON) << "ray " << i;
    }
}

// ============================================================================
// Collision Tests
// ============================================================================

TEST_F(SimulationTest, RobotDrivingIntoWallCollidesAndFinishes)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(WORLD_HALF_SIZE - 0.5f, 0.0f, 0.0f); // half a metre from the right wall
    sim.addRobot(robot);
    sim.reset();
    robot->setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);

    for (int i = 0; i < 20; i++) // 2 s, the wall is reached after 0.35 s
    {
        sim.simulate();
    }

    EXPECT_EQ(robot->getStatusCollision(), Robot::STATUS_COLLISION::WORLD);
    EXPECT_TRUE(robot->isFinished());
    EXPECT_TRUE(sim.isFinished());
    EXPECT_LT(robot->getPosition().x(), WORLD_HALF_SIZE); // stopped at the wall, not through it
}

TEST_F(SimulationTest, RobotsDrivingIntoEachOtherCollideAndFinish)
{
    const std::shared_ptr<Robot> robot1 = makeRobot("robot1");
    const std::shared_ptr<Robot> robot2 = makeRobot("robot2");
    robot1->setPose(-1.0f, 0.0f, 0.0f);
    robot2->setPose(1.0f, 0.0f, PI);
    sim.addRobot(robot1);
    sim.addRobot(robot2);
    sim.reset();
    robot1->setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);
    robot2->setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);

    for (int i = 0; i < 30; i++) // 3 s, they meet after 0.85 s
    {
        sim.simulate();
    }

    EXPECT_EQ(robot1->getStatusCollision(), Robot::STATUS_COLLISION::ROBOT);
    EXPECT_EQ(robot2->getStatusCollision(), Robot::STATUS_COLLISION::ROBOT);
    EXPECT_TRUE(robot1->isFinished());
    EXPECT_TRUE(robot2->isFinished());
    EXPECT_TRUE(sim.isFinished());
    EXPECT_LT(robot1->getPosition().x(), robot2->getPosition().x()); // they did not pass through each other
}

TEST_F(SimulationTest, CollidedRobotStopsMoving)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(WORLD_HALF_SIZE - 0.5f, 0.0f, 0.0f);
    sim.addRobot(robot);
    sim.reset();
    robot->setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);
    for (int i = 0; i < 10; i++)
    {
        sim.simulate();
    }
    ASSERT_EQ(robot->getStatusCollision(), Robot::STATUS_COLLISION::WORLD);
    const float x_at_collision = robot->getPosition().x();

    sim.simulate();

    EXPECT_FLOAT_EQ(robot->getPosition().x(), x_at_collision);
}

// ============================================================================
// Goal Tests
// ============================================================================

TEST_F(SimulationTest, RobotSearchesGoalUntilInsideGoalRadius)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(NODE_OFFSET, 0.0f, GOAL_RADIUS); // 3 m ahead
    sim.addRobot(robot);
    sim.reset();
    robot->setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);

    sim.simulate();

    EXPECT_EQ(robot->getStatusState(), Robot::STATUS_STATE::SEARCH_GOAL);
    EXPECT_FALSE(robot->isFinished());
}

TEST_F(SimulationTest, RobotReachingGoalFinishesInstantly)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(NODE_OFFSET, 0.0f, GOAL_RADIUS);
    sim.addRobot(robot);
    sim.reset();
    robot->setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);

    for (int i = 0; i < 40; i++) // 4 s, the goal radius is entered after 2.5 s
    {
        sim.simulate();
    }

    // Default time on goal is -1: the goal counts as reached on first contact
    EXPECT_EQ(robot->getStatusState(), Robot::STATUS_STATE::REACHED_GOAL);
    EXPECT_TRUE(robot->isFinished());
    EXPECT_TRUE(sim.isFinished());
    EXPECT_LT((robot->getPosition() - robot->getGoal().getPoint()).norm(), GOAL_RADIUS);
}

TEST_F(SimulationTest, TimeOnGoalDelaysReachedGoal)
{
    SimulationConfig config = sim.getConfig();
    config.setTimeOnGoal(0.5f); // five steps inside the radius
    sim.setConfig(config);
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setPose(0.0f, 0.0f, 0.0f);
    robot->setGoal(0.0f, 0.0f, GOAL_RADIUS); // starts on its goal
    sim.addRobot(robot);
    sim.reset();

    for (int i = 0; i < 4; i++)
    {
        sim.simulate();
    }
    EXPECT_EQ(robot->getStatusState(), Robot::STATUS_STATE::ON_GOAL);
    EXPECT_FALSE(robot->isFinished());

    sim.simulate();

    EXPECT_EQ(robot->getStatusState(), Robot::STATUS_STATE::REACHED_GOAL);
    EXPECT_TRUE(robot->isFinished());
}

TEST_F(SimulationTest, FourRobotsConvergingOnCentreAllReachGoal)
{
    SimulationConfig config = sim.getConfig();
    config.setTimeStep(TIME_STEP_FINE);
    sim.setConfig(config);
    const float start_radius = 2.0f;
    for (int i = 0; i < 4; i++)
    {
        const std::shared_ptr<Robot> robot = makeRobot("robot" + std::to_string(i));
        const float angle = PI_2 * static_cast<float>(i);
        robot->setPose(start_radius * std::cos(angle), start_radius * std::sin(angle), angle + PI); // facing centre
        robot->setGoal(0.0f, 0.0f, GOAL_RADIUS);
        sim.addRobot(robot);
    }
    sim.reset();
    for (size_t i = 0; i < sim.getRobotSize(); i++)
    {
        sim.getRobotAt(i).setActuatorCommandVelocity(VELOCITY_MAX, 0.0f, 0.0f);
    }

    for (int i = 0; i < 300; i++) // 3 s, the goal radius is entered after 1.5 s
    {
        sim.simulate();
    }

    EXPECT_TRUE(sim.isFinished());
    for (size_t i = 0; i < sim.getRobotSize(); i++)
    {
        EXPECT_EQ(sim.getRobotAt(i).getStatusState(), Robot::STATUS_STATE::REACHED_GOAL) << "robot " << i;
        EXPECT_EQ(sim.getRobotAt(i).getStatusCollision(), Robot::STATUS_COLLISION::NONE) << "robot " << i;
        EXPECT_TRUE(sim.getRobotAt(i).isFinished()) << "robot " << i;
    }
}

// ============================================================================
// Initialisation Tests
// ============================================================================

TEST_F(SimulationTest, SelectNodeAllRandomPlacesRobotsOnNodes)
{
    std::vector<std::shared_ptr<Robot>> robots = {makeRobot("robot1"), makeRobot("robot2")};
    RandomGenerator rng(SEED);

    const bool success =
        murosim::simulation::initialisation::selectNodeAllRandom(robots, rng, nodes, world, GOAL_RADIUS);

    ASSERT_TRUE(success);
    for (size_t i = 0; i < robots.size(); i++)
    {
        float nearest_node = std::numeric_limits<float>::max();
        for (const Node &node : nodes.getNodes())
        {
            nearest_node = std::min(nearest_node, (robots[i]->getPosition() - node.getPoint()).norm());
        }
        EXPECT_LE(nearest_node, NODE_RADIUS) << "robot " << i; // inside a node circle
        EXPECT_FLOAT_EQ(robots[i]->getGoal().getGoalRadius(), GOAL_RADIUS) << "robot " << i;
        EXPECT_FALSE(robots[i]->isFinished()) << "robot " << i;
    }
}

TEST_F(SimulationTest, SelectNewNodeAllPicksAnotherNode)
{
    const std::shared_ptr<Robot> robot = makeRobot("robot");
    robot->setGoal(nodes.getNodes()[0], GOAL_RADIUS);
    RandomGenerator rng(SEED);

    const bool success = murosim::simulation::initialisation::selectNewNodeAll(*robot, rng, nodes, GOAL_RADIUS);

    ASSERT_TRUE(success);
    const Eigen::Vector2f goal = robot->getGoal().getPoint();
    EXPECT_GT((goal - nodes.getNodes()[0].getPoint()).norm(), NODE_RADIUS); // not the old node
    bool on_a_node = false;
    for (const Node &node : nodes.getNodes())
    {
        on_a_node = on_a_node || (goal.x() == node.getX() && goal.y() == node.getY());
    }
    EXPECT_TRUE(on_a_node); // exactly one of the container's nodes
}

TEST_F(SimulationTest, ResetWithNodeAllRandomPlacesRobotsOnNodes)
{
    SimulationConfig config = sim.getConfig();
    config.setInitMode(SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);
    config.setSeed(SEED);
    sim.setConfig(config);
    sim.addRobot(makeRobot("robot1"));
    sim.addRobot(makeRobot("robot2"));

    sim.reset();

    EXPECT_TRUE(sim.isReady());
    for (size_t i = 0; i < sim.getRobotSize(); i++)
    {
        float nearest_node = std::numeric_limits<float>::max();
        for (const Node &node : nodes.getNodes())
        {
            nearest_node = std::min(nearest_node, (sim.getRobotAt(i).getPosition() - node.getPoint()).norm());
        }
        EXPECT_LE(nearest_node, NODE_RADIUS) << "robot " << i;
    }
}

// ============================================================================
// Config Value Tests
// ============================================================================

TEST(SimulationConfigValues, DefaultValues)
{
    const SimulationConfig config;

    EXPECT_FLOAT_EQ(config.getTimeStep(), 0.1f);
    EXPECT_FLOAT_EQ(config.getTimeIntervalCollision(), 1e-2f);
    EXPECT_FLOAT_EQ(config.getTimeIntervalActuator(), 1e-2f);
    EXPECT_FLOAT_EQ(config.getTimeIntervalSensor(), 1e-2f);
    EXPECT_FLOAT_EQ(config.getTimeOnGoal(), -1.0f); // instant finish
    EXPECT_FALSE(config.getPersistentGoalTimer());
    EXPECT_FLOAT_EQ(config.getGridScale(), 0.1f);
    EXPECT_FLOAT_EQ(config.getBufferScale(), 0.2f);
    EXPECT_FLOAT_EQ(config.getGoalRadius(), -1.0f); // use the node radius
    EXPECT_FALSE(config.getUseNewGoal());
    EXPECT_EQ(config.getInitMode(), SimulationConfig::INIT_MODE::MANUAL);
    EXPECT_EQ(config.getSensorUpdateMode(), SimulationConfig::SENSOR_UPDATE_MODE::SYNCHRONIZED);
    EXPECT_EQ(config.getPlannerType(), murosim::planning::Planner::TYPE::NONE);
}

TEST(SimulationConfigValues, SettersStoreValues)
{
    SimulationConfig config;

    config.setTimeStep(0.05f);
    config.setTimeIntervalCollision(0.001f);
    config.setTimeIntervalActuator(0.002f);
    config.setTimeIntervalSensor(0.003f);
    config.setTimeOnGoal(2.0f);
    config.setPersistentGoalTimer(true);
    config.setGridScale(0.05f);
    config.setBufferScale(0.3f);
    config.setGoalRadius(1.5f);
    config.setUseNewGoal(true);
    config.setInitMode(SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);
    config.setSensorUpdateMode(SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    config.setPlannerType(murosim::planning::Planner::TYPE::ASTAR8);
    config.setSeed(SEED);

    EXPECT_FLOAT_EQ(config.getTimeStep(), 0.05f);
    EXPECT_FLOAT_EQ(config.getTimeIntervalCollision(), 0.001f);
    EXPECT_FLOAT_EQ(config.getTimeIntervalActuator(), 0.002f);
    EXPECT_FLOAT_EQ(config.getTimeIntervalSensor(), 0.003f);
    EXPECT_FLOAT_EQ(config.getTimeOnGoal(), 2.0f);
    EXPECT_TRUE(config.getPersistentGoalTimer());
    EXPECT_FLOAT_EQ(config.getGridScale(), 0.05f);
    EXPECT_FLOAT_EQ(config.getBufferScale(), 0.3f);
    EXPECT_FLOAT_EQ(config.getGoalRadius(), 1.5f);
    EXPECT_TRUE(config.getUseNewGoal());
    EXPECT_EQ(config.getInitMode(), SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);
    EXPECT_EQ(config.getSensorUpdateMode(), SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS);
    EXPECT_EQ(config.getPlannerType(), murosim::planning::Planner::TYPE::ASTAR8);
    EXPECT_EQ(config.getSeed(), SEED);
}

TEST(SimulationConfigValues, NonPositiveTimeStepThrows)
{
    SimulationConfig config;

    EXPECT_THROW(config.setTimeStep(0.0f), std::invalid_argument);
    EXPECT_THROW(config.setTimeStep(-0.1f), std::invalid_argument);
}

TEST(SimulationConfigValues, TooSmallIntervalsThrow)
{
    SimulationConfig config;
    const float too_small = SimulationConfig::MINIMUM_TIME_INTERVAL / 2.0f;

    EXPECT_THROW(config.setTimeIntervalCollision(too_small), std::invalid_argument);
    EXPECT_THROW(config.setTimeIntervalActuator(too_small), std::invalid_argument);
    EXPECT_THROW(config.setTimeIntervalSensor(too_small), std::invalid_argument);
}

// ============================================================================
// Init Mode String Tests
// ============================================================================

TEST(SimulationConfigInitMode, ToStringNamesEveryValue)
{
    EXPECT_EQ(SimulationConfig::initModeToString(SimulationConfig::INIT_MODE::MANUAL), "MANUAL");
    EXPECT_EQ(SimulationConfig::initModeToString(SimulationConfig::INIT_MODE::FREE_ALL_RANDOM), "FREE_ALL_RANDOM");
    EXPECT_EQ(SimulationConfig::initModeToString(SimulationConfig::INIT_MODE::NODE_ALL_RANDOM), "NODE_ALL_RANDOM");
    EXPECT_EQ(SimulationConfig::initModeToString(SimulationConfig::INIT_MODE::NODE_GROUP_RANDOM),
              "NODE_GROUP_RANDOM");
}

TEST(SimulationConfigInitMode, FromStringRoundTrips)
{
    const std::vector<SimulationConfig::INIT_MODE> modes = {
        SimulationConfig::INIT_MODE::MANUAL, SimulationConfig::INIT_MODE::FREE_ALL_RANDOM,
        SimulationConfig::INIT_MODE::NODE_ALL_RANDOM, SimulationConfig::INIT_MODE::NODE_GROUP_RANDOM};

    for (const SimulationConfig::INIT_MODE mode : modes)
    {
        EXPECT_EQ(SimulationConfig::initModeFromString(SimulationConfig::initModeToString(mode)), mode)
            << SimulationConfig::initModeToString(mode);
    }
}

TEST(SimulationConfigInitMode, UnknownStringFallsBackToManual)
{
    EXPECT_EQ(SimulationConfig::initModeFromString("no_such_mode"), SimulationConfig::INIT_MODE::MANUAL);
}

TEST(SimulationConfigInitMode, MemberToStringUsesCurrentMode)
{
    SimulationConfig config;
    config.setInitMode(SimulationConfig::INIT_MODE::NODE_ALL_RANDOM);

    EXPECT_EQ(config.initModeToString(), "NODE_ALL_RANDOM");
}
