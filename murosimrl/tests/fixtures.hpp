// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_TESTS_FIXTURES_HPP_
#define MUROSIMRL_TESTS_FIXTURES_HPP_

#include <gtest/gtest.h>
#include "murosim/actuator/differential_simple.hpp"
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/geometry/world.hpp"
#include "murosim/navigation/container.hpp"
#include "murosim/robot/robot.hpp"
#include "murosim/sensor/lidar.hpp"
#include "murosim/simulation/simulation.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

/// Shared test world, robots and simulations for the murosimrl tests.
///
/// Every observation and reward is exercised in the same setting: a square
/// world of WORLD_SIZE metres around the origin, a holonomic robot with unit
/// velocity limits and, where needed, one full-circle lidar with LIDAR_RAYS
/// rays. The geometry is simple enough that every expected value can be
/// written in closed form (see wallDistance()).
namespace fixtures
{
using murosim::math::PI;
constexpr float EPSILON = 1e-5f;

constexpr float WORLD_SIZE = 10.0f;
constexpr float WORLD_HALF_SIZE = WORLD_SIZE / 2.0f;
constexpr float GRID_SCALE = 0.1f;
constexpr float BUFFER_SCALE = 0.2f;
constexpr float TIME_STEP = 0.1f; // SimulationConfig default

constexpr float VELOCITY_MAX = 1.0f; // linear and angular, symmetric
constexpr float ACCELERATION_LINEAR = 0.5f;
constexpr float ACCELERATION_ANGULAR = 0.25f;
constexpr float BODY_SIZE = 0.3f;

constexpr size_t LIDAR_RAYS = 16;
constexpr float LIDAR_FOV = 2.0f * PI;
constexpr float LIDAR_RANGE_MIN = 0.05f;
constexpr float LIDAR_RANGE_MAX = 10.0f;

using RobotList = std::vector<std::shared_ptr<murosim::robot::Robot>>;
using SimulationList = std::vector<std::shared_ptr<murosim::simulation::Simulation>>;

/// Square world of `size` metres centred on the origin.
inline murosim::geometry::World squareWorld(const float size = WORLD_SIZE)
{
    return murosim::geometry::World(murosim::geometry::primitives::square(size));
}

/// Two nodes in opposite corners, both in group 0.
inline murosim::navigation::Container twoNodes()
{
    const std::vector<murosim::navigation::Node> nodes = {
        murosim::navigation::Node(-3.0f, -3.0f, 0.5f, 0.0f, 0.0f, 0, 0),
        murosim::navigation::Node(3.0f, 3.0f, 0.5f, 0.0f, 0.0f, 1, 0)};
    return murosim::navigation::Container(nodes);
}

/// One full-circle lidar mounted at the robot origin.
inline std::vector<murosim::sensor::Lidar> oneLidar()
{
    return {murosim::sensor::Lidar(Eigen::Affine2f::Identity(), LIDAR_FOV, LIDAR_RAYS, LIDAR_RANGE_MIN,
                                   LIDAR_RANGE_MAX)};
}

/// Holonomic robot (X_Y_ANGULAR) with symmetric unit velocity limits and,
/// with `acceleration_limits`, the fixture acceleration limits.
/// @param linear_x_min Minimum x velocity, to build an asymmetric range.
inline std::shared_ptr<murosim::robot::Robot> holonomicRobot(const std::string &name, const bool with_lidar = false,
                                                             const bool acceleration_limits = false,
                                                             const float linear_x_min = -VELOCITY_MAX)
{
    const std::unique_ptr<murosim::actuator::Actuator> actuator =
        acceleration_limits
            ? std::make_unique<murosim::actuator::HolonomicSimple>(linear_x_min, VELOCITY_MAX, VELOCITY_MAX,
                                                                   VELOCITY_MAX, ACCELERATION_LINEAR,
                                                                   ACCELERATION_ANGULAR)
            : std::make_unique<murosim::actuator::HolonomicSimple>(linear_x_min, VELOCITY_MAX, VELOCITY_MAX,
                                                                   VELOCITY_MAX);
    const std::vector<murosim::sensor::Lidar> lidars = with_lidar ? oneLidar() : std::vector<murosim::sensor::Lidar>();
    return std::make_shared<murosim::robot::Robot>(name, actuator, murosim::geometry::primitives::square(BODY_SIZE),
                                                   murosim::geometry::Shape(), lidars);
}

/// Differential robot (X_ANGULAR) with symmetric unit velocity limits.
inline std::shared_ptr<murosim::robot::Robot> differentialRobot(const std::string &name, const bool with_lidar = false)
{
    const std::unique_ptr<murosim::actuator::Actuator> actuator =
        std::make_unique<murosim::actuator::DifferentialSimple>(-VELOCITY_MAX, VELOCITY_MAX, VELOCITY_MAX);
    const std::vector<murosim::sensor::Lidar> lidars = with_lidar ? oneLidar() : std::vector<murosim::sensor::Lidar>();
    return std::make_shared<murosim::robot::Robot>(name, actuator, murosim::geometry::primitives::square(BODY_SIZE),
                                                   murosim::geometry::Shape(), lidars);
}

/// Simulation of the square world in MANUAL init mode, without robots.
inline std::shared_ptr<murosim::simulation::Simulation> makeSimulation(const std::string &name,
                                                                       const float world_size = WORLD_SIZE)
{
    return std::make_shared<murosim::simulation::Simulation>(squareWorld(world_size), twoNodes(), name, GRID_SCALE,
                                                             BUFFER_SCALE,
                                                             murosim::simulation::Simulation::INIT_MODE::MANUAL);
}

inline RobotList robotList(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    return {robot};
}

/// Distance from `position` along `angle` to the wall of the square world.
inline float wallDistance(const Eigen::Vector2f &position, const float angle, const float half = WORLD_HALF_SIZE)
{
    constexpr float PARALLEL = 1e-6f;
    const float cos_angle = std::cos(angle);
    const float sin_angle = std::sin(angle);
    float distance = std::numeric_limits<float>::max();
    if (cos_angle > PARALLEL)
    {
        distance = std::min(distance, (half - position.x()) / cos_angle);
    }
    if (cos_angle < -PARALLEL)
    {
        distance = std::min(distance, (-half - position.x()) / cos_angle);
    }
    if (sin_angle > PARALLEL)
    {
        distance = std::min(distance, (half - position.y()) / sin_angle);
    }
    if (sin_angle < -PARALLEL)
    {
        distance = std::min(distance, (-half - position.y()) / sin_angle);
    }
    return distance;
}

/// Angle of ray `i` of the fixture lidar (full circle: LIDAR_RAYS rays spread
/// evenly, counter-clockwise from -pi).
inline float rayAngle(const size_t i)
{
    return -PI + static_cast<float>(i) * LIDAR_FOV / static_cast<float>(LIDAR_RAYS);
}

/// Base fixture: one holonomic robot in the square world, simulation reset
/// so the robot has its first recorded state. Tests place the robot, set
/// goal or command, and call step() to record the state they observe.
/// reset() clears actuator commands, so commands are set after it.
class SingleRobotTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        sim->addRobot(robot);
        sim->reset();
    }

    /// Advances the simulation by one TIME_STEP and records a state.
    void step(const int count = 1)
    {
        for (int i = 0; i < count; i++)
        {
            sim->simulate();
        }
    }

    RobotList robots() const
    {
        return {robot};
    }

    SimulationList sims() const
    {
        return {sim};
    }

    std::shared_ptr<murosim::simulation::Simulation> sim = makeSimulation("single_robot_test");
    std::shared_ptr<murosim::robot::Robot> robot = holonomicRobot("robot", true);
};
} // namespace fixtures

#endif // MUROSIMRL_TESTS_FIXTURES_HPP_
