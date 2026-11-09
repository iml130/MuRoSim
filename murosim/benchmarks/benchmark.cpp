// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// End-to-end benchmark for the simulation step loop.
//
// Usage: murosim_benchmark [robots] [steps]
//
// Runs the same scenario once with SYNCHRONIZED and once with CONTINUOUS
// sensor updates and reports the wall-clock time per simulated step.

#include "murosim/simulation/simulation.hpp"
#include "murosim/actuator/differential_simple.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{

constexpr size_t NUM_LIDAR_RAYS = 360;
constexpr float LIDAR_FREQUENCY = 10.0f;
constexpr float WORLD_HALF_SIZE = 20.0f;

std::shared_ptr<murosim::robot::Robot> createRobot(const std::string &name)
{
    const std::unique_ptr<murosim::actuator::Actuator> actuator =
        std::make_unique<murosim::actuator::DifferentialSimple>(-1.0f, 1.0f, 1.0f);

    const murosim::geometry::Shape shape_robot = murosim::geometry::primitives::square(0.3f);
    const murosim::geometry::Shape shape_lidar = murosim::geometry::primitives::square(0.3f);

    const std::vector<murosim::sensor::Lidar> lidars = {
        murosim::sensor::Lidar(Eigen::Affine2f::Identity(), murosim::math::TWO_PI,
                               NUM_LIDAR_RAYS, 0.05f, 20.0f, LIDAR_FREQUENCY)
    };

    return std::make_shared<murosim::robot::Robot>(name, actuator, shape_robot, shape_lidar, lidars);
}

murosim::geometry::World createWorld()
{
    murosim::geometry::Shape shape;
    shape.addLine(-WORLD_HALF_SIZE, -WORLD_HALF_SIZE,  WORLD_HALF_SIZE, -WORLD_HALF_SIZE);
    shape.addLine( WORLD_HALF_SIZE, -WORLD_HALF_SIZE,  WORLD_HALF_SIZE,  WORLD_HALF_SIZE);
    shape.addLine( WORLD_HALF_SIZE,  WORLD_HALF_SIZE, -WORLD_HALF_SIZE,  WORLD_HALF_SIZE);
    shape.addLine(-WORLD_HALF_SIZE,  WORLD_HALF_SIZE, -WORLD_HALF_SIZE, -WORLD_HALF_SIZE);
    shape.addCircle(0.0f, 0.0f, 1.0f);
    shape.addCircle(10.0f, 10.0f, 1.5f);
    shape.addCircle(-10.0f, 10.0f, 1.5f);
    shape.addCircle(10.0f, -10.0f, 1.5f);
    shape.addCircle(-10.0f, -10.0f, 1.5f);
    return murosim::geometry::World(shape);
}

void addRobots(murosim::simulation::Simulation &sim, const size_t num_robots)
{
    const size_t per_row = static_cast<size_t>(std::ceil(std::sqrt(static_cast<double>(num_robots))));
    const float spacing = 1.5f;
    const float offset = -0.5f * spacing * static_cast<float>(per_row - 1);

    for (size_t i = 0; i < num_robots; i++)
    {
        const float x = offset + spacing * static_cast<float>(i % per_row) + 2.0f;
        const float y = offset + spacing * static_cast<float>(i / per_row) + 2.0f;

        std::shared_ptr<murosim::robot::Robot> robot = createRobot("robot_" + std::to_string(i));
        robot->setPose(x, y, 0.0f);
        robot->setGoal(-x, -y, 0.5f);
        robot->setActuatorCommandVelocity(0.2f, 0.0f, 0.3f);

        sim.addRobot(robot);
    }
}

double benchmark(const size_t num_robots, const size_t num_steps, const bool continuous)
{
    murosim::simulation::Simulation sim(createWorld(), murosim::navigation::Container(), "benchmark",
                                        0.1f, 0.2f, murosim::simulation::Simulation::INIT_MODE::MANUAL,
                                        murosim::planning::Planner::TYPE::NONE, 42);

    murosim::simulation::SimulationConfig config = sim.getConfig();
    config.setSensorUpdateMode(continuous
        ? murosim::simulation::SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS
        : murosim::simulation::SimulationConfig::SENSOR_UPDATE_MODE::SYNCHRONIZED);
    sim.setConfig(config);

    addRobots(sim, num_robots);

    // warm-up (also performs the initial reset step)
    for (size_t i = 0; i < 5; i++)
    {
        sim.simulate();
    }

    const auto begin = std::chrono::steady_clock::now();

    for (size_t i = 0; i < num_steps; i++)
    {
        sim.simulate();
    }

    const auto end = std::chrono::steady_clock::now();

    return std::chrono::duration<double>(end - begin).count();
}

} // namespace

int main(int argc, char** argv)
{
    const size_t num_robots = (argc > 1) ? std::strtoul(argv[1], nullptr, 10) : 50;
    const size_t num_steps = (argc > 2) ? std::strtoul(argv[2], nullptr, 10) : 200;

    std::cout << "murosim benchmark: " << num_robots << " robots, " << num_steps
              << " steps, " << NUM_LIDAR_RAYS << " lidar rays\n";

    for (const bool continuous : {false, true})
    {
        const double seconds = benchmark(num_robots, num_steps, continuous);
        const double ms_per_step = seconds * 1000.0 / static_cast<double>(num_steps);

        std::cout << (continuous ? "CONTINUOUS  " : "SYNCHRONIZED")
                  << ": " << seconds << " s total, "
                  << ms_per_step << " ms/step, "
                  << (1000.0 / ms_per_step) << " steps/s\n";
    }

    return 0;
}
