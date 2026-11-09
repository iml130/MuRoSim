// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Renders every public Renderer::draw() overload in sequence, so each
// primitive type (world lines, goal nodes, robot bodies, lidar rays,
// grid-map quads, trajectories and planner paths) can be confirmed by eye.
//
// Needs a display: this example opens a GLFW window. Each scene plays for a
// few seconds; any key skips to the next one, closing the window quits.
//
//   cmake -S . -B build -DMUROSIM_BUILD_EXAMPLES=ON
//   cmake --build build
//   ./build/examples/murosim_example_visualization

#include "murosim/io/data_container.hpp"
#include "murosim/models/robot.hpp"
#include "murosim/navigation/node.hpp"
#include "murosim/geometry/shape.hpp"
#include "murosim/robot/robot.hpp"
#include "murosim/simulation/simulation.hpp"
#include "murosim/threading/thread_pool.hpp"
#include "murosim/visualization/renderer_glfw.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <utility>
#include <vector>

namespace
{
using murosim::io::DataContainer;
using murosim::simulation::Simulation;
using murosim::visualization::RendererGlfw;

constexpr float WORLD_SCALE = 5.0f;
constexpr float NODE_RADIUS = 0.4f;
constexpr size_t FRAMES_STATIC = 120;
constexpr size_t FRAMES_ANIMATED = 300;
constexpr int FRAME_DELAY_MS = 16; // ~60 FPS
constexpr uint32_t SEED = 42;

/// A square room with one goal node in the middle of each quadrant.
DataContainer buildWorld(const float scale)
{
    const float low = scale * 0.25f;
    const float high = scale * 0.75f;

    std::vector<murosim::geometry::Line> lines;
    lines.emplace_back(0.0f, 0.0f, 0.0f, scale);
    lines.emplace_back(0.0f, scale, scale, scale);
    lines.emplace_back(scale, scale, scale, 0.0f);
    lines.emplace_back(scale, 0.0f, 0.0f, 0.0f);

    std::vector<murosim::navigation::Node> nodes;
    nodes.emplace_back(low, low, NODE_RADIUS);
    nodes.emplace_back(low, high, NODE_RADIUS);
    nodes.emplace_back(high, low, NODE_RADIUS);
    nodes.emplace_back(high, high, NODE_RADIUS);

    return DataContainer(std::move(lines), std::vector<murosim::geometry::Circle>(), std::move(nodes));
}

/// Turns a differential-drive robot towards its goal and drives once it faces it.
void steerTowardsGoal(murosim::robot::Robot &robot)
{
    const Eigen::Affine2f &pose = robot.getPose();
    const murosim::navigation::Node &goal = robot.getGoal();
    const float heading = std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
    float bearing = std::atan2(goal.getY() - pose.translation().y(), goal.getX() - pose.translation().x()) - heading;
    bearing = std::atan2(std::sin(bearing), std::cos(bearing)); // wrap to [-pi, pi]

    const float forward = std::max(0.0f, std::cos(bearing));
    const float angular = std::clamp(2.0f * bearing, -1.0f, 1.0f);
    robot.setActuatorCommandVelocity(forward, 0.0f, angular);
}

/// Steers and steps every simulation once per frame, then draws and presents
/// one frame. A simulation whose robots have all finished starts over with
/// new goals. Returns false once the window has been closed.
bool runScene(RendererGlfw &renderer,
              const char *name,
              const std::function<void()> &draw,
              murosim::threading::ThreadPool &thread_pool,
              const std::vector<std::shared_ptr<Simulation>> &simulations,
              const size_t frames)
{
    std::cout << "[scene] " << name << " (any key: next scene)" << std::endl;
    for (size_t i = 0; i < frames; i++)
    {
        if (!simulations.empty())
        {
            std::list<std::function<void()>> jobs;
            for (const auto &simulation : simulations)
            {
                jobs.push_back([simulation]()
                {
                    if (simulation->isFinished())
                    {
                        simulation->reset();
                    }
                    for (const auto &robot : simulation->getRobots())
                    {
                        steerTowardsGoal(*robot);
                    }
                    simulation->simulate();
                });
            }
            thread_pool.execute(jobs);
        }

        draw();
        renderer.show();

        const int key = renderer.waitKey(FRAME_DELAY_MS);
        if (!renderer.isOpen())
        {
            return false;
        }
        if (key != -1)
        {
            break;
        }
    }
    return true;
}
} // namespace

int main()
{
    murosim::threading::ThreadPool thread_pool(0); // 0: one thread per hardware core

    const DataContainer data_container = buildWorld(WORLD_SCALE);

    // One simulation with two robots. Start and goal are drawn from the nodes,
    // and an A* planner fills in the path the renderer draws for each robot.
    auto simulation = std::make_shared<Simulation>(data_container);
    // mir100(velocity_x_min, velocity_x_max in m/s, velocity_angular_max in rad/s)
    simulation->addRobot(murosim::models::robot::mir100(0.0f, 1.0f, 1.0f));
    simulation->addRobot(murosim::models::robot::mir100(0.0f, 1.0f, 1.0f));

    auto config = simulation->getConfig();
    config.setInitMode(Simulation::INIT_MODE::NODE_ALL_RANDOM);
    config.setPlannerType(murosim::planning::Planner::TYPE::ASTAR8);
    config.setSeed(SEED);
    simulation->setConfig(config); // also triggers reset()

    // Four copies for the draw(vector<Simulation>) overload. A copy also
    // copies the random state, so each gets its own seed to differ.
    std::vector<std::shared_ptr<Simulation>> simulations;
    for (uint32_t i = 0; i < 4; i++)
    {
        auto copy = std::make_shared<Simulation>(*simulation);
        config.setSeed(SEED + 1 + i);
        copy->setConfig(config);
        simulations.push_back(copy);
    }

    const std::shared_ptr<murosim::robot::Robot> &robot = simulation->getRobots().front();
    const std::vector<DataContainer> data_containers{data_container, buildWorld(WORLD_SCALE * 2.0f)};

    RendererGlfw renderer(1000);
    renderer.setLaserSkipSteps(10); // draw every 10th lidar ray
    renderer.setDrawGridMap(true);
    renderer.setDrawRobotPath(true);
    renderer.setRobotTrajectoryLength(60);

    const std::vector<std::shared_ptr<Simulation>> no_step;
    const std::vector<std::shared_ptr<Simulation>> single{simulation};

    const bool completed =
        runScene(renderer, "DataContainer: world lines + goal nodes",
                 [&]() { renderer.draw(data_container); }, thread_pool, no_step, FRAMES_STATIC) &&
        runScene(renderer, "DataContainers: one world per grid cell",
                 [&]() { renderer.draw(data_containers); }, thread_pool, no_step, FRAMES_STATIC) &&
        runScene(renderer, "Shape: robot body",
                 [&]() { renderer.draw(robot->getShapeRobot()); }, thread_pool, no_step, FRAMES_STATIC) &&
        runScene(renderer, "Robot: body + lidar shape + lasers",
                 [&]() { renderer.draw(robot); }, thread_pool, no_step, FRAMES_STATIC) &&
        runScene(renderer, "World: world geometry only",
                 [&]() { renderer.draw(simulation->getWorld()); }, thread_pool, no_step, FRAMES_STATIC) &&
        // Animated: robots drive to their goals, so trajectories, planner
        // paths and lasers update live.
        runScene(renderer, "Simulation: robots, goals, lasers, grid, trajectories, paths",
                 [&]() { renderer.draw(simulation); }, thread_pool, single, FRAMES_ANIMATED) &&
        runScene(renderer, "Simulations: 2x2 grid of independent worlds",
                 [&]() { renderer.draw(simulations); }, thread_pool, simulations, FRAMES_ANIMATED);

    std::cout << (completed ? "[done] all draw() overloads shown" : "[done] window closed") << std::endl;
    return 0;
}
