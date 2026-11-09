// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/simulation/simulation.hpp"
#include "murosim/simulation/simulation_config.hpp"
#include "murosim/models/world.hpp"
#include "murosim/io/world.hpp"
#include "murosim/io/util.hpp"
#include "murosim/planning/factory.hpp"

#include <sstream>
#include <sys/types.h>

namespace murosimpy {
namespace {

std::shared_ptr<murosim::simulation::Simulation> createSimulationEmptyWorld(
    const float scale, const float grid_scale, const float buffer_scale)
{
    const murosim::io::DataContainer data = murosim::models::world::empty(scale * 10.0f, scale * 1.0f);
    return std::make_shared<murosim::simulation::Simulation>(data, grid_scale, buffer_scale);
}

std::shared_ptr<murosim::simulation::Simulation> createSimulationFromWorldAndNodes(
    const murosim::geometry::World &world, const murosim::navigation::Container &nodes,
    const std::string &name, const float grid_scale, const float buffer_scale)
{
    return std::make_shared<murosim::simulation::Simulation>(
        world, nodes, (name.empty()) ? "UNSET" : name, grid_scale, buffer_scale);
}

std::shared_ptr<murosim::simulation::Simulation> loadSimulationFromWorldFile(
    const std::string &path, const float scale, const float grid_scale,
    const float buffer_scale, const murosim::planning::Planner::TYPE planner_type)
{
    const std::filesystem::directory_entry filename = std::filesystem::directory_entry(path);
    const murosim::io::DataContainer data = murosim::io::world::loadWorldFile(filename);
    return std::make_shared<murosim::simulation::Simulation>(
        (scale == 1.0f) ? data : data.scale(scale), grid_scale, buffer_scale,
        murosim::simulation::Simulation::INIT_MODE::MANUAL,
        planner_type);
}

std::shared_ptr<murosim::simulation::Simulation> createSimulationFromDataContainer(
    const murosim::io::DataContainer &data, const float scale, const float grid_scale,
    const float buffer_scale, const murosim::planning::Planner::TYPE planner_type)
{
    return std::make_shared<murosim::simulation::Simulation>(
        data.scale(scale), grid_scale, buffer_scale,
        murosim::simulation::Simulation::INIT_MODE::MANUAL,
        planner_type);
}

std::shared_ptr<murosim::simulation::Simulation> createSimulationFromDataContainerWithConfig(
    const murosim::io::DataContainer &data, const murosim::simulation::SimulationConfig &config)
{
    return std::make_shared<murosim::simulation::Simulation>(data, config);
}

std::shared_ptr<murosim::simulation::Simulation> createSimulationFromWorldAndNodesWithConfig(
    const murosim::geometry::World &world, const murosim::navigation::Container &nodes,
    const std::string &name, const murosim::simulation::SimulationConfig &config)
{
    return std::make_shared<murosim::simulation::Simulation>(
        world, nodes, (name.empty()) ? "UNSET" : name, config);
}

pybind11::array_t<uint8_t> createGridMap(const murosim::grid::Map &map, const bool with_buffer)
{
    const size_t size_x = map.sizeX();
    const size_t size_y = map.sizeY();
    pybind11::array_t<uint8_t> array = pybind11::array_t<uint8_t>({size_x, size_y}, {});
    uint8_t *data = array.mutable_data();
    const std::array<uint8_t, 256>& lut = (with_buffer)
        ? murosim::grid::Map::LUT_WITH_BUFFER_UINT8_T
        : murosim::grid::Map::LUT_NO_BUFFER_UINT8_T;

    // Map storage is y-major (index = x + y * size_x) while the numpy array
    // is C-contiguous with shape {size_x, size_y}, i.e. array[x][y] is
    // data[x * size_y + y] — a plain linear copy would shear the grid.
    for (size_t y = 0; y < size_y; y++) {
        for (size_t x = 0; x < size_x; x++) {
            data[x * size_y + y] = lut[map[x + y * size_x]];
        }
    }
    return array;
}

// Helper to collect robots from a simulation. The vector type is opaque
// (see common.hpp), so this surfaces as a "Robots" container in Python.
std::vector<std::shared_ptr<murosim::robot::Robot>> collectRobots(murosim::simulation::Simulation &sim) {
    return sim.getRobots();
}

} // anonymous namespace

void initCoreSimulation(pybind11::module &m)
{
    auto pySimulation = pybind11::class_<murosim::simulation::Simulation, std::shared_ptr<murosim::simulation::Simulation>>(m, "Simulation",
        R"doc(Owns and advances one multi-robot scenario.

Holds the static world, the navigation graph, the derived occupancy grid,
the set of robots, and the per-simulation random generator and planner.
step() advances the whole scenario by one SimulationConfig.time_step,
handling actuation, collision detection, lidar updates, goal bookkeeping
and (re-)initialisation.

Not thread-safe: concurrent simulations must each own their own instance
(see ThreadPool for stepping many simulations in parallel).)doc");

    // Constructors
    pySimulation.def(pybind11::init(&createSimulationEmptyWorld),
        "Create a simulation with an empty world.\n\n"
        "Args:\n"
        "    scale: World scale factor (default: 1.0)\n"
        "    grid_scale: Grid resolution in meters per cell (default: 0.1)\n"
        "    buffer_scale: Safety buffer around obstacles in meters (default: 0.2)",
        pybind11::arg("scale") = 1.0f,
        pybind11::arg("grid_scale") = 0.1f,
        pybind11::arg("buffer_scale") = 0.2f);
    pySimulation.def(pybind11::init(&loadSimulationFromWorldFile),
        "Load a simulation from a world file.\n\n"
        "Args:\n"
        "    worldfile: Path to the world file\n"
        "    scale: World scale factor (default: 1.0)\n"
        "    grid_scale: Grid resolution in meters per cell (default: 0.1)\n"
        "    buffer_scale: Safety buffer around obstacles (default: 0.2)\n"
        "    planner_type: Path planner type (default: PlannerType.NONE)",
        pybind11::arg("worldfile"),
        pybind11::arg("scale") = 1.0f,
        pybind11::arg("grid_scale") = 0.1f,
        pybind11::arg("buffer_scale") = 0.2f,
        pybind11::arg("planner_type") = murosim::planning::Planner::TYPE::NONE);
    pySimulation.def(pybind11::init(&createSimulationFromDataContainer),
        "Create a simulation from a DataContainer.\n\n"
        "Args:\n"
        "    data: DataContainer with world geometry and nodes\n"
        "    scale: World scale factor (default: 1.0)\n"
        "    grid_scale: Grid resolution in meters per cell (default: 0.1)\n"
        "    buffer_scale: Safety buffer around obstacles (default: 0.2)\n"
        "    planner_type: Path planner type (default: PlannerType.NONE)",
        pybind11::arg("data"),
        pybind11::arg("scale") = 1.0f,
        pybind11::arg("grid_scale") = 0.1f,
        pybind11::arg("buffer_scale") = 0.2f,
        pybind11::arg("planner_type") = murosim::planning::Planner::TYPE::NONE);
    pySimulation.def(pybind11::init(&createSimulationFromWorldAndNodes),
        "Create a simulation from World and NodeContainer objects.\n\n"
        "Args:\n"
        "    world: World geometry\n"
        "    nodes: Navigation nodes\n"
        "    name: Simulation name (default: empty)\n"
        "    grid_scale: Grid resolution in meters per cell (default: 0.1)\n"
        "    buffer_scale: Safety buffer around obstacles (default: 0.2)",
        pybind11::arg("world"),
        pybind11::arg("nodes"),
        pybind11::arg("name") = std::string(),
        pybind11::arg("grid_scale") = 0.1f,
        pybind11::arg("buffer_scale") = 0.2f);
    pySimulation.def(pybind11::init(&createSimulationFromDataContainerWithConfig),
        "Create a simulation from a DataContainer with SimulationConfig.\n\n"
        "Args:\n"
        "    data: DataContainer with world geometry and nodes\n"
        "    config: SimulationConfig with all simulation parameters",
        pybind11::arg("data"),
        pybind11::arg("config"));
    pySimulation.def(pybind11::init(&createSimulationFromWorldAndNodesWithConfig),
        "Create a simulation from World and NodeContainer with SimulationConfig.\n\n"
        "Args:\n"
        "    world: World geometry\n"
        "    nodes: Navigation nodes\n"
        "    name: Simulation name\n"
        "    config: SimulationConfig with all simulation parameters",
        pybind11::arg("world"),
        pybind11::arg("nodes"),
        pybind11::arg("name"),
        pybind11::arg("config"));

    // Core methods (gil_scoped_release: pure C++ work with no Python
    // callbacks, matching ThreadPool.execute_* and the murosimrl bindings so
    // multi-threaded Python does not serialize on the GIL)
    pySimulation.def("reset", &murosim::simulation::Simulation::reset,
        pybind11::call_guard<pybind11::gil_scoped_release>(),
        "Reset every robot (including its lidars), re-initialise robot positions\n"
        "per the configured InitMode and run one step() to populate the initial\n"
        "lidar/state, then rewind the step counter to 0. In CONTINUOUS sensor\n"
        "mode every lidar first takes a complete scan of the new pose, so no\n"
        "reading from before the reset survives into the episode.\n\n"
        "Raises:\n"
        "    RuntimeError: If position initialisation fails.");
    pySimulation.def("step", &murosim::simulation::Simulation::simulate,
        pybind11::call_guard<pybind11::gil_scoped_release>(),
        "Advance the simulation by one time step (SimulationConfig.time_step\n"
        "seconds), sub-stepping actuators/collision/sensors at their configured\n"
        "intervals. If the simulation is not ready (e.g. just constructed or\n"
        "after set_config()), calls reset() instead, which itself performs one step.");
    pySimulation.def("planning", [](murosim::simulation::Simulation &sim) { sim.plan(); },
        pybind11::call_guard<pybind11::gil_scoped_release>(),
        "Execute path planning for all robots.");

    // Read-only properties
    pySimulation.def_property_readonly("name", &murosim::simulation::Simulation::getName,
        "Simulation name identifier.");
    pySimulation.def_property_readonly("world", &murosim::simulation::Simulation::getWorld,
        "World geometry containing lines and circles.");
    pySimulation.def_property_readonly("nodes", &murosim::simulation::Simulation::getNodes,
        "Navigation nodes for goal assignment.");
    pySimulation.def_property_readonly("finished", &murosim::simulation::Simulation::isFinished,
        "Whether all robots have finished (reached goals or failed).");
    pySimulation.def_property_readonly("steps", &murosim::simulation::Simulation::getSimulatedStepCount,
        "Number of simulation steps executed.");
    pySimulation.def_property_readonly("robot_count", [](const murosim::simulation::Simulation &sim) {
        return sim.getRobots().size();
    }, "Number of robots currently in the simulation.");
    pySimulation.def_property_readonly("max_robots", &murosim::simulation::Simulation::getPossibleRobotSize,
        "Maximum number of robots that can be added.");
    pySimulation.def_property_readonly("robots", &collectRobots,
        "List of all robots in the simulation.");

    // Config access (returns a copy: the property default reference_internal
    // would alias the live config_ and let its setters bypass setConfig()'s
    // rebuild of grid map, seed, and planner)
    pySimulation.def_property_readonly("config", &murosim::simulation::Simulation::getConfig,
        pybind11::return_value_policy::copy,
        "Copy of the SimulationConfig. Use set_config() to modify the simulation.");
    pySimulation.def("set_config", &murosim::simulation::Simulation::setConfig,
        "Set the simulation config and rebuild the state derived from it\n"
        "(grid map, random generator seed, planner). Robot positions are\n"
        "re-initialised by a reset on the next simulate() call.");

    // Robot management
    pySimulation.def("add_robot", [](murosim::simulation::Simulation &sim, const std::shared_ptr<murosim::robot::Robot> &robot) {
        if (!sim.addRobot(robot)) {
            throw std::runtime_error("Robot is already in the simulation.");
        }
    }, "Add a robot to the simulation.\n\n"
       "Raises:\n"
       "    RuntimeError: If the robot is already in the simulation.",
       pybind11::arg("robot"));
    pySimulation.def("get_robot", [](murosim::simulation::Simulation &sim, size_t index) {
        const auto& robots = sim.getRobots();
        checkIndex(index, robots.size(), "Robot");
        return robots[index];
    }, "Get a robot by index.\n\n"
       "Raises:\n"
       "    IndexError: If index is out of range.",
       pybind11::arg("index"));
    pySimulation.def("remove_all_robots", &murosim::simulation::Simulation::removeAllRobots,
        "Remove all robots from the simulation.");

    // Map access
    pySimulation.def("get_map", [](const murosim::simulation::Simulation &sim, const bool with_buffer) {
        return createGridMap(sim.getGridMap(), with_buffer);
    }, "Get the occupancy grid map as a numpy array.\n\n"
       "Args:\n"
       "    with_buffer: Include safety buffer around obstacles (default: True)\n\n"
       "Returns:\n"
       "    2D numpy array of uint8 occupancy values.",
       pybind11::arg("with_buffer") = true);

    // Python special methods
    pySimulation.def("__repr__", [](const murosim::simulation::Simulation &sim) {
        std::ostringstream oss;
        oss << "<Simulation '" << sim.getName() << "' robots=" << sim.getRobots().size()
            << " steps=" << sim.getSimulatedStepCount() << ">";
        return oss.str();
    }, "Return a short debug string with the simulation's name, robot count and step count.");
    pySimulation.def("__len__", [](const murosim::simulation::Simulation &sim) {
        return sim.getRobots().size();
    }, "Return the number of robots.");
    pySimulation.def("__getitem__", [](murosim::simulation::Simulation &sim, size_t index) {
        const auto& robots = sim.getRobots();
        checkIndex(index, robots.size(), "Robot");
        return robots[index];
    }, "Get a robot by index.\n\n"
       "Raises:\n"
       "    IndexError: If index is out of range.",
       pybind11::arg("index"));
    pySimulation.def("clone", [](const murosim::simulation::Simulation &sim) {
        return std::make_shared<murosim::simulation::Simulation>(sim);
    }, "Create a deep copy of this simulation.");
    pySimulation.def("__copy__", [](const murosim::simulation::Simulation &self) {
        return std::make_shared<murosim::simulation::Simulation>(self);
    }, "Support for copy.copy(): returns a deep copy, same as clone().");
    pySimulation.def("__deepcopy__", [](const murosim::simulation::Simulation &self, pybind11::dict /*memo*/) {
        return std::make_shared<murosim::simulation::Simulation>(self);
    }, "Support for copy.deepcopy(): returns a deep copy, same as clone().",
       pybind11::arg("memo"));
}

} // namespace murosimpy
