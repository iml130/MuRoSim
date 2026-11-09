// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/simulation/simulation.hpp"

#include <sstream>

namespace murosimpy {

void initContainersSimulationContainer(pybind11::module &m)
{
    // Type alias for readability
    using SimContainer = std::vector<std::shared_ptr<murosim::simulation::Simulation>>;

    auto pySimulationContainer = pybind11::class_<SimContainer>(m, "Simulations",
        "A container for managing multiple Simulation instances, e.g. for use\n"
        "with ThreadPool.execute_simulate() to step them all in parallel.");

    // Constructor
    pySimulationContainer.def(pybind11::init(),
        "Create an empty simulation container.");

    // Add methods
    pySimulationContainer.def("add", [](SimContainer &c, const std::shared_ptr<murosim::simulation::Simulation> &sim) {
        c.push_back(sim);
    }, "Add a single simulation to the container.",
       pybind11::arg("simulation"));
    pySimulationContainer.def("add", [](SimContainer &c, const SimContainer &other) {
        c.insert(c.end(), other.cbegin(), other.cend());
    }, "Add all simulations from another container.",
       pybind11::arg("container"));

    // Utility properties
    pySimulationContainer.def_property_readonly("max_world_size", [](const SimContainer &c) {
        float biggest = 0;
        for (const auto &sim : c) {
            float len = sim->getWorld().getAabbSizeX();
            if (len > biggest) biggest = len;
        }
        return biggest;
    }, "Maximum world size (x-axis) across all simulations.");
    pySimulationContainer.def_property_readonly("robots_per_simulation", [](const SimContainer &c) {
        std::vector<int64_t> counts;
        counts.reserve(c.size());
        for (const auto &sim : c) {
            counts.push_back(static_cast<int64_t>(sim->getRobots().size()));
        }
        return counts;
    }, "Number of robots in each simulation as a list.");
    pySimulationContainer.def_property_readonly("active_robot_counts", [](const SimContainer &c) {
        std::vector<int64_t> counts;
        counts.reserve(c.size());
        for (const auto &sim : c) {
            size_t active = 0;
            for (const auto& robot : sim->getRobots()) {
                if (!robot->isFinished()) active++;
            }
            counts.push_back(static_cast<int64_t>(active));
        }
        return counts;
    }, "Number of active (non-finished) robots in each simulation as a list.");
    pySimulationContainer.def("clear", &SimContainer::clear,
        "Remove all simulations from the container.");
    pySimulationContainer.def("active_simulations", [](const SimContainer &c) {
        SimContainer active;
        std::vector<int64_t> indices;
        for (size_t i = 0; i < c.size(); i++) {
            if (!c[i]->isFinished()) {
                active.push_back(c[i]);
                indices.push_back(static_cast<int64_t>(i));
            }
        }
        return pybind11::make_tuple(active, indices);
    }, "Get active (non-finished) simulations and their indices.\n\n"
       "Returns:\n"
       "    Tuple of (active_simulations, indices)");
    pySimulationContainer.def_property_readonly("all_robots", [](const SimContainer &c) {
        std::vector<std::shared_ptr<murosim::robot::Robot>> robots;
        for (const auto &sim : c) {
            for (const auto& robot : sim->getRobots()) {
                robots.push_back(robot);
            }
        }
        return robots;
    }, "All robots from all simulations as a flat list.");
    pySimulationContainer.def("active_robots", [](const SimContainer &c) {
        std::vector<std::shared_ptr<murosim::robot::Robot>> robots;
        std::vector<int64_t> indices;
        int64_t idx = 0;
        for (const auto &sim : c) {
            for (const auto& robot : sim->getRobots()) {
                if (!robot->isFinished()) {
                    robots.push_back(robot);
                    indices.push_back(idx);
                }
                idx++;
            }
        }
        return pybind11::make_tuple(robots, indices);
    }, "Get active (non-finished) robots and their global indices.\n\n"
       "Returns:\n"
       "    Tuple of (active_robots, indices)");
    pySimulationContainer.def_property_readonly("as_list", [](const SimContainer &c) {
        // The container type is opaque (see common.hpp), so build a real list.
        pybind11::list out;
        for (const auto &sim : c) {
            out.append(sim);
        }
        return out;
    }, "Container as a Python list.");

    // Python special methods
    pySimulationContainer.def("__repr__", [](const SimContainer &c) {
        std::ostringstream oss;
        oss << "<Simulations size=" << c.size() << ">";
        return oss.str();
    }, "Return a short debug string with the number of contained simulations.");
    pySimulationContainer.def("__len__", &SimContainer::size,
        "Return the number of simulations.");
    pySimulationContainer.def("__getitem__", [](const SimContainer &c, size_t index) {
        checkIndex(index, c.size(), "Simulation");
        return c[index];
    }, "Get simulation at the specified index.",
       pybind11::arg("index"));
    pySimulationContainer.def("__iter__", [](const SimContainer &c) {
        return pybind11::make_iterator(c.begin(), c.end());
    }, pybind11::keep_alive<0, 1>(),
       "Iterate over all simulations.\n\n"
       "The container is kept alive for as long as the returned iterator lives.");
    pySimulationContainer.def("__copy__", [](const SimContainer &c) { return SimContainer(c); },
        "Support for copy.copy(): returns a shallow copy sharing the same Simulation instances.");
    pySimulationContainer.def("__deepcopy__", [](const SimContainer &c, pybind11::dict memo) {
        SimContainer copy;
        copy.reserve(c.size());
        for (const auto &sim : c) {
            copy.emplace_back(std::make_shared<murosim::simulation::Simulation>(*sim));
        }
        memo[pybind11::cast(&c)] = copy;
        return copy;
    }, "Support for copy.deepcopy(): returns a new container holding deep copies of every simulation.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
