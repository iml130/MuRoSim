// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/robot/robot.hpp"

#include <sstream>

namespace murosimpy {

void initContainersRobotContainer(pybind11::module &m)
{
    // Type alias for readability
    using RobotContainer = std::vector<std::shared_ptr<murosim::robot::Robot>>;

    auto pyRobotContainer = pybind11::class_<RobotContainer>(m, "Robots",
        "A container for managing multiple Robot instances, e.g. to collect\n"
        "robots gathered from several simulations for batched observation/reward\n"
        "computation.");

    // Constructor
    pyRobotContainer.def(pybind11::init(),
        "Create an empty robot container.");

    // Add methods
    pyRobotContainer.def("add", [](RobotContainer &c, const std::shared_ptr<murosim::robot::Robot> &robot) {
        c.push_back(robot);
    }, "Add a single robot to the container.",
       pybind11::arg("robot"));
    pyRobotContainer.def("add", [](RobotContainer &c, const RobotContainer &other) {
        c.insert(c.end(), other.cbegin(), other.cend());
    }, "Add all robots from another container.",
       pybind11::arg("container"));
    pyRobotContainer.def("clear", &RobotContainer::clear,
        "Remove all robots from the container.");
    pyRobotContainer.def("active_robots", [](const RobotContainer &c) {
        RobotContainer active;
        std::vector<int64_t> indices;
        for (size_t i = 0; i < c.size(); i++) {
            if (!c[i]->isFinished()) {
                active.push_back(c[i]);
                indices.push_back(static_cast<int64_t>(i));
            }
        }
        return pybind11::make_tuple(active, indices);
    }, "Get active (non-finished) robots and their indices.\n\n"
       "Returns:\n"
       "    Tuple of (active_robots, indices)");
    pyRobotContainer.def_property_readonly("as_list", [](const RobotContainer &c) {
        // The container type is opaque (see common.hpp), so build a real list.
        pybind11::list out;
        for (const auto &robot : c) {
            out.append(robot);
        }
        return out;
    }, "Container as a Python list.");

    // Python special methods
    pyRobotContainer.def("__repr__", [](const RobotContainer &c) {
        std::ostringstream oss;
        oss << "<Robots size=" << c.size() << ">";
        return oss.str();
    }, "Return a short debug string with the number of contained robots.");
    pyRobotContainer.def("__len__", &RobotContainer::size,
        "Return the number of robots.");
    pyRobotContainer.def("__getitem__", [](const RobotContainer &c, size_t index) {
        checkIndex(index, c.size(), "Robot");
        return c[index];
    }, "Get robot at the specified index.",
       pybind11::arg("index"));
    pyRobotContainer.def("__iter__", [](const RobotContainer &c) {
        return pybind11::make_iterator(c.begin(), c.end());
    }, pybind11::keep_alive<0, 1>(),
       "Iterate over all robots.\n\n"
       "The container is kept alive for as long as the returned iterator lives.");
    pyRobotContainer.def("__copy__", [](const RobotContainer &c) { return RobotContainer(c); },
        "Support for copy.copy(): returns a shallow copy sharing the same Robot instances.");
    pyRobotContainer.def("__deepcopy__", [](const RobotContainer &c, pybind11::dict memo) {
        RobotContainer copy;
        copy.reserve(c.size());
        for (const auto &robot : c) {
            copy.emplace_back(std::make_shared<murosim::robot::Robot>(*robot));
        }
        memo[pybind11::cast(&c)] = copy;
        return copy;
    }, "Support for copy.deepcopy(): returns a new container holding deep copies of every robot.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
