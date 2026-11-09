// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosimrl/action.hpp"
#include "murosim/robot/robot.hpp"

namespace murosimrlpy {

namespace {

py::array_t<float> actionGetNeutralPoint(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    std::vector<size_t> shape = murosimrl::Action::shape(robot);
    py::array_t<float> array(shape);
    murosimrl::Action::getNeutralPoint(robot, array.mutable_data());
    return array;
}

// Action is the only murosimrl class reading a caller-provided buffer, so the
// element count must be checked here before the C++ side dereferences it.
void requireDataSize(const py::array_t<float> &data, const size_t expected)
{
    if (static_cast<size_t>(data.size()) != expected)
    {
        throw py::value_error("Action data has " + std::to_string(data.size()) +
            " elements, expected " + std::to_string(expected));
    }
}

} // anonymous namespace

void initCoreAction(py::module_ &m)
{
    auto pyClass = py::class_<murosimrl::Action>(m, "Action", R"doc(Applies RL actions (one float per actuated axis) to robots.

The command written follows the robot's actuator control mode: a target
velocity in VELOCITY mode, an acceleration in ACCELERATION mode. With
normalize, the [-1, 1] action maps onto the matching range (the velocity
limits, or the symmetric [-max, max] acceleration limits). The actuator's
mode is the single source of truth - there is deliberately no mode flag
here that could disagree with it.)doc");
    pyClass.def(py::init<bool>(),
        "Args:\n"
        "    normalize: Interpret action values as [-1, 1] and map them "
        "onto the actuator's command range.",
        py::arg("normalize") = true);
    pyClass.def("normalize", &murosimrl::Action::normalize,
        "Whether actions are interpreted as normalized [-1, 1] values.");

    // Apply action to robots
    pyClass.def("__call__", [](const murosimrl::Action &self,
                               std::shared_ptr<murosim::robot::Robot> &robot,
                               py::array_t<float> data) {
        requireDataSize(data, murosimrl::Action::size(robot));

        py::gil_scoped_release gil;
        self(robot, data.data());
    }, "Apply an action to a single robot.\n\n"
       "Args:\n"
       "    robot: Robot to command.\n"
       "    data: One float per actuated axis (see size()/shape()).",
       py::arg("robot"), py::arg("data"));

    pyClass.def("__call__", [](const murosimrl::Action &self,
                               const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots,
                               py::array_t<float> data) {
        // Heterogeneous actuator types throw inside operator() before any
        // read, so sizing the check off robots[0] is safe.
        requireDataSize(data, robots.empty() ? 0 : robots.size() * murosimrl::Action::size(robots[0]));

        py::gil_scoped_release gil;
        self(robots, data.data());
    }, "Apply one action per robot, packed contiguously. All robots must "
       "share one actuator type.\n\n"
       "Args:\n"
       "    robots: Robots to command, in the same order as `data`.\n"
       "    data: Concatenated per-robot actions, size(robots[0]) floats each.",
       py::arg("robots"), py::arg("data"));

    // Shape methods
    pyClass.def_static("shape", py::overload_cast<const std::shared_ptr<murosim::robot::Robot>&>(
        &murosimrl::Action::shape),
        "Action shape for a single robot: (size(robot),).",
        py::arg("robot"));
    pyClass.def_static("shape", py::overload_cast<const std::vector<std::shared_ptr<murosim::robot::Robot>>&>(
        &murosimrl::Action::shape),
        "Action shape for a robot list: (robots.size(), size(robots[0])), "
        "or empty if `robots` is empty.",
        py::arg("robots"));
    pyClass.def_static("size", &murosimrl::Action::size,
        "Number of actuated axes (action floats) for a robot's actuator type.",
        py::arg("robot"));

    // Neutral point
    pyClass.def_static("get_neutral_point", &actionGetNeutralPoint,
        "Return the action that commands zero (velocity or acceleration, "
        "per the robot's control mode), as a numpy array of size(robot) "
        "floats.",
        py::arg("robot"));

    addCopySupport(pyClass);
}

} // namespace murosimrlpy
