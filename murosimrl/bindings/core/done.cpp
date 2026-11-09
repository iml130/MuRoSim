// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosimrl/done.hpp"
#include "murosim/robot/robot.hpp"
#include "murosim/simulation/simulation.hpp"

namespace murosimrlpy {

namespace {

// Input is a vector of Robot or Simulation shared_ptrs; Done::shape and
// Done::operator() overload on both.
template<typename Input>
py::array_t<bool> callDone(const murosimrl::Done &done, const Input &input)
{
    std::vector<size_t> shape = murosimrl::Done::shape(input);
    py::array_t<bool> array(shape);
    bool* data = array.mutable_data();
    {
        py::gil_scoped_release gil;
        done(input, data);
    }
    return array;
}

} // anonymous namespace

void initCoreDone(py::module_ &m)
{
    auto pyClass = py::class_<murosimrl::Done>(m, "Done", R"doc(Reports each robot's episode-termination flag.

True once a robot has reached its goal or collided (Robot.is_finished()).
One bool per robot, in robot order (or simulation then robot order, for
the simulation overloads); no aggregation is done here (a caller wanting a
per-simulation "all done" needs to combine these itself).)doc");
    pyClass.def(py::init<>(), "Construct a Done check (stateless, no configuration).");

    // Call operators
    pyClass.def("__call__", [](const murosimrl::Done &self,
                               const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) {
        return callDone(self, robots);
    }, "Return each robot's is-finished flag, in robot order, as a boolean array.",
       py::arg("robots"));

    pyClass.def("__call__", [](const murosimrl::Done &self,
                               const std::shared_ptr<murosim::robot::Robot> &robot) {
        return callDone(self, std::vector<std::shared_ptr<murosim::robot::Robot>>{robot});
    }, "Return the robot's is-finished flag as a single-element boolean array.",
       py::arg("robot"));

    pyClass.def("__call__", [](const murosimrl::Done &self,
                               const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        return callDone(self, simulations);
    }, "Return each robot's is-finished flag, in simulation then robot "
       "order, as a boolean array.",
       py::arg("simulations"));

    pyClass.def("__call__", [](const murosimrl::Done &self,
                               const std::shared_ptr<murosim::simulation::Simulation> &simulation) {
        return callDone(self, std::vector<std::shared_ptr<murosim::simulation::Simulation>>{simulation});
    }, "Return each of the simulation's robots' is-finished flags, in "
       "robot order, as a boolean array.",
       py::arg("simulation"));

    // Shape methods
    pyClass.def_static("shape", py::overload_cast<const std::shared_ptr<murosim::robot::Robot>&>(
        &murosimrl::Done::shape),
        "Output shape for a single robot: (1,).",
        py::arg("robot"));
    pyClass.def_static("shape", py::overload_cast<const std::vector<std::shared_ptr<murosim::robot::Robot>>&>(
        &murosimrl::Done::shape),
        "Output shape for a robot list: (robots.size(), 1).",
        py::arg("robots"));
    pyClass.def_static("shape", py::overload_cast<const std::shared_ptr<murosim::simulation::Simulation>&>(
        &murosimrl::Done::shape),
        "Output shape for a single simulation: (simulation's robot count, 1).",
        py::arg("simulation"));
    pyClass.def_static("shape", py::overload_cast<const std::vector<std::shared_ptr<murosim::simulation::Simulation>>&>(
        &murosimrl::Done::shape),
        "Output shape for a simulation list: (total robot count, 1).",
        py::arg("simulations"));

    addCopySupport(pyClass);
}

} // namespace murosimrlpy
