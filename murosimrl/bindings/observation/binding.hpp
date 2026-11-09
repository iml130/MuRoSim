// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRLPY_OBSERVATION_BINDING_HPP_
#define MUROSIMRLPY_OBSERVATION_BINDING_HPP_

#include "../common.hpp"

#include "murosimrl/observation/observation_robot.hpp"

#include "murosim/robot/robot.hpp"
#include "murosim/simulation/simulation.hpp"

#include <memory>
#include <vector>

// Shared binding surface for the observation classes. Every scope exposes the
// same pattern (__call__/shape over one or many entities, name, stack_size,
// copy support); only the accepted input types differ. Note that a derived
// observation's shape(EntityList) override hides the base-class convenience
// overloads, so single entities are wrapped into lists here and the
// simulation adapters of the robot scope are reached through an upcast.
namespace murosimrlpy::observation_binding
{

// One numpy round-trip: allocate from shape(), release the GIL, let
// operator() validate and fill the buffer. An empty shape (empty input)
// yields an empty array.
template<typename ObsType, typename Input>
py::array_t<float> call(const ObsType &obs, const Input &input)
{
    const std::vector<size_t> shape = obs.shape(input);

    if (shape.empty())
    {
        return py::array_t<float>();
    }

    py::array_t<float> array(shape);
    float* data = array.mutable_data();

    {
        py::gil_scoped_release gil;
        obs(input, data);
    }

    return array;
}

template<typename ObsType, typename... Extra>
void bindCommon(py::class_<ObsType, Extra...> &pyClass)
{
    pyClass.def("name", &ObsType::name,
        "The observation's name, as given at construction.");
    pyClass.def("stack_size", &ObsType::stackSize,
        "The configured stack size (may be 0, meaning \"no stacking\").");
}

// __call__/shape over simulations (list and single) plus name/stack_size —
// the surface of the simulation- and robot-simulation-scope observations.
template<typename ObsType, typename... Extra>
void bindSimulationObservation(py::class_<ObsType, Extra...> &pyClass)
{
    using SimulationList = std::vector<std::shared_ptr<murosim::simulation::Simulation>>;

    pyClass.def("__call__", [](const ObsType &self, const SimulationList &simulations) {
        return call(self, simulations);
    }, "Compute the observation for each of `simulations` and return it as "
       "one numpy array (see shape() for its layout).",
       py::arg("simulations"));

    pyClass.def("__call__", [](const ObsType &self, const std::shared_ptr<murosim::simulation::Simulation> &simulation) {
        return call(self, SimulationList{simulation});
    }, "Compute the observation for a single simulation and return it as a numpy array.",
       py::arg("simulation"));

    pyClass.def("shape", [](const ObsType &self, const SimulationList &simulations) {
        return self.shape(simulations);
    }, "Output shape for observing `simulations`.",
       py::arg("simulations"));

    pyClass.def("shape", [](const ObsType &self, const std::shared_ptr<murosim::simulation::Simulation> &simulation) {
        return self.shape(SimulationList{simulation});
    }, "Output shape for observing a single simulation.",
       py::arg("simulation"));

    bindCommon(pyClass);
}

// Robot-scope surface: robot-list/robot overloads plus the simulation
// adapters ObservationRobot already provides (reached via upcast).
template<typename ObsType, typename... Extra>
void bindRobotObservation(py::class_<ObsType, Extra...> &pyClass)
{
    using RobotList = std::vector<std::shared_ptr<murosim::robot::Robot>>;
    using SimulationList = std::vector<std::shared_ptr<murosim::simulation::Simulation>>;

    pyClass.def("__call__", [](const ObsType &self, const RobotList &robots) {
        return call(self, robots);
    }, "Compute the observation for each of `robots` and return it as one "
       "numpy array (see shape() for its layout).",
       py::arg("robots"));

    pyClass.def("__call__", [](const ObsType &self, const std::shared_ptr<murosim::robot::Robot> &robot) {
        return call(self, RobotList{robot});
    }, "Compute the observation for a single robot and return it as a numpy array.",
       py::arg("robot"));

    pyClass.def("__call__", [](const ObsType &self, const SimulationList &simulations) {
        const murosimrl::observation::ObservationRobot &base = self;
        return call(base, simulations);
    }, "Compute the observation for the combined robots of `simulations` "
       "and return it as one numpy array.",
       py::arg("simulations"));

    pyClass.def("__call__", [](const ObsType &self, const std::shared_ptr<murosim::simulation::Simulation> &simulation) {
        const murosimrl::observation::ObservationRobot &base = self;
        return call(base, simulation);
    }, "Compute the observation for the robots of a single simulation and "
       "return it as a numpy array.",
       py::arg("simulation"));

    pyClass.def("shape", [](const ObsType &self, const RobotList &robots) {
        return self.shape(robots);
    }, "Output shape for observing `robots`.",
       py::arg("robots"));

    pyClass.def("shape", [](const ObsType &self, const std::shared_ptr<murosim::robot::Robot> &robot) {
        return self.shape(RobotList{robot});
    }, "Output shape for observing a single robot.",
       py::arg("robot"));

    pyClass.def("shape", [](const ObsType &self, const SimulationList &simulations) {
        const murosimrl::observation::ObservationRobot &base = self;
        return base.shape(simulations);
    }, "Output shape for observing the combined robots of `simulations`.",
       py::arg("simulations"));

    pyClass.def("shape", [](const ObsType &self, const std::shared_ptr<murosim::simulation::Simulation> &simulation) {
        const murosimrl::observation::ObservationRobot &base = self;
        return base.shape(simulation);
    }, "Output shape for observing the robots of a single simulation.",
       py::arg("simulation"));

    bindCommon(pyClass);
}

} // namespace murosimrlpy::observation_binding

#endif // MUROSIMRLPY_OBSERVATION_BINDING_HPP_
