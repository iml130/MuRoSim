// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "common.hpp"
#include "version.hpp"

namespace murosimrlpy {

// Forward declarations of all init functions
// Core module
void initCoreAction(py::module_ &m);
void initCoreDone(py::module_ &m);

// Observation module
void initObservationRobot(py::module_ &m);
void initObservationRobotSimulation(py::module_ &m);
void initObservationSimulation(py::module_ &m);
void initObservationSet(py::module_ &m);

// Reward module
void initRewardRobot(py::module_ &m);
void initRewardSimulation(py::module_ &m);
void initRewardSet(py::module_ &m);

PYBIND11_MODULE(murosimrlpy, m)
{
    m.doc() = "MuRoSim Reinforcement Learning Python Bindings";

    // Register the murosimpy types (Simulation, Robot, containers, ...) before
    // binding anything that uses them, so the module works when imported on
    // its own and the signatures name the Python types instead of C++ ones.
    pybind11::module_::import("murosimpy");

    m.attr("__version__") = MUROSIMRLPY_VERSION;

    // Create submodules
    auto pyModuleCore = m.def_submodule("core",
        "Core RL components: Action and Done.");

    auto pyModuleObservation = m.def_submodule("observation",
        "Observation functions for extracting state information.");
    auto pyModuleObservationRobot = pyModuleObservation.def_submodule("robot",
        "Robot-level observations.");
    auto pyModuleObservationRobotSimulation = pyModuleObservation.def_submodule("robot_simulation",
        "Robot simulation-level observations.");
    auto pyModuleObservationSimulation = pyModuleObservation.def_submodule("simulation",
        "Simulation-level observations.");

    auto pyModuleReward = m.def_submodule("reward",
        "Reward functions for reinforcement learning.");
    auto pyModuleRewardRobot = pyModuleReward.def_submodule("robot",
        "Robot-level rewards.");
    auto pyModuleRewardSimulation = pyModuleReward.def_submodule("simulation",
        "Simulation-level rewards.");

    // Initialize modules
    // 1. Core module
    initCoreAction(pyModuleCore);
    initCoreDone(pyModuleCore);

    // 2. Observation module
    initObservationRobot(pyModuleObservationRobot);
    initObservationRobotSimulation(pyModuleObservationRobotSimulation);
    initObservationSimulation(pyModuleObservationSimulation);
    initObservationSet(pyModuleObservation); // after the robot classes: base must be registered first

    // 3. Reward module
    initRewardRobot(pyModuleRewardRobot);
    initRewardSimulation(pyModuleRewardSimulation);
    initRewardSet(pyModuleReward);
}

} // namespace murosimrlpy
