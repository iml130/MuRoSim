// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_UTIL_SIMULATION_HPP_
#define MUROSIMRL_UTIL_SIMULATION_HPP_

#include "murosim/simulation/simulation.hpp"

namespace murosimrl::util::simulation
{

/// Throws std::runtime_error naming `observation` if any simulation's grid map
/// exceeds map_size. Single owner of the fit contract shared by the map-shaped
/// observations (occupancy/path/velocity), which embed the grid map centered in
/// a map_size x map_size output buffer.
/// @param simulations Simulations whose grid maps are checked.
/// @param map_size Side length, in cells, of the square output buffer the maps must fit into.
/// @param observation Name of the calling observation, used in the error message.
void requireGridMapFitSize(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, const size_t map_size, const std::string &observation);

/// Throws std::invalid_argument if any simulation has no robots. Single owner of
/// the contract shared by the robot- and robot-simulation-scope observations.
void requireSimulationsHaveRobots(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations);

/// Check whether every robot of every simulation has at least one recorded state.
bool checkRobotHasStates(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations);

/// Total number of robots across all simulations.
size_t getNumberOfRobots(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations);

} // namespace murosimrl::util::simulation

#endif // MUROSIMRL_UTIL_SIMULATION_HPP_
