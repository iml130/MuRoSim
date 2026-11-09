// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_UTIL_CONVERSION_HPP_
#define MUROSIMRL_UTIL_CONVERSION_HPP_

#include "murosim/robot/robot.hpp"
#include "murosim/simulation/simulation.hpp"

namespace murosimrl::util::conversion
{

/// Flatten several simulations into one combined list of their robots, preserving simulation then robot order.
/// @param simulations Simulations to read robots from.
/// @return All robots of all simulations, concatenated.
std::vector<std::shared_ptr<murosim::robot::Robot>> fromVectorSimulationToVectorRobot(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations);

} // namespace murosimrl::util::conversion

#endif // MUROSIMRL_UTIL_CONVERSION_HPP_
