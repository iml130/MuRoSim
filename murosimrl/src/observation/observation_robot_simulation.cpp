// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/observation_robot_simulation.hpp"
#include "murosimrl/util/simulation.hpp"

#include <stdexcept>

namespace murosimrl::observation
{

ObservationRobotSimulation::ObservationRobotSimulation(const std::string &name, const size_t stack_size)
    : ObservationOf<murosim::simulation::Simulation>(name, stack_size)
{
}

void ObservationRobotSimulation::validate(const EntityList &simulations) const
{
    util::simulation::requireSimulationsHaveRobots(simulations);

    if (!util::simulation::checkRobotHasStates(simulations))
    {
        throw std::runtime_error("Robots need at least one state to create observation!");
    }
}

} // namespace murosimrl::observation
