// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/observation_simulation.hpp"

namespace murosimrl::observation
{

ObservationSimulation::ObservationSimulation(const std::string &name, const size_t stack_size)
    : ObservationOf<murosim::simulation::Simulation>(name, stack_size)
{
}

} // namespace murosimrl::observation
