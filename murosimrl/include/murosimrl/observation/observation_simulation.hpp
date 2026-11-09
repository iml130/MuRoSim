// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_SIMULATION_HPP_
#define MUROSIMRL_OBSERVATION_SIMULATION_HPP_

#include "murosimrl/observation/observation_of.hpp"
#include "murosim/simulation/simulation.hpp"

namespace murosimrl::observation
{

/// Base class for observations computed over whole simulations, independent
/// of any particular robot (e.g. a global occupancy or velocity map).
/// Subclasses implement compute()/shape() over a list of simulations, as
/// required by ObservationOf; this class adds no extra readiness checks
/// beyond the base's default (accept any non-empty, non-null list).
class ObservationSimulation : public ObservationOf<murosim::simulation::Simulation>
{
protected:
    /// @param name Identifies the observation.
    /// @param stack_size Number of stacked past states per simulation; 0 and 1 are equivalent (no stacking).
    ObservationSimulation(const std::string &name, const size_t stack_size);

public:
    virtual ~ObservationSimulation() = default;
};

} // namespace murosimrl::observation

#endif // MUROSIMRL_OBSERVATION_SIMULATION_HPP_
