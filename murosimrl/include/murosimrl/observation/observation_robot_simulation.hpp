// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_HPP_

#include "murosimrl/observation/observation_of.hpp"
#include "murosim/simulation/simulation.hpp"

namespace murosimrl::observation
{

/// Base class for observations that are computed per robot but need the
/// surrounding simulation as context (e.g. a per-robot map centred on the
/// robot's pose, or a goal encoded relative to it). Subclasses implement
/// compute()/shape() over a list of simulations, as required by ObservationOf;
/// validate() additionally requires every simulation to have at least one
/// robot and every robot to have at least one recorded state.
class ObservationRobotSimulation : public ObservationOf<murosim::simulation::Simulation>
{
protected:
    /// @param name Identifies the observation.
    /// @param stack_size Number of stacked past states per robot; 0 and 1 are equivalent (no stacking).
    ObservationRobotSimulation(const std::string &name, const size_t stack_size);

    /// Require every simulation to have at least one robot with at least one recorded state.
    void validate(const EntityList &simulations) const override;

public:
    virtual ~ObservationRobotSimulation() = default;
};

} // namespace murosimrl::observation

#endif // MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_HPP_
