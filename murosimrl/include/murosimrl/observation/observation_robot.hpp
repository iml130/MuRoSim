// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_HPP_

#include "murosimrl/observation/observation_of.hpp"
#include "murosim/robot/robot.hpp"
#include "murosim/simulation/simulation.hpp"

#include <memory>
#include <vector>

namespace murosimrl::observation
{

/// Base class for observations computed per robot (e.g. pose, velocity, lidar).
/// Subclasses implement compute()/shape() over a list of robots, as required
/// by ObservationOf; validate() additionally requires every robot to have at
/// least one recorded state. On top of the robot-list interface, this class
/// adds convenience overloads that accept simulations directly and flatten
/// them to their robots first, so callers can observe either robots or whole
/// simulations without extracting robot lists themselves.
class ObservationRobot : public ObservationOf<murosim::robot::Robot>
{
protected:
    /// @param name Identifies the observation.
    /// @param stack_size Number of stacked past states per robot; 0 and 1 are equivalent (no stacking).
    ObservationRobot(const std::string &name, const size_t stack_size);

    /// Require every robot to have at least one recorded state; throws std::runtime_error otherwise.
    void validate(const EntityList &robots) const override;

public:
    virtual ~ObservationRobot() = default;

    // Inherit the robot-list / single-robot overloads from the base, then add
    // adapters that flatten simulations to their robots.
    using ObservationOf::operator();
    using ObservationOf::shape;

    /// Compute the observation for the robots of several simulations into `output`.
    /// @param simulations Simulations whose robots are observed; each must have at least one robot.
    /// @param output Buffer to write into; sized per shape(simulations).
    void operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const;

    /// Compute the observation for the robots of a single simulation into `output`.
    /// @param simulation Simulation whose robots are observed; must have at least one robot.
    /// @param output Buffer to write into; sized per shape(simulation).
    void operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, float* output) const;

    /// Compute the observation for the robots of several simulations and return it as a new vector.
    /// @param simulations Simulations whose robots are observed; each must have at least one robot.
    std::vector<float> operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

    /// Compute the observation for the robots of a single simulation and return it as a new vector.
    /// @param simulation Simulation whose robots are observed; must have at least one robot.
    std::vector<float> operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;

    /// Report the output shape for observing the robots of several simulations.
    std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

    /// Report the output shape for observing the robots of a single simulation.
    std::vector<size_t> shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;

private:
    /// Flatten several simulations to their combined robot list; throws if any simulation has no robots.
    std::vector<std::shared_ptr<murosim::robot::Robot>> extractRobots(
        const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

    /// Return a single simulation's robot list; throws std::invalid_argument if it has none.
    std::vector<std::shared_ptr<murosim::robot::Robot>> extractRobots(
        const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;
};

} // namespace murosimrl::observation

#endif // MUROSIMRL_OBSERVATION_ROBOT_HPP_
