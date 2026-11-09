// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/observation_robot.hpp"
#include "murosimrl/util/conversion.hpp"
#include "murosimrl/util/robot.hpp"
#include "murosimrl/util/simulation.hpp"

#include <stdexcept>

namespace murosimrl::observation
{

ObservationRobot::ObservationRobot(const std::string &name, const size_t stack_size)
    : ObservationOf<murosim::robot::Robot>(name, stack_size)
{
}

void ObservationRobot::validate(const EntityList &robots) const
{
    if (!util::robot::checkRobotHasStates(robots))
    {
        throw std::runtime_error("Robots need at least one state to create observation!");
    }
}

void ObservationRobot::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    operator()(extractRobots(simulations), output);
}

void ObservationRobot::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, float* output) const
{
    operator()(extractRobots(simulation), output);
}

std::vector<float> ObservationRobot::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    return operator()(extractRobots(simulations));
}

std::vector<float> ObservationRobot::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    return operator()(extractRobots(simulation));
}

std::vector<size_t> ObservationRobot::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    return shape(extractRobots(simulations));
}

std::vector<size_t> ObservationRobot::shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    return shape(extractRobots(simulation));
}

std::vector<std::shared_ptr<murosim::robot::Robot>> ObservationRobot::extractRobots(
    const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    util::simulation::requireSimulationsHaveRobots(simulations);
    return util::conversion::fromVectorSimulationToVectorRobot(simulations);
}

std::vector<std::shared_ptr<murosim::robot::Robot>> ObservationRobot::extractRobots(
    const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    // direct path (this runs per observation per RL step): same robots-present
    // contract as the vector overload, without the temporary simulation list
    if (simulation->getRobots().empty())
    {
        throw std::invalid_argument("Cannot create observation from a simulation without robots!");
    }
    return simulation->getRobots();
}

} // namespace murosimrl::observation
