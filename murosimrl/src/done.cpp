// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/done.hpp"
#include "murosimrl/util/simulation.hpp"

namespace murosimrl
{

Done::Done()
{
}

std::vector<size_t> Done::shape(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    (void) robot;
    return {1};
}

std::vector<size_t> Done::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots)
{
    return {robots.size(), 1};
}

std::vector<size_t> Done::shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation)
{
    return {simulation->getRobots().size(), 1};
}

std::vector<size_t> Done::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations)
{
    return {util::simulation::getNumberOfRobots(simulations), 1};
}

void Done::operator()(const std::shared_ptr<murosim::robot::Robot> &robot, bool* output) const
{
    output[0] = robot->isFinished();
}

void Done::operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, bool* output) const
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        output[i] = robots[i]->isFinished();
    }
}

void Done::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, bool* output) const
{
    const auto& robots = simulation->getRobots();
    for (size_t i = 0; i < robots.size(); i++)
    {
        output[i] = robots[i]->isFinished();
    }
}

void Done::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, bool* output) const
{
    size_t index = 0;

    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        for (const auto& robot : sim->getRobots())
        {
            output[index++] = robot->isFinished();
        }
    }
}

std::vector<bool> Done::operator()(const std::shared_ptr<murosim::robot::Robot> &robot) const
{
    std::vector<bool> result(1);
    bool temp;
    operator()(robot, &temp);
    result[0] = temp;
    return result;
}

std::vector<bool> Done::operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    std::vector<bool> result(robots.size());
    for (size_t i = 0; i < robots.size(); i++)
    {
        result[i] = robots[i]->isFinished();
    }
    return result;
}

std::vector<bool> Done::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    const auto& robots = simulation->getRobots();
    std::vector<bool> result(robots.size());
    for (size_t i = 0; i < robots.size(); i++)
    {
        result[i] = robots[i]->isFinished();
    }
    return result;
}

std::vector<bool> Done::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    std::vector<bool> result(util::simulation::getNumberOfRobots(simulations));
    size_t index = 0;
    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        for (const auto& robot : sim->getRobots())
        {
            result[index++] = robot->isFinished();
        }
    }
    return result;
}

} // namespace murosimrl
