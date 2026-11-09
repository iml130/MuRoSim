// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/util/simulation.hpp"

#include <stdexcept>

namespace murosimrl::util::simulation
{

void requireGridMapFitSize(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, const size_t map_size, const std::string &observation)
{
    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        const murosim::grid::Map &map = sim->getGridMap();

        if (map_size < map.sizeX() || map_size < map.sizeY())
        {
            throw std::runtime_error(
                observation + ": grid map (" + std::to_string(map.sizeX()) + "x" +
                std::to_string(map.sizeY()) + ") exceeds map_size (" +
                std::to_string(map_size) + ")");
        }
    }
}

void requireSimulationsHaveRobots(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations)
{
    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        if (sim->getRobots().empty())
        {
            throw std::invalid_argument("Cannot create observation from a simulation without robots!");
        }
    }
}

bool checkRobotHasStates(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations)
{
    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        for (const auto& robot : sim->getRobots())
        {
            if (!robot->hasStates())
            {
                return false;
            }
        }
    }

    return true;
}

size_t getNumberOfRobots(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations)
{
    size_t number_of_robots = 0;

    for (size_t i = 0; i < simulations.size(); i++)
    {
        number_of_robots += simulations[i]->getRobots().size();
    }

    return number_of_robots;
}

} // namespace murosimrl::util::simulation
