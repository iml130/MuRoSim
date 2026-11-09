// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/util/conversion.hpp"

namespace murosimrl::util::conversion
{

std::vector<std::shared_ptr<murosim::robot::Robot>> fromVectorSimulationToVectorRobot(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations)
{
    // Count total robots first
    size_t total_robots = 0;
    for (size_t i = 0; i < simulations.size(); i++)
    {
        total_robots += simulations[i]->getRobots().size();
    }

    // Reserve capacity to avoid reallocations
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots;
    robots.reserve(total_robots);

    for (size_t i = 0; i < simulations.size(); i++)
    {
        for (const auto& robot : simulations[i]->getRobots())
        {
            robots.push_back(robot);
        }
    }

    return robots;
}

} // namespace murosimrl::util::conversion
