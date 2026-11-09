// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/reward_simulation.hpp"

namespace murosimrl::reward
{

RewardSimulation::RewardSimulation(const std::string &name)
    : name_(name)
{
}

std::vector<size_t> RewardSimulation::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    return {simulations.size(), 1};
}

void RewardSimulation::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    if (!simulations.empty())
    {
        compute(simulations, output);
    }
}

void RewardSimulation::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, float* output) const
{
    operator()(std::vector<std::shared_ptr<murosim::simulation::Simulation>>{simulation}, output);
}

std::vector<float> RewardSimulation::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    std::vector<size_t> s = shape(simulations);
    size_t total_size = computeTotalSize(s);
    std::vector<float> result(total_size);

    if (!result.empty())
    {
        operator()(simulations, result.data());
    }

    return result;
}

std::vector<float> RewardSimulation::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    return operator()(std::vector<std::shared_ptr<murosim::simulation::Simulation>>{simulation});
}

std::vector<size_t> RewardSimulation::shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    return shape(std::vector<std::shared_ptr<murosim::simulation::Simulation>>{simulation});
}

const std::string& RewardSimulation::name() const
{
    return name_;
}

size_t RewardSimulation::computeTotalSize(const std::vector<size_t> &shape) const
{
    if (shape.empty()) return 0;

    size_t total = 1;
    for (size_t dim : shape)
    {
        total *= dim;
    }
    return total;
}

} // namespace murosimrl::reward
