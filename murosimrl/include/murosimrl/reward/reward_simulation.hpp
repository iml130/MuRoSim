// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_SIMULATION_HPP_
#define MUROSIMRL_REWARD_SIMULATION_HPP_

#include "murosim/simulation/simulation.hpp"

#include <memory>
#include <vector>
#include <string>

namespace murosimrl::reward
{

/// Base class for rewards computed per simulation, independent of any
/// particular robot. Subclasses provide compute() (write one scalar reward
/// per simulation into a caller-owned buffer) and may override shape() if
/// their output is not a single scalar (the default is one float per
/// simulation: {simulations.size(), 1}). Empty input lists are handled here
/// (operator() is a no-op on empty input rather than calling compute()).
class RewardSimulation
{
private:
    std::string name_;

protected:
    /// @param name Identifies the reward, e.g. for logging or lookup by name.
    RewardSimulation(const std::string &name);

    /// Write the reward for every simulation in `simulations` into `output`, in list order.
    /// @param simulations Non-empty list of simulations to evaluate.
    /// @param output Caller-owned buffer sized to match shape(simulations); must not be null.
    virtual void compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const = 0;

public:
    virtual ~RewardSimulation() = default;

    /// Report the output shape for evaluating `simulations`. Default: {simulations.size(), 1} (one scalar per simulation).
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

    /// Compute the reward for `simulations` into `output`. No-op if `simulations` is empty.
    void operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const;
    /// Compute the reward for a single simulation into `output`.
    void operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, float* output) const;

    /// Compute the reward for `simulations` and return it as a newly allocated vector.
    std::vector<float> operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;
    /// Compute the reward for a single simulation and return it as a newly allocated vector.
    std::vector<float> operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;

    /// Report the output shape for evaluating a single simulation.
    std::vector<size_t> shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;

    /// The reward's name, as given at construction.
    const std::string& name() const;

private:
    /// Product of all dimensions in a shape vector; 0 for an empty shape.
    size_t computeTotalSize(const std::vector<size_t> &shape) const;
};

} // namespace murosimrl::reward

#endif // MUROSIMRL_REWARD_SIMULATION_HPP_
