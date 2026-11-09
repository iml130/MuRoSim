// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_ROBOT_HPP_
#define MUROSIMRL_REWARD_ROBOT_HPP_

#include "murosim/robot/robot.hpp"
#include "murosim/simulation/simulation.hpp"

#include <memory>
#include <vector>
#include <string>

namespace murosimrl::reward
{

/// Base class for rewards computed per robot. This is the contract every
/// concrete robot reward implements: subclasses provide compute() (write one
/// scalar reward per robot into a caller-owned buffer, in robot order), and
/// may override shape() if their per-robot output is not a single scalar
/// (the default is one float per robot: {robots.size(), 1}). Empty input
/// lists are handled here (operator() is a no-op on empty input rather than
/// calling compute()), and this class provides overloads that accept
/// simulations directly, flattening them to their combined robot list first.
/// Concrete subclasses live under reward/robot/.
class RewardRobot
{
private:
    std::string name_;

protected:
    /// @param name Identifies the reward, e.g. for logging or lookup by name.
    RewardRobot(const std::string &name);

    /// Write the reward for every robot in `robots` into `output`, in robot order.
    /// @param robots Non-empty list of robots to evaluate.
    /// @param output Caller-owned buffer sized to match shape(robots); must not be null.
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const = 0;

public:
    virtual ~RewardRobot() = default;

    /// Report the output shape for evaluating `robots`. Default: {robots.size(), 1} (one scalar per robot).
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;

    /// Compute robots [begin, end) only; `output` points at the slice for
    /// `begin` (one float per robot). Default: sub-list + compute().
    /// @param robots Full robot list.
    /// @param begin Index of the first robot in the range (inclusive).
    /// @param end Index one past the last robot in the range (exclusive).
    /// @param output Buffer positioned at the start of this range's slice.
    virtual void computeRange(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots,
                              const size_t begin, const size_t end, float* output) const;

    /// Compute the reward for `robots` into `output`. No-op if `robots` is empty.
    void operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const;
    /// Compute the reward for a single robot into `output`.
    void operator()(const std::shared_ptr<murosim::robot::Robot> &robot, float* output) const;
    /// Compute the reward for the combined robots of several simulations into `output`.
    void operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const;
    /// Compute the reward for the robots of a single simulation into `output`.
    void operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, float* output) const;

    /// Compute the reward for `robots` and return it as a newly allocated vector.
    std::vector<float> operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;
    /// Compute the reward for a single robot and return it as a newly allocated vector.
    std::vector<float> operator()(const std::shared_ptr<murosim::robot::Robot> &robot) const;
    /// Compute the reward for the combined robots of several simulations and return it as a new vector.
    std::vector<float> operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;
    /// Compute the reward for the robots of a single simulation and return it as a new vector.
    std::vector<float> operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;

    /// Report the output shape for evaluating a single robot.
    std::vector<size_t> shape(const std::shared_ptr<murosim::robot::Robot> &robot) const;
    /// Report the output shape for evaluating the combined robots of several simulations.
    std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;
    /// Report the output shape for evaluating the robots of a single simulation.
    std::vector<size_t> shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;

    /// The reward's name, as given at construction.
    const std::string& name() const;

private:
    /// Flatten several simulations to their combined robot list.
    std::vector<std::shared_ptr<murosim::robot::Robot>> extractRobots(
        const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

    /// Return a single simulation's robot list.
    std::vector<std::shared_ptr<murosim::robot::Robot>> extractRobots(
        const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;

    /// Product of all dimensions in a shape vector; 0 for an empty shape.
    size_t computeTotalSize(const std::vector<size_t> &shape) const;
};

} // namespace murosimrl::reward

#endif // MUROSIMRL_REWARD_ROBOT_HPP_
