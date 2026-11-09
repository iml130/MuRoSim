// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_DONE_HPP_
#define MUROSIMRL_DONE_HPP_

#include <murosim/robot/robot.hpp>
#include <murosim/simulation/simulation.hpp>
#include <memory>
#include <vector>

namespace murosimrl
{

/// Reports each robot's episode-termination flag (Robot::isFinished(), true
/// once a robot has reached its goal or collided). One bool per robot, in
/// robot order (or simulation then robot order, for the simulation
/// overloads); no aggregation is done here (a caller wanting a per-simulation
/// "all done" needs to combine these itself).
class Done
{
public:
    Done();

    /// Write each robot's isFinished() flag into `output`.
    void operator()(const std::shared_ptr<murosim::robot::Robot> &robot, bool* output) const;
    /// Write each robot's isFinished() flag into `output`, in robot order.
    void operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, bool* output) const;
    /// Write each of the simulation's robots' isFinished() flags into `output`, in robot order.
    void operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, bool* output) const;
    /// Write each robot's isFinished() flag into `output`, in simulation then robot order.
    void operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, bool* output) const;

    /// Return the robot's isFinished() flag as a single-element vector.
    std::vector<bool> operator()(const std::shared_ptr<murosim::robot::Robot> &robot) const;
    /// Return each robot's isFinished() flag, in robot order.
    std::vector<bool> operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const;
    /// Return each of the simulation's robots' isFinished() flags, in robot order.
    std::vector<bool> operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const;
    /// Return each robot's isFinished() flag, in simulation then robot order.
    std::vector<bool> operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

    /// Output shape for a single robot: {1}.
    static std::vector<size_t> shape(const std::shared_ptr<murosim::robot::Robot> &robot);
    /// Output shape for a robot list: {robots.size(), 1}.
    static std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots);
    /// Output shape for a single simulation: {simulation's robot count, 1}.
    static std::vector<size_t> shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation);
    /// Output shape for a simulation list: {total robot count, 1}.
    static std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations);
};

} // namespace murosimrl

#endif // MUROSIMRL_DONE_HPP_
