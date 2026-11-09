// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/reward_robot.hpp"

namespace murosimrl::reward
{

RewardRobot::RewardRobot(const std::string &name)
    : name_(name)
{
}

std::vector<size_t> RewardRobot::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    return {robots.size(), 1};
}

void RewardRobot::computeRange(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots,
                               const size_t begin, const size_t end, float* output) const
{
    const std::vector<std::shared_ptr<murosim::robot::Robot>> sub(
        robots.begin() + static_cast<std::ptrdiff_t>(begin),
        robots.begin() + static_cast<std::ptrdiff_t>(end));
    compute(sub, output);
}

void RewardRobot::operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    if (!robots.empty())
    {
        compute(robots, output);
    }
}

void RewardRobot::operator()(const std::shared_ptr<murosim::robot::Robot> &robot, float* output) const
{
    operator()(std::vector<std::shared_ptr<murosim::robot::Robot>>{robot}, output);
}

void RewardRobot::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots = extractRobots(simulations);
    operator()(robots, output);
}

void RewardRobot::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation, float* output) const
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots = extractRobots(simulation);
    operator()(robots, output);
}

std::vector<float> RewardRobot::operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    std::vector<size_t> s = shape(robots);
    size_t total_size = computeTotalSize(s);
    std::vector<float> result(total_size);

    if (!result.empty())
    {
        operator()(robots, result.data());
    }

    return result;
}

std::vector<float> RewardRobot::operator()(const std::shared_ptr<murosim::robot::Robot> &robot) const
{
    return operator()(std::vector<std::shared_ptr<murosim::robot::Robot>>{robot});
}

std::vector<float> RewardRobot::operator()(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots = extractRobots(simulations);
    return operator()(robots);
}

std::vector<float> RewardRobot::operator()(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots = extractRobots(simulation);
    return operator()(robots);
}

std::vector<size_t> RewardRobot::shape(const std::shared_ptr<murosim::robot::Robot> &robot) const
{
    return shape(std::vector<std::shared_ptr<murosim::robot::Robot>>{robot});
}

std::vector<size_t> RewardRobot::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots = extractRobots(simulations);
    return shape(robots);
}

std::vector<size_t> RewardRobot::shape(const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots = extractRobots(simulation);
    return shape(robots);
}

const std::string& RewardRobot::name() const
{
    return name_;
}

std::vector<std::shared_ptr<murosim::robot::Robot>> RewardRobot::extractRobots(
    const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots;

    for (const auto& simulation : simulations)
    {
        for (const auto& robot : simulation->getRobots())
        {
            robots.emplace_back(robot);
        }
    }

    return robots;
}

std::vector<std::shared_ptr<murosim::robot::Robot>> RewardRobot::extractRobots(
    const std::shared_ptr<murosim::simulation::Simulation> &simulation) const
{
    return simulation->getRobots();
}

size_t RewardRobot::computeTotalSize(const std::vector<size_t> &shape) const
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
