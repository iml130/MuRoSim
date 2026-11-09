// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/util/robot.hpp"
#include <murosim/robot/util.hpp>

#include <stdexcept>
#include <string>

namespace murosimrl::util::robot
{

bool checkRobotHasSameActuator(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots)
{
    if (1 < robots.size())
    {
        const murosim::actuator::Actuator::TYPE type = robots[0]->getActuatorType();

        for (size_t i = 1; i < robots.size(); i++)
        {
            if (type != robots[i]->getActuatorType())
            {
                return false;
            }
        }
    }

    return true;
}

bool checkRobotHasStates(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots)
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        if(robots[i]->getStateSize() == 0)
        {
            return false;
        }
    }

    return true;
}

size_t axesForActuatorType(const murosim::actuator::Actuator::TYPE type, std::array<size_t, 3> &axes)
{
    switch (type)
    {
    case murosim::actuator::Actuator::TYPE::X_ANGULAR:
        axes = {0, 2, 0};
        return 2;

    case murosim::actuator::Actuator::TYPE::X_Y:
        axes = {0, 1, 0};
        return 2;

    case murosim::actuator::Actuator::TYPE::X_Y_ANGULAR:
        axes = {0, 1, 2};
        return 3;

    default:
        throw std::runtime_error("Unknown actuator type");
    }
}

void requireConsistentLidars(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const size_t index_lidar)
{
    for (size_t i = 0; i < robots.size(); i++)
    {
        if (robots[i]->getLidars().size() <= index_lidar)
        {
            throw std::runtime_error("Lidar index " + std::to_string(index_lidar) +
                " out of range for robot " + std::to_string(i) +
                " (has " + std::to_string(robots[i]->getLidars().size()) + " lidars)");
        }
    }

    const size_t size_lidar = robots[0]->getLidars()[index_lidar].getLaserSize();

    for (size_t i = 1; i < robots.size(); i++)
    {
        if (size_lidar != robots[i]->getLidars()[index_lidar].getLaserSize())
        {
            throw std::runtime_error("Lidar size mismatch: robot 0 has " + std::to_string(size_lidar) +
                " lasers, robot " + std::to_string(i) + " has " +
                std::to_string(robots[i]->getLidars()[index_lidar].getLaserSize()));
        }
    }
}

Eigen::Vector2f orientationToGoalFromState(const murosim::robot::State &state)
{
    return murosim::robot::util::orientationToGoal(state.getPose(), state.getGoal());
}

float distanceToGoalFromState(const murosim::robot::State &state)
{
    return murosim::robot::util::distanceToGoal(state.getPose(), state.getGoal());
}

} // namespace murosimrl::util::robot
