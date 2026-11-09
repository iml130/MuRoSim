// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/action.hpp"
#include "murosimrl/util/robot.hpp"
#include "murosim/actuator/actuator.hpp"

#include <array>
#include <stdexcept>

namespace
{

// The [-1, 1] action range maps onto the command range of the robot's control
// mode: the velocity limits in VELOCITY mode, the symmetric [-max, max]
// acceleration range in ACCELERATION mode. Returns whether the robot is in
// ACCELERATION mode.
bool actionCommandRange(const murosim::robot::Robot &robot, const std::array<size_t, 3> &axes, const size_t axis_count, Eigen::Vector3f &min, Eigen::Vector3f &max)
{
    if (robot.getActuatorControlMode() == murosim::actuator::CONTROL_MODE::ACCELERATION)
    {
        max = robot.getActuatorAccelerationMax();
        min = -max;

        // an axis without an acceleration limit cannot be commanded at all in
        // this mode (setCommandAcceleration clamps it to zero) - fail loudly
        // instead of training a policy whose actions are silently inert
        for (size_t k = 0; k < axis_count; k++)
        {
            if (max[static_cast<Eigen::Index>(axes[k])] <= 0.0f)
            {
                throw std::runtime_error("ACCELERATION control mode requires an acceleration limit on every actuated axis!");
            }
        }
        return true;
    }

    min = robot.getActuatorVelocityMin();
    max = robot.getActuatorVelocityMax();
    return false;
}

// Writes one robot's command (velocity or acceleration, per its control mode)
// from `data` (one float per actuated axis) and returns the pointer advanced
// past the consumed values.
const float* applyActionToRobot(murosim::robot::Robot &robot, const float* data, const std::array<size_t, 3> &axes, const size_t axis_count, const bool normalize)
{
    Eigen::Vector3f min, max;
    const bool acceleration_mode = actionCommandRange(robot, axes, axis_count, min, max);

    // Axes the actuator does not actuate stay 0 in the command.
    Eigen::Vector3f command = Eigen::Vector3f::Zero();

    if (normalize)
    {
        for (size_t k = 0; k < axis_count; k++)
        {
            const Eigen::Index axis = static_cast<Eigen::Index>(axes[k]);
            command[axis] = murosim::actuator::Actuator::fromRelativeToAbsolute(*data, min[axis], max[axis]);
            data++;
        }
    }
    else
    {
        for (size_t k = 0; k < axis_count; k++)
        {
            command[static_cast<Eigen::Index>(axes[k])] = *data;
            data++;
        }
    }

    if (acceleration_mode)
    {
        robot.setActuatorCommandAcceleration(command);
    }
    else
    {
        robot.setActuatorCommandVelocity(command);
    }
    return data;
}

} // anonymous namespace

namespace murosimrl
{

Action::Action(const bool normalize)
    : normalize_(normalize)
{
}

size_t Action::size(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    std::array<size_t, 3> axes;
    return util::robot::axesForActuatorType(robot->getActuatorType(), axes);
}

std::vector<size_t> Action::shape(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    return {size(robot)};
}

std::vector<size_t> Action::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots)
{
    if (robots.empty())
    {
        return {};
    }
    return {robots.size(), size(robots[0])};
}

void Action::operator()(std::shared_ptr<murosim::robot::Robot> &robot, const float* data) const
{
    // direct path: this runs per robot per RL step, so no temporary robot
    // list and no same-actuator check for a single robot
    std::array<size_t, 3> axes;
    const size_t axis_count = util::robot::axesForActuatorType(robot->getActuatorType(), axes);
    applyActionToRobot(*robot, data, axes, axis_count, normalize_);
}

void Action::operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const float* data) const
{
    if (robots.empty())
    {
        return;
    }

    if (!util::robot::checkRobotHasSameActuator(robots))
    {
        throw std::runtime_error("Robots have different actuator types!");
    }

    std::array<size_t, 3> axes;
    const size_t axis_count = util::robot::axesForActuatorType(robots[0]->getActuatorType(), axes);

    for (size_t i = 0; i < robots.size(); i++)
    {
        data = applyActionToRobot(*robots[i], data, axes, axis_count, normalize_);
    }
}

void Action::getNeutralPoint(const std::shared_ptr<murosim::robot::Robot> &robot, float* output)
{
    std::array<size_t, 3> axes;
    const size_t axis_count = util::robot::axesForActuatorType(robot->getActuatorType(), axes);

    // mode-aware: zero velocity or zero acceleration in the mode's range
    Eigen::Vector3f min, max;
    actionCommandRange(*robot, axes, axis_count, min, max);

    for (size_t k = 0; k < axis_count; k++)
    {
        const Eigen::Index axis = static_cast<Eigen::Index>(axes[k]);
        output[k] = murosim::actuator::Actuator::fromAbsoluteToRelative(0.0f, min[axis], max[axis]);
    }
}

std::vector<float> Action::getNeutralPoint(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    std::vector<float> result(size(robot));
    getNeutralPoint(robot, result.data());
    return result;
}

} // namespace murosimrl
