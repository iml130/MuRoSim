// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/velocity_base.hpp"
#include "murosimrl/util/robot.hpp"
#include "murosim/actuator/actuator.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace murosimrl::observation::robot
{

VelocityBase::VelocityBase(const std::string &name, const bool normalize, const size_t stack_size)
    : ObservationRobot(name, stack_size)
    , normalize_(normalize)
{
}

void VelocityBase::validate(const EntityList &robots) const
{
    ObservationRobot::validate(robots);

    if (!util::robot::checkRobotHasSameActuator(robots))
    {
        throw std::runtime_error("Robots have different actuator types!");
    }
}

std::vector<size_t> VelocityBase::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    if (robots.empty()) return {};

    std::array<size_t, 3> axes;
    const size_t axis_count = util::robot::axesForActuatorType(robots[0]->getActuatorType(), axes);

    return ObservationBase::shape(robots.size(), {axis_count});
}

void VelocityBase::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const size_t stack_size = effectiveStackSize();

    std::array<size_t, 3> axes;
    const size_t axis_count = util::robot::axesForActuatorType(robots[0]->getActuatorType(), axes);

    size_t index = 0;

    for (size_t i = 0; i < robots.size(); i++)
    {
        const murosim::robot::Robot &robot = *robots[i];
        const Eigen::Vector3f velocity_min = robot.getActuatorVelocityMin();
        const Eigen::Vector3f velocity_max = robot.getActuatorVelocityMax();

        for (size_t j = 0; j < stack_size; j++)
        {
            const Eigen::Vector3f velocity = velocityFromState(robot.getStateAtOrOldest(j));

            for (size_t k = 0; k < axis_count; k++)
            {
                const Eigen::Index axis = static_cast<Eigen::Index>(axes[k]);
                output[index++] = (normalize_)
                    ? murosim::actuator::Actuator::fromAbsoluteToRelative(velocity[axis], velocity_min[axis], velocity_max[axis])
                    : velocity[axis];
            }
        }
    }
}

} // namespace murosimrl::observation::robot
