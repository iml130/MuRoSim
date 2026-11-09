// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/delta_pose.hpp"
#include "murosim/robot/util.hpp"

#include <algorithm>

namespace murosimrl::observation::robot
{

DeltaPose::DeltaPose(const size_t stack_size)
    : ObservationRobot("delta_pose", stack_size)
{
}

std::vector<size_t> DeltaPose::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    return ObservationBase::shape(robots.size(), {3});
}

void DeltaPose::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const size_t stack_size = effectiveStackSize();
    size_t index = 0;

    for (size_t i = 0; i < robots.size(); i++)
    {
        const murosim::robot::Robot &robot = *robots[i];

        for (size_t j = 0; j < stack_size; j++)
        {
            const Eigen::Affine2f current = robot.getStateAtOrOldest(j).getPose();
            const Eigen::Affine2f previous = robot.getStateAtOrOldest(j + 1).getPose();
            const Eigen::Vector3f delta = murosim::robot::util::deltaPose(current, previous);

            output[index++] = delta.x();
            output[index++] = delta.y();
            output[index++] = delta.z();
        }
    }
}

} // namespace murosimrl::observation::robot
