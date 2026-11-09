// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/command_velocity.hpp"

namespace murosimrl::observation::robot
{

CommandVelocity::CommandVelocity(const bool normalize, const size_t stack_size)
    : VelocityBase("command_velocity", normalize, stack_size)
{
}

Eigen::Vector3f CommandVelocity::velocityFromState(const murosim::robot::State &state) const
{
    return state.getActuatorCommandVelocity();
}

} // namespace murosimrl::observation::robot
