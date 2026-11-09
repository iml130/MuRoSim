// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/velocity.hpp"

namespace murosimrl::observation::robot
{

Velocity::Velocity(const bool normalize, const size_t stack_size)
    : VelocityBase("velocity", normalize, stack_size)
{
}

Eigen::Vector3f Velocity::velocityFromState(const murosim::robot::State &state) const
{
    return state.getActuatorVelocity();
}

} // namespace murosimrl::observation::robot
