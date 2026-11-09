// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_COMMAND_VELOCITY_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_COMMAND_VELOCITY_HPP_

#include "murosimrl/observation/robot/velocity_base.hpp"

namespace murosimrl::observation::robot
{

/// The velocity command last sent to the robot's actuator, per
/// State::getActuatorCommandVelocity(). See VelocityBase for feature shape,
/// units and normalization.
class CommandVelocity : public VelocityBase
{
protected:
    virtual Eigen::Vector3f velocityFromState(const murosim::robot::State &state) const override;

public:
    /// @param normalize Scale each value to [-1, 1] against the robot's own actuator velocity limits.
    /// @param stack_size Number of stacked past values per robot; 0 and 1 are equivalent (no stacking).
    CommandVelocity(const bool normalize, const size_t stack_size);
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_COMMAND_VELOCITY_HPP_
