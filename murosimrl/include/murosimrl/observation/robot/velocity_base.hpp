// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_VELOCITY_BASE_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_VELOCITY_BASE_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Shared implementation for the velocity-like observations: one loop reads a
/// Vector3f from each stacked state via velocityFromState() and writes only the
/// axes the actuator type actuates. With normalize, each value is mapped to
/// [-1, 1] against the robot's OWN velocity limits
/// (murosim::actuator::Actuator::fromAbsoluteToRelative), so fleets with equal
/// actuator types but different limits normalize correctly per robot.
/// Feature size per stacked state equals the actuator's axis count (2 for
/// X_ANGULAR/X_Y, 3 for X_Y_ANGULAR; see util::robot::axesForActuatorType).
/// Subclasses only need to say which Vector3f field of State to read.
class VelocityBase : public ObservationRobot
{
private:
    bool normalize_;

protected:
    /// @param name Identifies the observation.
    /// @param normalize Scale each value to [-1, 1] against the robot's own actuator velocity limits.
    /// @param stack_size Number of stacked past values per robot; 0 and 1 are equivalent (no stacking).
    VelocityBase(const std::string &name, const bool normalize, const size_t stack_size);

    /// Extract the raw (x, y, angular) velocity to observe from a state; units per the actuator.
    virtual Eigen::Vector3f velocityFromState(const murosim::robot::State &state) const = 0;

    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

    /// All robots must share one actuator type so the feature axes line up.
    virtual void validate(const EntityList &robots) const override;

public:
    virtual ~VelocityBase() = default;

    /// Per-robot feature size is the actuator's axis count (2 or 3), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;

    /// Whether values are scaled to [-1, 1] against the robot's own velocity limits.
    bool normalize() const { return normalize_; }
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_VELOCITY_BASE_HPP_
