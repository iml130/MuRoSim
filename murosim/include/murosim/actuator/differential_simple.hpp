// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ACTUATOR_DIFFERENTIAL_SIMPLE_HPP_
#define MUROSIM_ACTUATOR_DIFFERENTIAL_SIMPLE_HPP_

#include "actuator.hpp"

namespace murosim::actuator
{

/// Simplified differential-drive actuator (TYPE::X_ANGULAR: linear x and angular velocity only), with no
/// wheel-level kinematics or hardware-specific limits — velocity is integrated directly per the current
/// CONTROL_MODE. Velocities are in m/s and rad/s, accelerations in m/s^2 and rad/s^2.
class DifferentialSimple: public Actuator
{
protected:
    Eigen::Vector3f move(const float dt) override;

public:
    /// Default limits: +-1 m/s linear, +-pi rad/s angular, no acceleration limit (commands apply instantly).
    DifferentialSimple();
    /// Custom velocity limits, no acceleration limit.
    DifferentialSimple(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max);
    /// Custom velocity and acceleration limits.
    DifferentialSimple(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular);
    /// @return a deep copy of this actuator.
    virtual std::unique_ptr<Actuator> clone() const override;
    /// Zeroes commanded and realized velocity/acceleration.
    virtual void reset() override;
    /// @return TYPE::X_ANGULAR.
    virtual TYPE getType() const override;
    virtual std::ostream& print(std::ostream& out) const override;
};

} // murosim::actuator

#endif // MUROSIM_ACTUATOR_DIFFERENTIAL_SIMPLE_HPP_
