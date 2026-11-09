// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ACTUATOR_HOLONOMIC_SIMPLE_HPP_
#define MUROSIM_ACTUATOR_HOLONOMIC_SIMPLE_HPP_

#include "actuator.hpp"

namespace murosim::actuator
{

/// Simplified fully holonomic actuator (TYPE::X_Y_ANGULAR: independent x, y and angular velocity), with
/// no wheel-level kinematics — velocity is integrated directly per the current CONTROL_MODE. Velocities
/// are in m/s and rad/s, accelerations in m/s^2 and rad/s^2.
class HolonomicSimple:public Actuator
{
protected:
    Eigen::Vector3f move(const float dt) override;

public:
    /// Default limits: +-1 m/s on x and y, +-pi rad/s angular, no acceleration limit (commands apply instantly).
    HolonomicSimple();
    /// Custom velocity limits, no acceleration limit.
    HolonomicSimple(const float linear_x_min, const float linear_x_max, const float linear_y_max, const float angular_max);
    /// Custom velocity and acceleration limits (the linear acceleration limit applies to both x and y).
    HolonomicSimple(const float linear_x_min, const float linear_x_max, const float linear_y_max, const float angular_max, const float acceleration_linear, const float acceleration_angular);

    /// @return a deep copy of this actuator.
    virtual std::unique_ptr<Actuator> clone() const override;
    /// Zeroes commanded and realized velocity/acceleration.
    virtual void reset() override;
    /// @return TYPE::X_Y_ANGULAR.
    virtual TYPE getType() const override;
    virtual std::ostream& print(std::ostream& out) const override;
};

} // namespace murosim::actuator

#endif // MUROSIM_ACTUATOR_HOLONOMIC_SIMPLE_HPP_
