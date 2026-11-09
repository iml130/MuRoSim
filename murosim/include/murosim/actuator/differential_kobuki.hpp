// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ACTUATOR_DIFFERENTIAL_KOBUKI_HPP_
#define MUROSIM_ACTUATOR_DIFFERENTIAL_KOBUKI_HPP_

#include "actuator.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>

namespace murosim::actuator
{

/// Differential-drive actuator modeling a Kobuki base (TYPE::X_ANGULAR: linear x and angular velocity only).
/// Commands are converted to/from left/right wheel speeds and clamped to the Kobuki's fixed wheel-speed
/// limit, reproducing the wheel-slip-avoidance behavior of the real hardware. Velocities are in m/s,
/// angular velocities in rad/s, accelerations in m/s^2 and rad/s^2.
class DifferentialKobuki : public Actuator
{
public:
    /// Hardware limits of the Kobuki base. Velocity limits are the defaults of the default constructor;
    /// acceleration limits passed to a constructor are capped to the values here. WHEEL_SPEED_MAX caps the
    /// realised speed of each wheel, so a configured linear velocity limit above it is never reached and
    /// turning while driving at full speed is not possible.
    static constexpr float VELOCITY_LINEAR_MAX = 0.7f;       ///< [m/s]
    static constexpr float VELOCITY_ANGULAR_MAX = murosim::math::PI;     ///< [rad/s]
    static constexpr float ACCELERATION_LINEAR_MAX = 2.5f;   ///< [m/s^2]
    static constexpr float ACCELERATION_ANGULAR_MAX = 32.0f; ///< [rad/s^2]
    static constexpr float WHEEL_SPEED_MAX = 0.7f;           ///< [m/s] per wheel

private:
    /// Differential-drive forward kinematics: wheel speeds (rad/s) to body linear (m/s) and angular (rad/s) velocity.
    void forward(const double left, const double right, double &linear, double &angular) const;
    /// Differential-drive inverse kinematics: body linear/angular velocity to left/right wheel speeds (rad/s).
    void inverse(const double linear, const double angular, double &left, double &right) const;
    /// Clamps left/right wheel speeds to the Kobuki's fixed physical maximum.
    void limitWheelSpeed(double &left, double &right);

protected:
    Eigen::Vector3f move(const float dt) override;

public:
    /// Default limits: +-VELOCITY_LINEAR_MAX, +-VELOCITY_ANGULAR_MAX, ACCELERATION_LINEAR_MAX / ACCELERATION_ANGULAR_MAX.
    DifferentialKobuki();
    /// Custom velocity limits with the default Kobuki acceleration limits.
    DifferentialKobuki(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max);
    /// Custom velocity and acceleration limits; acceleration is capped at ACCELERATION_LINEAR_MAX / ACCELERATION_ANGULAR_MAX.
    DifferentialKobuki(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular);

    /// @return a deep copy of this actuator.
    virtual std::unique_ptr<Actuator> clone() const override;
    /// Zeroes commanded and realized velocity/acceleration.
    virtual void reset() override;
    /// @return TYPE::X_ANGULAR.
    virtual TYPE getType() const override;
    virtual std::ostream& print(std::ostream& out) const override;
};

} // namespace murosim::actuator

#endif // MUROSIM_ACTUATOR_DIFFERENTIAL_KOBUKI_HPP_
