// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ACTUATOR_ROBOMASTER_HPP_
#define MUROSIM_ACTUATOR_ROBOMASTER_HPP_

#include "actuator.hpp"

namespace murosim::actuator
{

/// Mecanum-wheel actuator modeling a DJI RoboMaster base. Commands are converted to/from the four wheel
/// speeds via mecanum forward/inverse kinematics, and wheel speeds are further limited by both a
/// controller-level acceleration bound and an independent hardware wheel-slew-rate limit, so motion is
/// integrated in fixed 50 Hz sub-steps regardless of the requested dt. TYPE defaults to X_Y_ANGULAR (full
/// holonomic motion) but can be restricted to X_ANGULAR or X_Y via the constructor. Velocities are in m/s
/// and rad/s, accelerations in m/s^2 and rad/s^2.
class RoboMaster: public Actuator
{
public:
    /// Hardware limits of the RoboMaster base. VELOCITY_* are the defaults of the default constructor. The
    /// wheels cannot exceed WHEEL_RPM_MAX and need TIME_TO_MAX_RPM to get there from standstill (the
    /// wheel-slew-rate limit), independent of any acceleration limit passed to a constructor. Motion is
    /// integrated in sub-steps of SAMPLE_TIME regardless of the requested dt.
    static constexpr float VELOCITY_LINEAR_MAX = 3.0f;      ///< [m/s], x and y
    static constexpr float VELOCITY_ANGULAR_MAX = 2.0f;     ///< [rad/s]
    static constexpr float WHEEL_RPM_MAX = 1000.0f;         ///< [1/min] per wheel
    static constexpr float TIME_TO_MAX_RPM = 5.0f / 3.0f;   ///< [s] from standstill to WHEEL_RPM_MAX
    static constexpr float SAMPLE_TIME = 1.0f / 50.0f;      ///< [s] internal integration sub-step (50 Hz)

private:
    double w1_; // wheel front left rad/s
    double w2_; // wheel front right rad/s
    double w3_; // wheel rear left rad/s
    double w4_; // wheel rear right rad/s

    Actuator::TYPE type_;

    /// Advances the wheel speeds by one 50 Hz sub-step (dt is normally SAMPLE_TIME, or the
    /// leftover remainder) toward the control-mode target, applying the wheel-slew-rate limit, and returns
    /// the realized body-frame velocity.
    Eigen::Vector3f update(const float dt);
    /// @return the current body-frame velocity implied by the current wheel speeds (inverse kinematics).
    Eigen::Vector3f velocity() const;
    /// Clips the velocity delta from `velocity_current` toward `velocity_target` to the given per-dt
    /// acceleration bounds, preserving direction for the linear (x, y) component.
    static Eigen::Vector3f clipVelocityTarget(const Eigen::Vector3f &velocity_target, const Eigen::Vector3f &velocity_current, const float dt, const float acceleration_linear, const float acceleration_angular);
    /// Zeroes velocity_min_/velocity_max_ components on axes the constructed TYPE does not support.
    void correctVelocityLimits();
    /// Mecanum-wheel forward kinematics: wheel speeds (rad/s) to body velocity (x, y, angular).
    static void inverse(const double w1, const double w2, const double w3, const double w4, Eigen::Vector3f &velocity);
    /// Mecanum-wheel inverse kinematics: body velocity (x, y, angular) to wheel speeds (rad/s).
    static void forward(const Eigen::Vector3f& velocity, double &w1, double &w2, double &w3, double &w4);

protected:
    /// Integrates motion in fixed 50 Hz sub-steps (see update()) and reduces the composed sub-step pose
    /// delta back to a single equivalent body velocity for the given dt.
    Eigen::Vector3f move(const float dt) override;

public:
    /// Default limits: +-VELOCITY_LINEAR_MAX on x and y, +-VELOCITY_ANGULAR_MAX, no controller-level acceleration limit; TYPE::X_Y_ANGULAR.
    RoboMaster();
    /// Symmetric velocity limits, no controller-level acceleration limit.
    RoboMaster(const float velocity_linear_max, const float velocity_angular_max, const Actuator::TYPE type=Actuator::TYPE::X_Y_ANGULAR);
    /// Custom velocity limits, no controller-level acceleration limit.
    RoboMaster(const float velocity_linear_x_min, const float velocity_linear_x_max, const float velocity_linear_y_max, const float velocity_angular_max, const Actuator::TYPE type=Actuator::TYPE::X_Y_ANGULAR);
    /// Custom velocity and controller-level acceleration limits (in addition to the fixed hardware wheel-slew-rate limit).
    RoboMaster(const float velocity_linear_x_min, const float velocity_linear_x_max, const float velocity_linear_y_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular, const Actuator::TYPE type=Actuator::TYPE::X_Y_ANGULAR);

    /// @return a deep copy of this actuator.
    virtual std::unique_ptr<Actuator> clone() const override;
    /// Zeroes wheel speeds and commanded/realized velocity/acceleration.
    virtual void reset() override;
    /// @return the TYPE this instance was constructed with.
    virtual TYPE getType() const override;
    virtual std::ostream& print(std::ostream& out) const override;
};

} // namespace murosim::actuator

#endif // MUROSIM_ACTUATOR_ROBOMASTER_HPP_
