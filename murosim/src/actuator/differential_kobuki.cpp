// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/actuator/differential_kobuki.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
constexpr double WHEEL_BASE = 0.23;    // wheel to wheel, in [m]
constexpr double WHEEL_RADIUS = 0.035; // main wheel, in [m]
// wheel speed limit in rad/s; the public limit is expressed in m/s at the wheel
constexpr double WHEEL_SPEED_MAX_RAD = static_cast<double>(murosim::actuator::DifferentialKobuki::WHEEL_SPEED_MAX) / WHEEL_RADIUS;
} // namespace

namespace murosim::actuator
{

void DifferentialKobuki::forward(const double left, const double right, double &linear, double &angular) const
{
    // Standard differential-drive forward kinematics.
    linear = WHEEL_RADIUS * (left + right) / 2.0;
    angular = WHEEL_RADIUS * (right - left) / WHEEL_BASE;
}

void DifferentialKobuki::inverse(const double linear, const double angular, double &left, double &right) const
{
    // Standard differential-drive inverse kinematics.
    right = (2.0 * linear + angular * WHEEL_BASE) / (2.0 * WHEEL_RADIUS);
    left = 2.0 * linear / WHEEL_RADIUS - right;
}

void DifferentialKobuki::limitWheelSpeed(double &left, double &right)
{
    left = std::clamp(left, -WHEEL_SPEED_MAX_RAD, WHEEL_SPEED_MAX_RAD);
    right = std::clamp(right, -WHEEL_SPEED_MAX_RAD, WHEEL_SPEED_MAX_RAD);
}


DifferentialKobuki::DifferentialKobuki()
{
    velocity_min_.x() = -VELOCITY_LINEAR_MAX;
    velocity_max_.x() =  VELOCITY_LINEAR_MAX;
    velocity_min_.z() = -VELOCITY_ANGULAR_MAX;
    velocity_max_.z() =  VELOCITY_ANGULAR_MAX;

    acceleration_max_.x() = ACCELERATION_LINEAR_MAX;
    acceleration_max_.z() = ACCELERATION_ANGULAR_MAX;
}

DifferentialKobuki::DifferentialKobuki(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max)
{
    setVelocityLimits(0, velocity_linear_min, velocity_linear_max);
    setVelocityLimitsSymmetric(2, velocity_angular_max);

    acceleration_max_.x() = ACCELERATION_LINEAR_MAX;
    acceleration_max_.z() = ACCELERATION_ANGULAR_MAX;
}

DifferentialKobuki::DifferentialKobuki(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular)
{
    setVelocityLimits(0, velocity_linear_min, velocity_linear_max);
    setVelocityLimitsSymmetric(2, velocity_angular_max);

    // the Kobuki models a fixed physical maximum acceleration
    setAccelerationLimit(0, acceleration_linear, ACCELERATION_LINEAR_MAX);
    setAccelerationLimit(2, acceleration_angular, ACCELERATION_ANGULAR_MAX);
}

std::unique_ptr<Actuator> DifferentialKobuki::clone() const
{
    return std::make_unique<DifferentialKobuki>(*this);
}

void DifferentialKobuki::reset()
{
    command_acceleration_.setZero();
    command_velocity_.setZero();
    velocity_current_.setZero();
}

Eigen::Vector3f DifferentialKobuki::move(const float dt)
{
    double left;
    double right;

    const Eigen::Vector3f velocity = velocityFromControlMode(velocity_current_, dt);
    double velocity_linear = static_cast<double>(velocity.x());
    double velocity_angular = static_cast<double>(velocity.z());

    // Apply Kobuki-specific kinematics and wheel speed limits
    inverse(velocity_linear, velocity_angular, left, right);
    limitWheelSpeed(left, right);
    forward(left, right, velocity_linear, velocity_angular);

    velocity_current_.x() = static_cast<float>(velocity_linear);
    velocity_current_.z() = static_cast<float>(velocity_angular);

    return velocity_current_;
}

Actuator::TYPE DifferentialKobuki::getType() const
{
    return Actuator::TYPE::X_ANGULAR;
}

std::ostream& DifferentialKobuki::print(std::ostream& out) const
{
    out << "DifferentialKobuki {"
        << " type: " << getTypeAsString()
        << ", vel_x: [" << velocity_min_.x() << ", " << velocity_max_.x() << "]"
        << ", vel_angular: [" << velocity_min_.z() << ", " << velocity_max_.z() << "]"
        << ", acc_x: " << acceleration_max_.x()
        << ", acc_angular: " << acceleration_max_.z()
        << " }";
    return out;
}

} // murosim::actuator
