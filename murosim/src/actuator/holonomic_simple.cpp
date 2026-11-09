// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/math/constants.hpp"

#include <cmath>

namespace murosim::actuator
{

HolonomicSimple::HolonomicSimple()
{
    velocity_min_.x() = -1.0f;
    velocity_max_.x() =  1.0f;
    velocity_min_.y() = -1.0f;
    velocity_max_.y() =  1.0f;
    velocity_min_.z() =  -murosim::math::PI;
    velocity_max_.z() =  murosim::math::PI;
}

HolonomicSimple::HolonomicSimple(const float linear_x_min, const float linear_x_max, const float linear_y_max, const float angular_max)
{
    setVelocityLimits(0, linear_x_min, linear_x_max);
    setVelocityLimitsSymmetric(1, linear_y_max);
    setVelocityLimitsSymmetric(2, angular_max);
}

HolonomicSimple::HolonomicSimple(const float linear_x_min, const float linear_x_max, const float linear_y_max, const float angular_max, const float acceleration_linear, const float acceleration_angular)
{
    setVelocityLimits(0, linear_x_min, linear_x_max);
    setVelocityLimitsSymmetric(1, linear_y_max);
    setVelocityLimitsSymmetric(2, angular_max);

    setAccelerationLimit(0, acceleration_linear);
    setAccelerationLimit(1, acceleration_linear);
    setAccelerationLimit(2, acceleration_angular);
}

std::unique_ptr<Actuator> HolonomicSimple::clone() const
{
    return std::make_unique<HolonomicSimple>(*this);
}

Eigen::Vector3f HolonomicSimple::move(const float dt)
{
    velocity_current_ = velocityFromControlMode(velocity_current_, dt);
    return velocity_current_;
}

void HolonomicSimple::reset()
{
    command_velocity_ = Eigen::Vector3f::Zero();
    command_acceleration_ = Eigen::Vector3f::Zero();
    velocity_current_ = Eigen::Vector3f::Zero();
}

Actuator::TYPE HolonomicSimple::getType() const
{
    return Actuator::TYPE::X_Y_ANGULAR;
}

std::ostream& HolonomicSimple::print(std::ostream& out) const
{
    out << "HolonomicSimple {"
        << " type: " << getTypeAsString()
        << ", vel_x: [" << velocity_min_.x() << ", " << velocity_max_.x() << "]"
        << ", vel_y: [" << velocity_min_.y() << ", " << velocity_max_.y() << "]"
        << ", vel_angular: [" << velocity_min_.z() << ", " << velocity_max_.z() << "]"
        << ", acc_x: " << acceleration_max_.x()
        << ", acc_y: " << acceleration_max_.y()
        << ", acc_angular: " << acceleration_max_.z()
        << " }";
    return out;
}

} // murosim::actuator
