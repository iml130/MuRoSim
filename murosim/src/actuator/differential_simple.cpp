// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/actuator/differential_simple.hpp"
#include "murosim/math/constants.hpp"

#include <cmath>
#include <memory>

namespace murosim::actuator
{

DifferentialSimple::DifferentialSimple()
{
    velocity_min_.x() = -1.0f;
    velocity_max_.x() =  1.0f;
    velocity_min_.z() = -murosim::math::PI;
    velocity_max_.z() = murosim::math::PI;
}

DifferentialSimple::DifferentialSimple(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max)
{
    setVelocityLimits(0, velocity_linear_min, velocity_linear_max);
    setVelocityLimitsSymmetric(2, velocity_angular_max);
}

DifferentialSimple::DifferentialSimple(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular)
{
    setVelocityLimits(0, velocity_linear_min, velocity_linear_max);
    setVelocityLimitsSymmetric(2, velocity_angular_max);

    setAccelerationLimit(0, acceleration_linear);
    setAccelerationLimit(2, acceleration_angular);
}

std::unique_ptr<Actuator> DifferentialSimple::clone() const
{
    return std::make_unique<DifferentialSimple>(*this);
}

Eigen::Vector3f DifferentialSimple::move(const float dt)
{
    velocity_current_ = velocityFromControlMode(velocity_current_, dt);
    return velocity_current_;
}

void DifferentialSimple::reset()
{
    velocity_current_.setZero();
    command_velocity_.setZero();
    command_acceleration_.setZero();
}

Actuator::TYPE DifferentialSimple::getType() const
{
    return Actuator::TYPE::X_ANGULAR;
}

std::ostream& DifferentialSimple::print(std::ostream& out) const
{
    out << "DifferentialSimple {"
        << " type: " << getTypeAsString()
        << ", vel_x: [" << velocity_min_.x() << ", " << velocity_max_.x() << "]"
        << ", vel_angular: [" << velocity_min_.z() << ", " << velocity_max_.z() << "]"
        << ", acc_x: " << acceleration_max_.x()
        << ", acc_angular: " << acceleration_max_.z()
        << " }";
    return out;
}

} // murosim::actuator
