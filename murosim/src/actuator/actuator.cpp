// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/actuator/actuator.hpp"
#include "murosim/geometry/primitives.hpp"
#include <cctype>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace
{

float axisVelocityTowards(const float current, const float target, const float acceleration, const float dt)
{
    if (0.0f < acceleration)
    {
        return current + std::clamp(target - current, -acceleration * dt, acceleration * dt);
    }

    // no acceleration limit: reach the target instantly
    return target;
}

} // namespace

namespace murosim::actuator
{

float Actuator::fromRelativeToAbsolute(const float relative, const float absolute_min, const float absolute_max)
{
    return relative * (absolute_max - absolute_min) * 0.5f + (absolute_min + absolute_max) * 0.5f;
}

float Actuator::fromAbsoluteToRelative(const float absolute, const float absolute_min, const float absolute_max)
{
    const float range = absolute_max - absolute_min;
    if (range == 0.0f)
    {
        return 0.0f;
    }
    return (2.0f * absolute - absolute_min - absolute_max) / range;
}

Actuator::Actuator()
    : velocity_current_(Eigen::Vector3f::Zero()),
      velocity_min_(Eigen::Vector3f::Zero()),
      velocity_max_(Eigen::Vector3f::Zero()),
      acceleration_max_(Eigen::Vector3f::Zero()),
      command_velocity_(Eigen::Vector3f::Zero()),
      command_acceleration_(Eigen::Vector3f::Zero()),
      control_mode_(CONTROL_MODE::VELOCITY)
{
}

Actuator::~Actuator()
{

}

void Actuator::setCommandVelocity(const Eigen::Vector3f &cmd_vel)
{
    setCommandVelocity(cmd_vel.x(), cmd_vel.y(), cmd_vel.z());
}

void Actuator::setCommandVelocity(const float x, const float y, const float angular)
{
    command_velocity_.x() = std::clamp(x, velocity_min_.x(), velocity_max_.x());
    command_velocity_.y() = std::clamp(y, velocity_min_.y(), velocity_max_.y());
    command_velocity_.z() = std::clamp(angular, velocity_min_.z(), velocity_max_.z());
    zeroUnsupportedAxes(command_velocity_);
}

void Actuator::setCommandAcceleration(const Eigen::Vector3f &cmd_acc)
{
    setCommandAcceleration(cmd_acc.x(), cmd_acc.y(), cmd_acc.z());
}

void Actuator::setCommandAcceleration(const float x, const float y, const float angular)
{
    command_acceleration_.x() = std::clamp(x, -acceleration_max_.x(), acceleration_max_.x());
    command_acceleration_.y() = std::clamp(y, -acceleration_max_.y(), acceleration_max_.y());
    command_acceleration_.z() = std::clamp(angular, -acceleration_max_.z(), acceleration_max_.z());
    zeroUnsupportedAxes(command_acceleration_);
}

void Actuator::zeroUnsupportedAxes(Eigen::Vector3f &vector) const
{
    switch (getType())
    {
    case TYPE::X_ANGULAR:
        vector.y() = 0.0f;
        break;

    case TYPE::X_Y_ANGULAR:
        break;

    case TYPE::X_Y:
        vector.z() = 0.0f;
        break;

    default:
        vector.setZero();
        break;
    }
}

void Actuator::getCommandVelocity(float &x, float &y, float &angular) const
{
    x = command_velocity_.x();
    y = command_velocity_.y();
    angular = command_velocity_.z();
}

Eigen::Vector3f Actuator::getCommandVelocity() const
{
    return command_velocity_;
}

void Actuator::getCommandAcceleration(float &x, float &y, float &angular) const
{
    x = command_acceleration_.x();
    y = command_acceleration_.y();
    angular = command_acceleration_.z();
}

Eigen::Vector3f Actuator::getCommandAcceleration() const
{
    return command_acceleration_;
}

void Actuator::getVelocity(float &x, float &y, float &angular) const
{
    x = velocity_current_.x();
    y = velocity_current_.y();
    angular = velocity_current_.z();
}

Eigen::Vector3f Actuator::getVelocity() const
{
    return velocity_current_;
}

void Actuator::getVelocityMax(float &x, float &y, float &angular) const
{
    x = velocity_max_.x();
    y = velocity_max_.y();
    angular = velocity_max_.z();
}

Eigen::Vector3f Actuator::getVelocityMax() const
{
    return velocity_max_;
}

void Actuator::getVelocityMin(float &x, float &y, float &angular) const
{
    x = velocity_min_.x();
    y = velocity_min_.y();
    angular = velocity_min_.z();
}

Eigen::Vector3f Actuator::getVelocityMin() const
{
    return velocity_min_;
}

void Actuator::getAccelerationMax(float &x, float &y, float &angular) const
{
    x = acceleration_max_.x();
    y = acceleration_max_.y();
    angular = acceleration_max_.z();
}

Eigen::Vector3f Actuator::getAccelerationMax() const
{
    return acceleration_max_;
}

std::ostream& operator<<(std::ostream &os, const Actuator& a)
{
    return a.print(os);
}

std::string Actuator::getTypeAsString(const TYPE type)
{
    switch (type)
    {
    case TYPE::X_ANGULAR:
        return "X_ANGULAR";
    case TYPE::X_Y_ANGULAR:
        return "X_Y_ANGULAR";
    case TYPE::X_Y:
        return "X_Y";
    default:
        return "UNKNOWN";
    }
}

std::string Actuator::getTypeAsString() const
{
    return getTypeAsString(getType());
}

Actuator::TYPE Actuator::getTypeFromString(std::string type)
{
    std::transform(type.begin(), type.end(), type.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if(type == "X_ANGULAR")
        return Actuator::TYPE::X_ANGULAR;
    else if(type == "X_Y_ANGULAR")
        return Actuator::TYPE::X_Y_ANGULAR;
    else if(type == "X_Y")
        return Actuator::TYPE::X_Y;
    else
        throw std::runtime_error("Unknown Actuator type!");
}

Eigen::Affine2f Actuator::step(const float dt)
{
    // A negative or NaN timestep is a caller error; the !(dt >= 0) form rejects
    // both (every comparison with NaN is false). A dt of zero is a valid no-op.
    if (!(dt >= 0.0f))
    {
        throw std::invalid_argument("Actuator::step: dt must be non-negative and not NaN");
    }
    return (dt > 0.0f)? poseDelta(move(dt), dt): Eigen::Affine2f::Identity();
}

Eigen::Affine2f Actuator::poseDelta(const Eigen::Vector3f &velocity, const float dt)
{
    return murosim::geometry::primitives::pose(velocity.x() * dt, velocity.y() * dt, velocity.z() * dt);
}

void Actuator::setVelocityLimits(const size_t axis, const float min, const float max)
{
    const Eigen::Index index = static_cast<Eigen::Index>(axis);
    velocity_min_[index] = std::min(min, max);
    velocity_max_[index] = std::max(min, max);
}

void Actuator::setVelocityLimitsSymmetric(const size_t axis, const float limit)
{
    const float sanitized = std::max(0.0f, limit);
    const Eigen::Index index = static_cast<Eigen::Index>(axis);
    velocity_min_[index] = -sanitized;
    velocity_max_[index] = sanitized;
}

void Actuator::setAccelerationLimit(const size_t axis, const float max)
{
    acceleration_max_[static_cast<Eigen::Index>(axis)] = std::max(0.0f, max);
}

void Actuator::setAccelerationLimit(const size_t axis, const float max, const float cap)
{
    acceleration_max_[static_cast<Eigen::Index>(axis)] = std::clamp(max, 0.0f, cap);
}

Eigen::Vector3f Actuator::velocityFromControlMode(const Eigen::Vector3f &velocity_current, const float dt) const
{
    Eigen::Vector3f velocity = velocity_current;

    switch (control_mode_)
    {
    case CONTROL_MODE::VELOCITY:
    {
        for (int i = 0; i < 3; i++)
        {
            velocity[i] = axisVelocityTowards(velocity[i], command_velocity_[i], acceleration_max_[i], dt);
        }
        break;
    }

    case CONTROL_MODE::ACCELERATION:
    {
        for (int i = 0; i < 3; i++)
        {
            velocity[i] = std::clamp(velocity[i] + command_acceleration_[i] * dt, velocity_min_[i], velocity_max_[i]);
        }
        break;
    }
    }

    return velocity;
}

void Actuator::setControlMode(const CONTROL_MODE mode)
{
    control_mode_ = mode;
}

CONTROL_MODE Actuator::getControlMode() const
{
    return control_mode_;
}

std::string Actuator::getControlModeAsString(const CONTROL_MODE mode)
{
    switch (mode)
    {
    case CONTROL_MODE::VELOCITY:
        return "VELOCITY";
    case CONTROL_MODE::ACCELERATION:
        return "ACCELERATION";
    default:
        return "UNKNOWN";
    }
}

std::string Actuator::getControlModeAsString() const
{
    return getControlModeAsString(control_mode_);
}

CONTROL_MODE Actuator::getControlModeFromString(std::string mode)
{
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if (mode == "VELOCITY")
        return CONTROL_MODE::VELOCITY;
    else if (mode == "ACCELERATION")
        return CONTROL_MODE::ACCELERATION;
    else
        throw std::runtime_error("Unknown Control mode!");
}


} // namespace murosim::actuator
