// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/actuator/robomaster.hpp"
#include <numbers>

#include <algorithm>
#include <cmath>

namespace
{
constexpr double WHEEL_SEPARATION_WIDTH = 0.10;
constexpr double WHEEL_SEPARATION_LENGTH = 0.10;
constexpr double WHEEL_RADIUS = 0.05;
constexpr double WHEEL_RADIUS_INVERSE = 1.0 / WHEEL_RADIUS;
// the public limits are rpm and seconds; the kinematics work in rad/s
constexpr double WHEEL_RAD_PER_S_MAX = static_cast<double>(murosim::actuator::RoboMaster::WHEEL_RPM_MAX) / 60.0 * 2.0 * std::numbers::pi;
constexpr double WHEEL_RAD_PER_S_DELTA_MAX = WHEEL_RAD_PER_S_MAX / static_cast<double>(murosim::actuator::RoboMaster::TIME_TO_MAX_RPM);
} // namespace

namespace murosim::actuator
{

RoboMaster::RoboMaster():
    type_(Actuator::TYPE::X_Y_ANGULAR)
{
    velocity_min_.x() = -VELOCITY_LINEAR_MAX;
    velocity_max_.x() =  VELOCITY_LINEAR_MAX;
    velocity_min_.y() = -VELOCITY_LINEAR_MAX;
    velocity_max_.y() =  VELOCITY_LINEAR_MAX;
    velocity_min_.z() = -VELOCITY_ANGULAR_MAX;
    velocity_max_.z() =  VELOCITY_ANGULAR_MAX;

    correctVelocityLimits();
    reset();
}

RoboMaster::RoboMaster(const float velocity_linear_max, const float velocity_angular_max, const Actuator::TYPE type):
    type_(type)
{
    setVelocityLimitsSymmetric(0, velocity_linear_max);
    setVelocityLimitsSymmetric(1, velocity_linear_max);
    setVelocityLimitsSymmetric(2, velocity_angular_max);

    correctVelocityLimits();
    reset();
}

RoboMaster::RoboMaster(const float velocity_linear_x_min, const float velocity_linear_x_max, const float velocity_linear_y_max, const float velocity_angular_max, const Actuator::TYPE type):
    type_(type)
{
    setVelocityLimits(0, velocity_linear_x_min, velocity_linear_x_max);
    setVelocityLimitsSymmetric(1, velocity_linear_y_max);
    setVelocityLimitsSymmetric(2, velocity_angular_max);

    correctVelocityLimits();
    reset();
}

RoboMaster::RoboMaster(const float velocity_linear_x_min, const float velocity_linear_x_max, const float velocity_linear_y_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular, const Actuator::TYPE type):
    type_(type)
{
    setVelocityLimits(0, velocity_linear_x_min, velocity_linear_x_max);
    setVelocityLimitsSymmetric(1, velocity_linear_y_max);
    setVelocityLimitsSymmetric(2, velocity_angular_max);

    setAccelerationLimit(0, acceleration_linear);
    setAccelerationLimit(1, acceleration_linear);
    setAccelerationLimit(2, acceleration_angular);

    correctVelocityLimits();
    reset();
}

void RoboMaster::forward(const Eigen::Vector3f& velocity, double &w1, double &w2, double &w3, double &w4)
{
    const Eigen::Vector3d v = velocity.cast<double>();
    w1 = WHEEL_RADIUS_INVERSE * (v.x() - v.y() - (WHEEL_SEPARATION_LENGTH + WHEEL_SEPARATION_WIDTH) * v.z());
    w2 = WHEEL_RADIUS_INVERSE * (v.x() + v.y() + (WHEEL_SEPARATION_LENGTH + WHEEL_SEPARATION_WIDTH) * v.z());
    w3 = WHEEL_RADIUS_INVERSE * (v.x() + v.y() - (WHEEL_SEPARATION_LENGTH + WHEEL_SEPARATION_WIDTH) * v.z());
    w4 = WHEEL_RADIUS_INVERSE * (v.x() - v.y() + (WHEEL_SEPARATION_LENGTH + WHEEL_SEPARATION_WIDTH) * v.z());
}

void RoboMaster::inverse(const double w1, const double w2, const double w3, const double w4, Eigen::Vector3f &velocity)
{
    velocity.x() = static_cast<float>((  w1 + w2 + w3 + w4) * WHEEL_RADIUS * 0.25);
    velocity.y() = static_cast<float>((- w1 + w2 + w3 - w4) * WHEEL_RADIUS * 0.25);
    velocity.z() = static_cast<float>((- w1 + w2 - w3 + w4) * WHEEL_RADIUS * 0.25 / (WHEEL_SEPARATION_LENGTH + WHEEL_SEPARATION_WIDTH));
}

void RoboMaster::correctVelocityLimits()
{
    zeroUnsupportedAxes(velocity_min_);
    zeroUnsupportedAxes(velocity_max_);
}

Eigen::Vector3f RoboMaster::update(const float dt)
{
    const double delta_limit = WHEEL_RAD_PER_S_DELTA_MAX * static_cast<double>(dt);
    double w1, w2, w3, w4;
    Eigen::Vector3f vel = velocity();
    // initialized in every CONTROL_MODE case; the fallback (keep the current
    // velocity) also silences GCC's maybe-uninitialized false positive
    Eigen::Vector3f target_vel = vel;

    switch (control_mode_)
    {
    case CONTROL_MODE::VELOCITY:
    {
        // Accelerate toward target velocity using acceleration_max_
        target_vel = clipVelocityTarget(command_velocity_, vel, dt, std::min(acceleration_max_.x(), acceleration_max_.y()), acceleration_max_.z());
        break;
    }

    case CONTROL_MODE::ACCELERATION:
    {
        // Integrate commanded acceleration, clamped to the velocity limits
        target_vel = velocityFromControlMode(vel, dt);
        break;
    }
    }

    forward(target_vel, w1, w2, w3, w4);

    // Second, independent acceleration bound: a rough approximation of the
    // physical wheel slew rate (see WHEEL_RAD_PER_S_DELTA_MAX). The
    // controller-level cap above and this hardware limit are intentionally
    // separate; the realized acceleration is whichever is more restrictive.
    w1_ += std::clamp(w1 - w1_, -delta_limit, delta_limit);
    w2_ += std::clamp(w2 - w2_, -delta_limit, delta_limit);
    w3_ += std::clamp(w3 - w3_, -delta_limit, delta_limit);
    w4_ += std::clamp(w4 - w4_, -delta_limit, delta_limit);

    w1_ = std::clamp(w1_, -WHEEL_RAD_PER_S_MAX, WHEEL_RAD_PER_S_MAX);
    w2_ = std::clamp(w2_, -WHEEL_RAD_PER_S_MAX, WHEEL_RAD_PER_S_MAX);
    w3_ = std::clamp(w3_, -WHEEL_RAD_PER_S_MAX, WHEEL_RAD_PER_S_MAX);
    w4_ = std::clamp(w4_, -WHEEL_RAD_PER_S_MAX, WHEEL_RAD_PER_S_MAX);

    inverse(w1_, w2_, w3_, w4_, vel);

    velocity_current_ = vel;

    return vel;
}

Eigen::Vector3f RoboMaster::velocity() const
{
    Eigen::Vector3f velocity;
    inverse(w1_, w2_, w3_, w4_, velocity);
    return velocity;
}

Eigen::Vector3f RoboMaster::clipVelocityTarget(const Eigen::Vector3f &velocity_target, const Eigen::Vector3f &velocity_current, const float dt, const float acceleration_linear, const float acceleration_angular)
{
    Eigen::Vector3f delta = velocity_target - velocity_current;

    if (0.0f < acceleration_linear)
    {
        // clip the linear delta isotropically so the direction is preserved
        const float delta_linear = delta.head(2).norm();

        if (0.0f < delta_linear)
        {
            const float delta_linear_clipped = std::clamp(delta_linear, -acceleration_linear * dt, acceleration_linear * dt);
            const float ratio = delta_linear_clipped / delta_linear;
            // component-wise instead of delta.head(2) *= ratio: the block op
            // makes GCC's vectorizer load 16 bytes from the 12-byte Vector3f
            // and emit a -Warray-bounds false positive
            delta.x() *= ratio;
            delta.y() *= ratio;
        }
    }

    if (0.0f < acceleration_angular)
    {
        delta.z() = std::clamp(delta.z(), -acceleration_angular * dt, acceleration_angular * dt);
    }

    return velocity_current + delta;
}

std::unique_ptr<Actuator> RoboMaster::clone() const
{
    return std::make_unique<RoboMaster>(*this);
}

Eigen::Vector3f RoboMaster::move(const float dt)
{
    // Integrate at the measured 50 Hz control rate: the wheel-slew limiter ramps
    // the velocity each sub-step, so the realized motion is the composition of
    // the per-sub-step pose deltas.
    Eigen::Affine2f movement = Eigen::Affine2f::Identity();

    float t = dt;

    while(SAMPLE_TIME < t)
    {
        movement = movement * poseDelta(update(SAMPLE_TIME), SAMPLE_TIME);
        t -= SAMPLE_TIME;
    }

    movement = movement * poseDelta(update(t), t);

    // Reduce the composed sub-step motion to a single equivalent body velocity so
    // step() rebuilds the identical pose delta (poseDelta spans SE(2), so this is
    // exact, not an approximation). dt > 0 is guaranteed by step().
    const float angle = Eigen::Rotation2Df(movement.linear()).angle();
    const Eigen::Vector2f translation = movement.translation();
    return Eigen::Vector3f(translation.x() / dt, translation.y() / dt, angle / dt);
}

void RoboMaster::reset()
{
    w1_ = 0.0;
    w2_ = 0.0;
    w3_ = 0.0;
    w4_ = 0.0;

    command_velocity_ = Eigen::Vector3f::Zero();
    command_acceleration_ = Eigen::Vector3f::Zero();
    velocity_current_ = Eigen::Vector3f::Zero();
}

Actuator::TYPE RoboMaster::getType() const
{
    return type_;
}

std::ostream& RoboMaster::print(std::ostream& out) const
{
    out << "RoboMaster {"
        << " type: " << getTypeAsString()
        << ", vel_x: [" << velocity_min_.x() << ", " << velocity_max_.x() << "]"
        << ", vel_y: [" << velocity_min_.y() << ", " << velocity_max_.y() << "]"
        << ", vel_angular: [" << velocity_min_.z() << ", " << velocity_max_.z() << "]"
        << ", acc_x: " << acceleration_max_.x()
        << ", acc_y: " << acceleration_max_.y()
        << ", acc_angular: " << acceleration_max_.z()
        << ", wheels: [" << w1_ << ", " << w2_ << ", " << w3_ << ", " << w4_ << "]"
        << " }";
    return out;
}

} // murosim::actuator
