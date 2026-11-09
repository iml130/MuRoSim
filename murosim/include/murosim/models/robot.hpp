// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_MODELS_ROBOT_HPP_
#define MUROSIM_MODELS_ROBOT_HPP_

#include "murosim/robot/robot.hpp"
#include "murosim/math/constants.hpp"

/// Factory functions building preconfigured Robot instances with realistic
/// shapes, lidar placement and actuator limits for known physical platforms.
namespace murosim::models::robot
{

/// Builds a DJI RoboMaster-shaped robot (0.32 x 0.24 m body) with a single
/// 270 deg, 1200-ray lidar (range 0.05-30 m, 10 Hz, normal noise) mounted at
/// its front.
/// @param velocity_x_min Minimum forward linear velocity (m/s).
/// @param velocity_x_max Maximum forward linear velocity (m/s).
/// @param velocity_y_max Maximum lateral linear velocity (m/s), used only if holonomic.
/// @param velocity_angular_max Maximum angular velocity (rad/s).
/// @param acceleration_linear Maximum linear acceleration (m/s^2).
/// @param acceleration_angular Maximum angular acceleration (rad/s^2).
/// @param holonomic If true, use an X_Y_ANGULAR (mecanum) actuator; otherwise X_ANGULAR (differential-like).
std::shared_ptr<murosim::robot::Robot> robomaster(const float velocity_x_min, const float velocity_x_max, const float velocity_y_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular, const bool holonomic=true);
/// Builds a MiR100-shaped differential-drive robot (0.9 x 0.6 m body) with two
/// SICK S300-like 270 deg, 541-ray lidars (range 0.05-29 m, 12.5 Hz, normal
/// noise of 9 mm at 1 sigma) on the front-left and rear-right corners.
/// @param velocity_x_min Minimum forward linear velocity (m/s), clamped to [-0.3, 1.5]
/// (the MiR100 reverses at up to 0.3 m/s).
/// @param velocity_x_max Maximum forward linear velocity (m/s), clamped to [-1.5, 1.5].
/// @param velocity_angular_max Maximum angular velocity (rad/s), clamped to [0, pi/2].
std::shared_ptr<murosim::robot::Robot> mir100(const float velocity_x_min=-0.3f, const float velocity_x_max=1.5f, const float velocity_angular_max=murosim::math::PI_2);

} // namespace murosim::models::robot

#endif // MUROSIM_MODELS_ROBOT_HPP_
