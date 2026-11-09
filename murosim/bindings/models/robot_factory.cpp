// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"
#include "murosim/models/robot.hpp"
#include "murosim/math/constants.hpp"

namespace murosimpy {

void initModelsRobotFactory(pybind11::module &m)
{
    m.def("robomaster", [](const float velocity_x_min, const float velocity_x_max, const float velocity_y_max,
                            const float velocity_angular_max, const float acceleration_linear,
                            const float acceleration_angular, const bool holonomic) {
        return murosim::models::robot::robomaster(velocity_x_min, velocity_x_max, std::abs(velocity_y_max),
            std::abs(velocity_angular_max), acceleration_linear, acceleration_angular, holonomic);
    }, "Create a DJI RoboMaster-shaped robot (0.32 x 0.24 m body) with a single 270 deg, "
       "1200-ray lidar (range 0.05-30 m, 10 Hz, normal noise) mounted at its front.\n\n"
       "The sign of velocity_y_max and velocity_angular_max is ignored (their absolute value is used).\n\n"
       "Args:\n"
       "    velocity_x_min: Minimum forward linear velocity, m/s (default: 0.0)\n"
       "    velocity_x_max: Maximum forward linear velocity, m/s (default: 3.0)\n"
       "    velocity_y_max: Maximum lateral linear velocity, m/s, used only if holonomic (default: 3.0)\n"
       "    velocity_angular_max: Maximum angular velocity, rad/s (default: pi)\n"
       "    acceleration_linear: Maximum linear acceleration, m/s^2 (default: pi)\n"
       "    acceleration_angular: Maximum angular acceleration, rad/s^2 (default: pi)\n"
       "    holonomic: If True, use an X_Y_ANGULAR (mecanum) actuator; otherwise X_ANGULAR "
       "(differential-like) (default: True)\n\n"
       "Returns:\n"
       "    Robot instance configured as RoboMaster.",
       pybind11::arg("velocity_x_min") = 0.0f,
       pybind11::arg("velocity_x_max") = 3.0f,
       pybind11::arg("velocity_y_max") = 3.0f,
       pybind11::arg("velocity_angular_max") = murosim::math::PI,
       pybind11::arg("acceleration_linear") = murosim::math::PI,
       pybind11::arg("acceleration_angular") = murosim::math::PI,
       pybind11::arg("holonomic") = true);

    m.def("mir100", [](const float velocity_x_min, const float velocity_x_max, const float velocity_angular_max) {
        return murosim::models::robot::mir100(velocity_x_min, velocity_x_max, velocity_angular_max);
    }, "Create a MiR100-shaped differential-drive robot (0.9 x 0.6 m body) with two SICK S300-like 270 deg, "
       "541-ray lidars (range 0.05-29 m, 12.5 Hz, normal noise of 9 mm at 1 sigma) on the front-left and "
       "rear-right corners.\n\n"
       "Args:\n"
       "    velocity_x_min: Minimum forward linear velocity, m/s, clamped to [-0.3, 1.5] (default: -0.3)\n"
       "    velocity_x_max: Maximum forward linear velocity, m/s, clamped to [-1.5, 1.5] (default: 1.5)\n"
       "    velocity_angular_max: Maximum angular velocity, rad/s; its absolute value is clamped to "
       "[0, pi/2] (default: pi/2)\n\n"
       "Returns:\n"
       "    Robot instance configured as MiR100.",
       pybind11::arg("velocity_x_min") = -0.3f,
       pybind11::arg("velocity_x_max") = 1.5f,
       pybind11::arg("velocity_angular_max") = murosim::math::PI_2);
}

} // namespace murosimpy
