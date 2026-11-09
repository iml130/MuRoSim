# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Factory functions for creating predefined robot models.
"""
from __future__ import annotations
import murosimpy.core
import typing
__all__: list[str] = ['mir100', 'robomaster']
def mir100(velocity_x_min: typing.SupportsFloat | typing.SupportsIndex = -0.30000001192092896, velocity_x_max: typing.SupportsFloat | typing.SupportsIndex = 1.5, velocity_angular_max: typing.SupportsFloat | typing.SupportsIndex = 1.5707963705062866) -> murosimpy.core.Robot:
    """
    Create a MiR100-shaped differential-drive robot (0.9 x 0.6 m body) with two SICK S300-like 270 deg, 541-ray lidars (range 0.05-29 m, 12.5 Hz, normal noise of 9 mm at 1 sigma) on the front-left and rear-right corners.

    Args:
        velocity_x_min: Minimum forward linear velocity, m/s, clamped to [-0.3, 1.5] (default: -0.3)
        velocity_x_max: Maximum forward linear velocity, m/s, clamped to [-1.5, 1.5] (default: 1.5)
        velocity_angular_max: Maximum angular velocity, rad/s; its absolute value is clamped to [0, pi/2] (default: pi/2)

    Returns:
        Robot instance configured as MiR100.
    """
def robomaster(velocity_x_min: typing.SupportsFloat | typing.SupportsIndex = 0.0, velocity_x_max: typing.SupportsFloat | typing.SupportsIndex = 3.0, velocity_y_max: typing.SupportsFloat | typing.SupportsIndex = 3.0, velocity_angular_max: typing.SupportsFloat | typing.SupportsIndex = 3.1415927410125732, acceleration_linear: typing.SupportsFloat | typing.SupportsIndex = 3.1415927410125732, acceleration_angular: typing.SupportsFloat | typing.SupportsIndex = 3.1415927410125732, holonomic: bool = True) -> murosimpy.core.Robot:
    """
    Create a DJI RoboMaster-shaped robot (0.32 x 0.24 m body) with a single 270 deg, 1200-ray lidar (range 0.05-30 m, 10 Hz, normal noise) mounted at its front.

    The sign of velocity_y_max and velocity_angular_max is ignored (their absolute value is used).

    Args:
        velocity_x_min: Minimum forward linear velocity, m/s (default: 0.0)
        velocity_x_max: Maximum forward linear velocity, m/s (default: 3.0)
        velocity_y_max: Maximum lateral linear velocity, m/s, used only if holonomic (default: 3.0)
        velocity_angular_max: Maximum angular velocity, rad/s (default: pi)
        acceleration_linear: Maximum linear acceleration, m/s^2 (default: pi)
        acceleration_angular: Maximum angular acceleration, rad/s^2 (default: pi)
        holonomic: If True, use an X_Y_ANGULAR (mecanum) actuator; otherwise X_ANGULAR (differential-like) (default: True)

    Returns:
        Robot instance configured as RoboMaster.
    """
