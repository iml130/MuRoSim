# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Hardware limits of the DJI RoboMaster base used by models.robot.robomaster().

VELOCITY_* are the defaults of the actuator. The wheels cannot exceed
WHEEL_RPM_MAX and need TIME_TO_MAX_RPM seconds to get there from standstill,
independent of the acceleration limits passed to robomaster(); motion is
integrated in sub-steps of SAMPLE_TIME seconds regardless of the simulation
time step.
"""
from __future__ import annotations
__all__: list[str] = ['SAMPLE_TIME', 'TIME_TO_MAX_RPM', 'VELOCITY_ANGULAR_MAX', 'VELOCITY_LINEAR_MAX', 'WHEEL_RPM_MAX']
SAMPLE_TIME: float = 0.019999999552965164
TIME_TO_MAX_RPM: float = 1.6666666269302368
VELOCITY_ANGULAR_MAX: float = 2.0
VELOCITY_LINEAR_MAX: float = 3.0
WHEEL_RPM_MAX: float = 1000.0
