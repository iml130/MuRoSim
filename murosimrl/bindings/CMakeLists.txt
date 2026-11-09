# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Python bindings for murosimrl (module: murosimrlpy)

murosim_add_python_module(murosimrlpy
    MACRO_PREFIX MUROSIMRLPY
    SOURCES
        # Main entry point
        murosimrlpy.cpp

        # Core module
        core/action.cpp
        core/done.cpp

        # Observation module
        observation/robot.cpp
        observation/robot_simulation.cpp
        observation/simulation.cpp
        observation/set.cpp

        # Reward module
        reward/robot.cpp
        reward/simulation.cpp
        reward/set.cpp
    LINK MuRoSim::murosimrl
)
