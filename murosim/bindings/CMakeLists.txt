# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Python bindings for murosim (module: murosimpy)

murosim_add_python_module(murosimpy
    MACRO_PREFIX MUROSIMPY
    SOURCES
        # Main entry point
        murosimpy.cpp
        definitions.cpp

        # Core module
        core/robot.cpp
        core/robot_builder.cpp
        core/simulation.cpp
        core/thread_pool.cpp

        # Geometry module
        geometry/transform.cpp
        geometry/line.cpp
        geometry/circle.cpp
        geometry/shape.cpp
        geometry/world.cpp
        geometry/primitives.cpp

        # Navigation module
        navigation/node.cpp
        navigation/node_container.cpp

        # Models module
        models/robot_factory.cpp
        models/world_factory.cpp

        # Containers module
        containers/simulation_container.cpp
        containers/robot_container.cpp

        # Rendering module
        rendering/offline_renderer.cpp
        rendering/online_renderer.cpp

        # IO module
        io/data_container.cpp
        io/functions.cpp
    LINK MuRoSim::murosim
)
