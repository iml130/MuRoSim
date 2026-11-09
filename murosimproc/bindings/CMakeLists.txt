# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Python bindings for murosimproc (module: murosimprocpy)

murosim_add_python_module(murosimprocpy
    MACRO_PREFIX MUROSIMPROCPY
    SOURCES
        # Main entry point
        murosimprocpy.cpp

        generator.cpp
        factory.cpp
    LINK MuRoSim::murosimproc
)

# Repo root, for murosim/bindings/common.hpp (shared opaque-type declarations)
target_include_directories(murosimprocpy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)
