# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Added by the root CMakeLists.txt when MUROSIM_BUILD_EXAMPLES=ON.
#
# cmake_consumer/ is deliberately not added here: it is a standalone project
# that consumes the *installed* package through find_package(MuRoSim), which
# is the whole point of it. ci/cpp.sh configures it separately against the
# install prefix.

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(MuRoSimHelpers)

# Both examples open a window, so they are built but never run in CI.
add_executable(murosim_example_visualization visualization.cpp)
target_link_libraries(murosim_example_visualization PRIVATE MuRoSim::murosim)
murosim_configure_warnings(murosim_example_visualization)

add_executable(murosim_example_generators generators.cpp)
target_link_libraries(murosim_example_generators PRIVATE MuRoSim::murosimproc)
murosim_configure_warnings(murosim_example_generators)
