# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Minimal external project using the installed MuRoSim C++ libraries:
#
#   sudo cmake --install <murosim build dir> --prefix /opt/murosim
#   cmake -S examples/cmake_consumer -B build-consumer -DCMAKE_PREFIX_PATH=/opt/murosim
#   cmake --build build-consumer && ./build-consumer/consumer

cmake_minimum_required(VERSION 3.24)
project(murosim_consumer LANGUAGES CXX)

find_package(MuRoSim 1.0 CONFIG REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE
    MuRoSim::murosim
    MuRoSim::murosimrl
    MuRoSim::murosimproc
)
