# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(MuRoSimHelpers)

project(murosim
    VERSION ${MUROSIM_VERSION}
    DESCRIPTION "Multi-robot simulator"
    LANGUAGES CXX
)

find_package(Eigen3 REQUIRED CONFIG)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EGL REQUIRED IMPORTED_TARGET egl)
pkg_check_modules(GLESv2 REQUIRED IMPORTED_TARGET glesv2)
pkg_check_modules(GLFW3 REQUIRED IMPORTED_TARGET glfw3)

add_library(murosim STATIC
    # Actuator
    src/actuator/actuator.cpp
    src/actuator/differential_kobuki.cpp
    src/actuator/differential_simple.cpp
    src/actuator/holonomic_simple.cpp
    src/actuator/robomaster.cpp
    # Collision
    src/collision/array_kernels.cpp
    src/collision/circle.cpp
    src/collision/lidar.cpp
    src/collision/line.cpp
    src/collision/shape.cpp
    # Grid
    src/grid/bresenham.cpp
    src/grid/drawing.cpp
    src/grid/map.cpp
    # IO
    src/io/util.cpp
    src/io/world.cpp
    # Model
    src/models/robot.cpp
    src/models/worlds.cpp
    # Node
    src/navigation/container.cpp
    src/navigation/node.cpp
    # Planning
    src/planning/astar.cpp
    src/planning/planner.cpp
    src/planning/util.cpp
    src/planning/voronoi.cpp
    # Robot
    src/robot/robot.cpp
    src/robot/robot_builder.cpp
    src/robot/state_buffer.cpp
    src/robot/state.cpp
    src/robot/util.cpp
    # Sensor
    src/sensor/lidar.cpp
    # Shape
    src/geometry/shape.cpp
    src/geometry/world.cpp
    # SIMD
    src/simd/array_float.cpp
    src/simd/util.cpp
    # Simulation
    src/simulation/initialisation.cpp
    src/simulation/simulation.cpp
    # Threading
    src/threading/batch.cpp
    src/threading/thread_pool.cpp
    # Visualization
    src/visualization/canvas.cpp
    src/visualization/renderer.cpp
    src/visualization/renderer_egl.cpp
    src/visualization/renderer_glfw.cpp
    src/visualization/scenery.cpp
    src/visualization/shader/gl_shader_manager.cpp
    src/visualization/shader/shader.cpp
    src/visualization/shader/shader_circle.cpp
    src/visualization/shader/shader_line.cpp
    src/visualization/shader/shader_path.cpp
    src/visualization/shader/shader_quad.cpp
    src/visualization/shader/shader_ring.cpp
    src/visualization/shader/util.cpp
)
murosim_configure_library(murosim)
# GCC reports -Wnull-dereference inside Eigen 5's BinaryFunctors.h at -O3
# (false positive, surfaced at our instantiation site in scenery.cpp; Eigen 3.4
# does not trigger it). Silence it there only, so -Werror works with both.
set_source_files_properties(src/visualization/scenery.cpp
    PROPERTIES COMPILE_OPTIONS $<$<CXX_COMPILER_ID:GNU>:-Wno-null-dereference>
)

target_link_libraries(murosim
    PUBLIC
        Eigen3::Eigen
        Threads::Threads
        # PUBLIC: the installed visualization headers include EGL, GLES and GLFW
        PkgConfig::EGL
        PkgConfig::GLESv2
        PkgConfig::GLFW3
)

if(BUILD_TESTING)
    add_subdirectory(tests)

    # murosim_benchmark [robots] [steps]
    add_executable(murosim_benchmark benchmarks/benchmark.cpp)
    target_link_libraries(murosim_benchmark PRIVATE MuRoSim::murosim)
    murosim_configure_warnings(murosim_benchmark)
endif()

if(MUROSIM_BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings)
endif()
