# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

find_package(GTest REQUIRED)

add_executable(murosimtest
    main.cpp
    actuator/differential_kobuki_test.cpp
    actuator/differential_simple_test.cpp
    actuator/holonomic_simple_test.cpp
    actuator/robomaster_test.cpp
    collision/circle_test.cpp
    collision/laser_test.cpp
    collision/lidar_sweep_test.cpp
    collision/line_test.cpp
    collision/shape_test.cpp
    grid/bresenham_test.cpp
    grid/drawing_test.cpp
    grid/map_test.cpp
    io/util_test.cpp
    io/world_test.cpp
    math/angle_test.cpp
    models/robot_test.cpp
    models/world_test.cpp
    navigation/node_test.cpp
    navigation/container_test.cpp
    planning/planner_test.cpp
    random/distribution_test.cpp
    random/random_test.cpp
    robot/robot_builder_test.cpp
    robot/robot_test.cpp
    robot/state_test.cpp
    robot/state_buffer_test.cpp
    robot/util_test.cpp
    sensor/lidar_test.cpp
    simulation/initialisation_test.cpp
    simulation/simulation_test.cpp
    geometry/circle_test.cpp
    geometry/primitives_test.cpp
    geometry/line_test.cpp
    geometry/shape_test.cpp
    geometry/world_test.cpp
    simd/array_float_test.cpp
    threading/batch_test.cpp
    threading/thread_pool_test.cpp
    visualization/color_test.cpp
    visualization/renderer_egl_test.cpp
)
target_link_libraries(murosimtest PRIVATE MuRoSim::murosim GTest::gtest)
murosim_configure_warnings(murosimtest)

include(GoogleTest)
gtest_discover_tests(murosimtest)
