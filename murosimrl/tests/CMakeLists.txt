# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

find_package(GTest REQUIRED)

add_executable(murosimrltest
    main.cpp
    action_test.cpp
    done_test.cpp
    observation/observation_set_test.cpp
    observation/robot/command_velocity_test.cpp
    observation/robot/delta_pose_test.cpp
    observation/robot/distance_to_goal_test.cpp
    observation/robot/goal_test.cpp
    observation/robot/lidar_point_test.cpp
    observation/robot/lidar_test.cpp
    observation/robot/orientation_to_goal_test.cpp
    observation/robot/pose_test.cpp
    observation/robot/subgoal_direction_test.cpp
    observation/robot/velocity_test.cpp
    observation/robot_simulation/goal_map_test.cpp
    observation/robot_simulation/goal_normalized_test.cpp
    observation/robot_simulation/pose_map_test.cpp
    observation/robot_simulation/pose_normalized_test.cpp
    observation/simulation/occupancy_map_test.cpp
    observation/simulation/path_map_test.cpp
    observation/simulation/velocity_map_test.cpp
    reward/reward_set_test.cpp
    reward/robot/action_rate_test.cpp
    reward/robot/delta_distance_along_path_test.cpp
    reward/robot/delta_distance_to_goal_test.cpp
    reward/robot/done_test.cpp
    reward/robot/forward_driving_test.cpp
    reward/robot/minimum_lidar_distance_test.cpp
    reward/robot/on_goal_test.cpp
    reward/robot/orientation_to_goal_test.cpp
    util/conversion_test.cpp
    util/grid_test.cpp
    util/path_test.cpp
    util/robot_test.cpp
    util/simulation_test.cpp
)
target_link_libraries(murosimrltest PRIVATE MuRoSim::murosimrl GTest::gtest)
murosim_configure_warnings(murosimrltest)
target_include_directories(murosimrltest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

include(GoogleTest)
gtest_discover_tests(murosimrltest)
