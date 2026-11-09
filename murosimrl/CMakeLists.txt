# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(MuRoSimHelpers)

project(murosimrl
    VERSION ${MUROSIM_VERSION}
    DESCRIPTION "Multi-robot simulator reinforcement learning library"
    LANGUAGES CXX
)

add_library(murosimrl STATIC
    # Core
    src/action.cpp
    src/done.cpp
    # Observation base
    src/observation/observation_base.cpp
    src/observation/observation_robot.cpp
    src/observation/observation_set.cpp
    src/observation/observation_robot_simulation.cpp
    src/observation/observation_simulation.cpp
    # Observation robot
    src/observation/robot/command_velocity.cpp
    src/observation/robot/delta_pose.cpp
    src/observation/robot/distance_to_goal.cpp
    src/observation/robot/goal.cpp
    src/observation/robot/lidar.cpp
    src/observation/robot/lidar_point.cpp
    src/observation/robot/orientation_to_goal.cpp
    src/observation/robot/pose.cpp
    src/observation/robot/subgoal_direction.cpp
    src/observation/robot/velocity.cpp
    src/observation/robot/velocity_base.cpp
    # Observation robot simulation
    src/observation/robot_simulation/goal_map.cpp
    src/observation/robot_simulation/goal_normalized.cpp
    src/observation/robot_simulation/pose_map.cpp
    src/observation/robot_simulation/pose_normalized.cpp
    # Observation simulation
    src/observation/simulation/occupancy_map.cpp
    src/observation/simulation/path_map.cpp
    src/observation/simulation/velocity_map.cpp
    # Reward
    src/reward/reward_robot.cpp
    src/reward/reward_set.cpp
    src/reward/reward_simulation.cpp
    # Reward robot
    src/reward/robot/action_rate.cpp
    src/reward/robot/delta_distance_along_path.cpp
    src/reward/robot/delta_distance_to_goal.cpp
    src/reward/robot/done.cpp
    src/reward/robot/forward_driving.cpp
    src/reward/robot/minimum_lidar_distance.cpp
    src/reward/robot/on_goal.cpp
    src/reward/robot/orientation_to_goal.cpp
    # Util
    src/util/conversion.cpp
    src/util/grid.cpp
    src/util/path.cpp
    src/util/robot.cpp
    src/util/simulation.cpp
)
murosim_configure_library(murosimrl)

target_link_libraries(murosimrl PUBLIC MuRoSim::murosim)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(MUROSIM_BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings)
endif()
