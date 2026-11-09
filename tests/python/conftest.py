# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
"""Shared fixtures for the Python smoke tests.

The suite checks the pybind11 boundary (imports, conversions, lifetimes,
exception mapping, one end-to-end path per module), not the simulator
logic, which the C++ suites cover.
"""

import math

import pytest

import murosimpy

WORLD_SIZE = 10.0  # models.world.empty spans [0, WORLD_SIZE] on both axes
LIDAR_RAYS = 16
LIDAR_RANGE_MAX = 10.0
VELOCITY_MAX = 1.0
BODY_SIZE = 0.3
LIDAR_RANGE_MIN = 0.05
GOAL_RADIUS = 0.3  # a goal is [x, y, radius]; a zero radius can never be reached
ROBOT_POSES = [[2.0, 2.0, 0.0], [8.0, 8.0, math.pi / 2.0]]  # apart, so reset does not collide
ROBOT_GOALS = [[8.0, 8.0, GOAL_RADIUS], [2.0, 2.0, GOAL_RADIUS]]


def build_robot(name, with_lidar=True):
    """Holonomic robot with a square body and one full-circle lidar."""
    builder = murosimpy.core.RobotBuilder()
    builder.set_name(name)
    builder.set_actuator_holonomic_simple(-VELOCITY_MAX, VELOCITY_MAX, VELOCITY_MAX, VELOCITY_MAX)
    builder.set_shape_robot_rectangle(BODY_SIZE, BODY_SIZE)
    if with_lidar:
        builder.add_lidar(murosimpy.geometry.Transform(), 2.0 * math.pi, LIDAR_RAYS, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX, 10.0)
    return builder.build()


def build_simulation(data):
    """Simulation with two robots on opposite corners, reset and ready to step."""
    simulation = murosimpy.core.Simulation(data)
    for index, (pose, goal) in enumerate(zip(ROBOT_POSES, ROBOT_GOALS)):
        robot = build_robot(f"robot_{index}")
        robot.pose = pose
        robot.goal = goal
        simulation.add_robot(robot)
    simulation.reset()
    return simulation


@pytest.fixture
def world_data():
    return murosimpy.models.world.empty(WORLD_SIZE, 1.0)


@pytest.fixture
def simulation(world_data):
    return build_simulation(world_data)


@pytest.fixture
def robots(simulation):
    return simulation.robots


@pytest.fixture
def thread_pool():
    return murosimpy.core.ThreadPool(2)
