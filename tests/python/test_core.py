# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
"""murosimpy: geometry, robots, simulations, containers, io."""

import copy
import gc
import math

import numpy as np
import pytest

import murosimpy
from conftest import (GOAL_RADIUS, LIDAR_RANGE_MAX, LIDAR_RANGE_MIN, LIDAR_RAYS, ROBOT_POSES, VELOCITY_MAX,
                      WORLD_SIZE, build_robot, build_simulation)

STEPS = 10
TIME_STEP = 0.1
MAP_SIZE = 101  # WORLD_SIZE / grid scale 0.1, plus one


# ---------------------------------------------------------------- geometry

def test_geometry_objects_round_trip_their_values():
    line = murosimpy.geometry.Line(1.0, 2.0, 3.0, 4.0)
    circle = murosimpy.geometry.Circle(5.0, 6.0, 0.5)
    assert (line.x1, line.y1, line.x2, line.y2) == (1.0, 2.0, 3.0, 4.0)
    assert (circle.x, circle.y, circle.radius) == (5.0, 6.0, 0.5)


def test_transform_applies_to_geometry():
    transform = murosimpy.geometry.Transform(1.0, 0.0, math.pi / 2.0)
    moved = transform * murosimpy.geometry.Line(0.0, 0.0, 1.0, 0.0)
    assert moved.x1 == pytest.approx(1.0)
    assert moved.y1 == pytest.approx(0.0)
    assert moved.x2 == pytest.approx(1.0, abs=1e-6)
    assert moved.y2 == pytest.approx(1.0)


def test_shape_and_world_from_primitives():
    shape = murosimpy.geometry.primitives.square(4.0)
    assert shape.line_count == 4
    assert shape.circle_count == 0
    shape.add_circle(0.0, 0.0, 0.5)
    world = murosimpy.geometry.World(shape)
    assert world.aabb_x == pytest.approx((-2.0, 2.0))
    assert world.aabb_y == pytest.approx((-2.0, 2.0))
    assert world.circle_at(0).radius == 0.5


# ------------------------------------------------------------------ robots

def test_builder_configures_the_robot():
    robot = build_robot("r")
    assert robot.name == "r"
    assert robot.lidar_count == 1
    assert robot.actuator_type == murosimpy.definitions.ActuatorType.X_Y_ANGULAR
    np.testing.assert_array_equal(robot.velocity_max, [VELOCITY_MAX] * 3)
    assert robot.velocity_max.dtype == np.float32


def test_robot_vector_properties_are_float32_arrays():
    robot = build_robot("r")
    robot.pose = [1.0, 2.0, 0.5]
    robot.goal = (3.0, 4.0, 0.0)
    robot.command_velocity = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    assert robot.pose.dtype == np.float32 and robot.pose.shape == (3,)
    np.testing.assert_allclose(robot.pose, [1.0, 2.0, 0.5])
    np.testing.assert_allclose(robot.goal, [3.0, 4.0, 0.0])
    np.testing.assert_allclose(robot.command_velocity, [0.1, 0.2, 0.3], rtol=1e-6)
    np.testing.assert_allclose(robot.position, [1.0, 2.0])
    assert robot.rotation == pytest.approx(0.5)


def test_wrong_vector_size_is_rejected():
    robot = build_robot("r")
    with pytest.raises((TypeError, RuntimeError, ValueError)):
        robot.pose = [1.0, 2.0]


def test_model_robots_build():
    assert murosimpy.models.robot.robomaster().lidar_count == 1
    assert murosimpy.models.robot.mir100().lidar_count == 2


# ------------------------------------------------------------- simulation

def test_simulation_from_data_container(world_data):
    simulation = murosimpy.core.Simulation(world_data)
    assert simulation.name == world_data.name
    assert simulation.max_robots == world_data.node_count
    assert len(simulation) == 0
    assert simulation.world.aabb_x == pytest.approx((0.0, WORLD_SIZE))
    assert simulation.nodes.size == world_data.node_count
    assert simulation.config.init_mode == murosimpy.definitions.InitMode.MANUAL


def test_reset_places_robots_on_their_poses(simulation):
    assert len(simulation) == 2
    for index, pose in enumerate(ROBOT_POSES):
        np.testing.assert_allclose(simulation[index].pose, pose)
        assert simulation[index].steps == 0
    assert simulation.steps == 0
    assert not simulation.finished


def test_commanded_robot_moves(simulation):
    robot = simulation[0]
    robot.command_velocity = [VELOCITY_MAX / 2.0, 0.0, 0.0]
    for _ in range(STEPS):
        simulation.step()
    expected_x = ROBOT_POSES[0][0] + STEPS * TIME_STEP * VELOCITY_MAX / 2.0
    assert robot.pose[0] == pytest.approx(expected_x, abs=1e-4)
    assert robot.steps == STEPS
    assert simulation.steps == STEPS
    assert robot.collision_status == murosimpy.definitions.CollisionStatus.NONE
    assert robot.traveled_distance == pytest.approx(STEPS * TIME_STEP * VELOCITY_MAX / 2.0, abs=1e-4)


def test_lidar_scan_is_a_float32_vector(simulation):
    simulation.step()
    scan = simulation[0].lidar_distance()
    assert scan.shape == (LIDAR_RAYS,)
    assert scan.dtype == np.float32
    # REP 117: +inf is no return, -inf too close; finite readings lie in the sensor range
    finite = scan[np.isfinite(scan)]
    assert finite.size > 0
    assert (LIDAR_RANGE_MIN <= finite).all() and (finite <= LIDAR_RANGE_MAX).all()
    normalized = simulation[0].lidar_distance(0, True)
    assert np.isfinite(normalized).all()
    assert (0.0 <= normalized).all() and (normalized <= 1.0).all()


def test_lidar_reports_no_return_as_inf(world_data):
    simulation = murosimpy.core.Simulation(world_data)
    robot = build_robot("far")
    robot.pose = [100.0 * WORLD_SIZE, 100.0 * WORLD_SIZE, 0.0]  # nothing within range
    robot.goal = [100.0 * WORLD_SIZE + 1.0, 100.0 * WORLD_SIZE, GOAL_RADIUS]
    simulation.add_robot(robot)
    simulation.reset()

    assert np.isposinf(robot.lidar_distance()).all()
    assert (robot.lidar_distance(0, True) == 1.0).all()  # normalized: no return reads 1


def test_grid_map_is_uint8(simulation):
    grid = simulation.get_map()
    assert grid.shape == (MAP_SIZE, MAP_SIZE)
    assert grid.dtype == np.uint8
    assert 0 in grid and 255 in grid  # free cells and the walls
    assert 128 not in simulation.get_map(False)  # the buffer band is not drawn


def test_robot_outlives_its_python_reference(world_data):
    simulation = murosimpy.core.Simulation(world_data)
    robot = build_robot("kept_alive")
    robot.pose = ROBOT_POSES[0]
    simulation.add_robot(robot)
    del robot
    gc.collect()
    simulation.reset()
    simulation.step()
    assert simulation[0].name == "kept_alive"
    assert simulation[0].steps == 1


def test_clone_is_independent(simulation):
    clone = simulation.clone()
    clone[0].command_velocity = [VELOCITY_MAX, 0.0, 0.0]
    for _ in range(STEPS):
        clone.step()
    assert clone.steps == STEPS
    assert simulation.steps == 0
    np.testing.assert_allclose(simulation[0].pose, ROBOT_POSES[0])
    assert copy.deepcopy(simulation).steps == 0


def test_config_is_validated_and_applied(simulation):
    config = murosimpy.definitions.SimulationConfig()
    with pytest.raises(ValueError):
        config.time_step = 0.0
    config.time_step = 0.05
    config.goal_radius = 0.3
    simulation.set_config(config)
    assert simulation.config.time_step == pytest.approx(0.05)
    assert simulation.config.goal_radius == pytest.approx(0.3)


def test_missing_world_file_raises():
    with pytest.raises(RuntimeError):
        murosimpy.core.Simulation("no_such_file.world")


# ------------------------------------------------------------- containers

def test_thread_pool_steps_every_simulation(world_data, thread_pool):
    simulations = murosimpy.containers.Simulations()
    for _ in range(3):
        simulations.add(build_simulation(world_data))
    for simulation in simulations:
        simulation[0].command_velocity = [VELOCITY_MAX / 2.0, 0.0, 0.0]
    thread_pool.execute_simulate(simulations)
    assert thread_pool.thread_count == 2
    assert [simulation.steps for simulation in simulations] == [1, 1, 1]
    assert len(simulations.all_robots) == 6
    assert simulations.robots_per_simulation == [2, 2, 2]
    assert simulations.max_world_size == pytest.approx(WORLD_SIZE)


def test_robots_container_iterates_in_order(simulation):
    robots = simulation.robots
    assert len(robots) == 2
    assert [robot.name for robot in robots] == ["robot_0", "robot_1"]
    active, indices = robots.active_robots()
    assert len(active) == 2 and indices == [0, 1]


# --------------------------------------------------------------------- io

def test_world_file_round_trip(world_data, tmp_path):
    path = str(tmp_path / "smoke.world")
    murosimpy.io.save_world(path, world_data)
    loaded = murosimpy.io.load_world(path)
    assert loaded.line_count == world_data.line_count
    assert loaded.node_count == world_data.node_count
    assert loaded.node_at(0).x == pytest.approx(world_data.node_at(0).x)
    assert murosimpy.core.Simulation(path).max_robots == world_data.node_count


def test_data_container_from_python_lists():
    lines = murosimpy.geometry.primitives.square(4.0).lines
    nodes = [murosimpy.navigation.Node(0.0, 0.0, 0.5), murosimpy.navigation.Node(1.0, 1.0, 0.5)]
    data = murosimpy.io.DataContainer(lines, [], nodes, "from_lists")
    assert data.line_count == 4 and data.circle_count == 0 and data.node_count == 2
    assert data.name == "from_lists"
    assert murosimpy.core.Simulation(data).max_robots == 2


def test_enums_convert_to_int():
    assert int(murosimpy.definitions.InitMode.MANUAL) == 0
    assert murosimpy.definitions.PlannerType(0) == murosimpy.definitions.PlannerType.NONE
    assert murosimpy.core.LidarNoiseType.UNIFORM.name == "UNIFORM"


def test_robot_reaches_its_goal(world_data):
    """End-to-end: a robot driven at its goal arrives and reports finished.

    Guards the fixtures too: with a zero goal radius (a goal is [x, y, radius])
    the reach test can never succeed, and every other test would still pass.
    """
    simulation = murosimpy.core.Simulation(world_data)
    robot = build_robot("driver")
    robot.pose = [2.0, 2.0, 0.0]
    robot.goal = [6.0, 2.0, GOAL_RADIUS]
    simulation.add_robot(robot)
    simulation.reset()

    for _ in range(200):
        x, y, rotation = robot.pose
        goal_x, goal_y, _ = robot.goal
        distance = math.hypot(goal_x - x, goal_y - y)
        bearing = math.atan2(goal_y - y, goal_x - x) - rotation
        speed = min(VELOCITY_MAX, distance)
        robot.command_velocity = [speed * math.cos(bearing), speed * math.sin(bearing), 0.0]
        simulation.step()
        if simulation.finished:
            break

    assert robot.finished
    assert robot.collision_status == murosimpy.definitions.CollisionStatus.NONE
    assert math.hypot(robot.pose[0] - 6.0, robot.pose[1] - 2.0) < GOAL_RADIUS
