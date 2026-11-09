#!/usr/bin/env python3
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""Load a hand-written .world file and simulate in it.

warehouse.world next to this script is a 12 x 8 m hall with two shelf rows.
The file format is documented in its header: wall segments, round obstacles
and navigation nodes, where nodes carry a group so that
InitMode.NODE_GROUP_RANDOM pairs each robot's start and goal within one
aisle. See save_world.py for writing such a file from code.

    pip install .
    python examples/python/world_file.py
"""

import math
from pathlib import Path

import murosimpy

WORLD_FILE = Path(__file__).with_name("warehouse.world")
ROBOT_COUNT = 3
MAX_STEPS = 300
SEED = 7
SPEED = 1.0


def build_robot(name):
    builder = murosimpy.core.RobotBuilder()
    builder.set_name(name)
    builder.set_actuator_holonomic_simple(-SPEED, SPEED, SPEED, SPEED)
    builder.set_shape_robot_rectangle(0.3, 0.3)
    builder.add_lidar(murosimpy.geometry.Transform(), 2.0 * math.pi, 32, 0.05, 10.0, 10.0)
    return builder.build()


def drive_towards_goal(robot):
    """The straight-line controller from simulation.py."""
    x, y, rotation = robot.pose
    goal_x, goal_y, _ = robot.goal
    distance = math.hypot(goal_x - x, goal_y - y)
    bearing = math.atan2(goal_y - y, goal_x - x) - rotation
    speed = min(SPEED, distance)
    robot.command_velocity = [speed * math.cos(bearing), speed * math.sin(bearing), 0.0]


def node_near(nodes, x, y):
    """The node whose centre is closest to (x, y)."""
    return min(nodes, key=lambda node: math.hypot(node.x - x, node.y - y))


def main():
    # load_world() parses the file into a DataContainer named after the file's
    # stem. It raises RuntimeError with the offending line number on any
    # malformed record.
    data = murosimpy.io.load_world(str(WORLD_FILE))
    print(f"world '{data.name}': {data.line_count} lines, {data.circle_count} circles, "
          f"{data.node_count} nodes")
    for node in data.nodes:
        print(f"  node {node.id} at ({node.x:4.1f}, {node.y:4.2f}) radius {node.radius:.1f} "
              f"group {node.group} start heading {math.degrees(node.start_angle):5.1f} deg")

    # scale and mirror apply to everything in the file, which lets one file
    # serve as a family of worlds; the same file at half size:
    half = murosimpy.io.load_world(str(WORLD_FILE), scale=0.5)
    last = half.nodes[-1]
    print(f"at scale 0.5 node {last.id} sits at ({last.x:.2f}, {last.y:.3f})")

    # A Simulation can also be built from the path directly. Either way the
    # node count caps how many robots fit.
    from_path = murosimpy.core.Simulation(str(WORLD_FILE))
    print(f"Simulation(path) accepts up to {from_path.max_robots} robots")

    simulation = murosimpy.core.Simulation(data)
    for index in range(ROBOT_COUNT):
        simulation.add_robot(build_robot(f"robot_{index}"))
    config = simulation.config
    config.init_mode = murosimpy.definitions.InitMode.NODE_GROUP_RANDOM
    config.seed = SEED
    simulation.set_config(config)
    simulation.reset()

    # Start and goal of every robot come from the same group, i.e. the same
    # aisle, so a straight line between them is free of shelves.
    nodes = data.nodes
    for robot in simulation.robots:
        start = node_near(nodes, *robot.position)
        goal = node_near(nodes, robot.goal[0], robot.goal[1])
        print(f"  {robot.name}: node {start.id} -> node {goal.id} (group {start.group})")

    for _ in range(MAX_STEPS):
        for robot in simulation.robots:
            drive_towards_goal(robot)
        simulation.step()
        if simulation.finished:
            break

    none = murosimpy.definitions.CollisionStatus.NONE
    reached = sum(robot.finished and robot.collision_status == none for robot in simulation.robots)
    crashed = sum(robot.collision_status != none for robot in simulation.robots)
    print(f"after {simulation.steps} steps: {reached} reached their goal, {crashed} collided, "
          f"{ROBOT_COUNT - reached - crashed} still driving")


if __name__ == "__main__":
    main()
