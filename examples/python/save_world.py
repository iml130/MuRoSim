#!/usr/bin/env python3
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""Build a world in code, save it as a .world file, load it back.

The geometry is a Shape of wall segments and circles; the places a robot can
start at or be sent to are Nodes, which have to be created by hand -- a
world without nodes can be simulated, but the node-based init modes need at
least two of them. The saved file is plain text (see warehouse.world), so
it can be checked in, edited and loaded with load_world() or passed straight
to Simulation().

    pip install .
    python examples/python/save_world.py
"""

import math
import tempfile
from pathlib import Path

import murosimpy

ROOM_SIZE = 8.0
NODE_RADIUS = 0.4
MAX_STEPS = 300
SPEED = 1.0


def build_world():
    """An 8 x 8 m room with a short inner wall and one pillar, both off the
    diagonal between the two nodes."""
    shape = murosimpy.geometry.Shape()
    shape.add_line(0.0, 0.0, ROOM_SIZE, 0.0)
    shape.add_line(ROOM_SIZE, 0.0, ROOM_SIZE, ROOM_SIZE)
    shape.add_line(ROOM_SIZE, ROOM_SIZE, 0.0, ROOM_SIZE)
    shape.add_line(0.0, ROOM_SIZE, 0.0, 0.0)
    shape.add_line(1.5, 5.0, 3.0, 6.5)  # inner wall, upper left
    shape.add_circle(6.0, 2.0, 0.3)  # pillar, lower right

    # Node(x, y, radius, start_angle, start_bias, id, group): the radius is
    # also the goal radius, a start heading is drawn from
    # start_angle +/- start_bias / 2, ids must be unique.
    dock = murosimpy.navigation.Node(1.5, 1.5, NODE_RADIUS, start_angle=math.pi / 4.0,
                                     start_bias=math.radians(10.0), id=0)
    pickup = murosimpy.navigation.Node(6.5, 6.5, NODE_RADIUS, start_angle=-3.0 * math.pi / 4.0,
                                       start_bias=math.radians(10.0), id=1)

    # The other constructor takes plain lists of Line, Circle and Node.
    return murosimpy.io.DataContainer(shape, [dock, pickup], "two_nodes")


def contents(data):
    """Every stored value of a DataContainer, for comparing two of them."""
    return ([(line.x1, line.y1, line.x2, line.y2) for line in data.lines],
            [(circle.x, circle.y, circle.radius) for circle in data.circles],
            [(node.x, node.y, node.radius, node.start_angle, node.start_bias, node.id, node.group)
             for node in data.nodes])


def build_robot(name):
    builder = murosimpy.core.RobotBuilder()
    builder.set_name(name)
    builder.set_actuator_holonomic_simple(-SPEED, SPEED, SPEED, SPEED)
    builder.set_shape_robot_rectangle(0.3, 0.3)
    builder.add_lidar(murosimpy.geometry.Transform(), 2.0 * math.pi, 32, 0.05, 10.0, 10.0)
    return builder.build()


def drive_towards_goal(robot):
    x, y, rotation = robot.pose
    goal_x, goal_y, _ = robot.goal
    distance = math.hypot(goal_x - x, goal_y - y)
    bearing = math.atan2(goal_y - y, goal_x - x) - rotation
    speed = min(SPEED, distance)
    robot.command_velocity = [speed * math.cos(bearing), speed * math.sin(bearing), 0.0]


def main():
    data = build_world()
    print(f"built '{data.name}': {data.line_count} lines, {data.circle_count} circles, "
          f"{data.node_count} nodes")

    # save_world() forces the .world extension and writes floats with enough
    # digits to round-trip exactly; angles are stored in degrees, while the
    # API takes radians. A real project would keep the file next to its code,
    # like warehouse.world; this example uses a temporary one.
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "two_nodes.world"
        murosimpy.io.save_world(str(path), data)
        print(f"saved to {path}:")
        for line in path.read_text().splitlines():
            print(f"  {line}")
        loaded = murosimpy.io.load_world(str(path))

    # The name is not stored in the file; load_world() takes it from the file name.
    if contents(loaded) != contents(data):
        raise SystemExit(f"loaded '{loaded.name}' back, but it differs from what was saved")
    print(f"loaded '{loaded.name}' back, every line, circle and node identical")

    # One robot, start at one node and goal at the other. With two nodes
    # NODE_ALL_RANDOM only decides the direction; reset() draws it anew.
    simulation = murosimpy.core.Simulation(loaded)
    simulation.add_robot(build_robot("robot_0"))
    config = simulation.config
    config.init_mode = murosimpy.definitions.InitMode.NODE_ALL_RANDOM
    config.seed = 1
    simulation.set_config(config)
    simulation.reset()

    robot = simulation.robots[0]
    x, y, heading = robot.pose
    print(f"{robot.name} starts at ({x:.2f}, {y:.2f}) heading {math.degrees(heading):.0f} deg, "
          f"goal ({robot.goal[0]:.1f}, {robot.goal[1]:.1f}) radius {robot.goal[2]:.1f}")

    for _ in range(MAX_STEPS):
        drive_towards_goal(robot)
        simulation.step()
        if simulation.finished:
            break

    none = murosimpy.definitions.CollisionStatus.NONE
    reached = robot.finished and robot.collision_status == none
    print(f"{'reached the goal' if reached else 'did not reach the goal'} "
          f"after {simulation.steps} steps, "
          f"{robot.traveled_distance:.2f} m travelled")


if __name__ == "__main__":
    main()
