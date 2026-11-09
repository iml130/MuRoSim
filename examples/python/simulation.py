#!/usr/bin/env python3
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
"""Build a world, put two robots in it, drive them to their goals.

The smallest complete murosimpy program: world -> robots -> step loop ->
sensor readout. Headless, no rendering.

    pip install .
    python examples/python/simulation.py
"""

import math

import murosimpy

WORLD_SIZE = 10.0  # the world spans [0, 10] on both axes
MAX_STEPS = 400
SPEED = 1.0  # m/s, the robot's velocity limit
TURN_RATE = 1.0  # rad/s, the robot's angular velocity limit
LIDAR_RAYS = 32
LIDAR_RANGE = 10.0  # rays that hit nothing within this read +inf
LIDAR_RANGE_MIN = 0.05  # hits closer than this read -inf
GOAL_RADIUS = 0.3  # a robot counts as finished inside this radius


def build_robot(name):
    """A square holonomic robot with one full-circle lidar."""
    builder = murosimpy.core.RobotBuilder()
    builder.set_name(name)
    # (linear_x_min, linear_x_max, linear_y_max, angular_max)
    builder.set_actuator_holonomic_simple(-SPEED, SPEED, SPEED, TURN_RATE)
    builder.set_shape_robot_rectangle(0.3, 0.3)
    # (pose, field_of_view, rays, range_min, range_max, frequency)
    builder.add_lidar(murosimpy.geometry.Transform(), 2.0 * math.pi, LIDAR_RAYS,
                      LIDAR_RANGE_MIN, LIDAR_RANGE, 10.0)
    return builder.build()


def drive_towards_goal(robot):
    """Point the velocity command at the goal, slowing down on approach.

    Commands are in the robot's own frame, so the world-frame bearing to the
    goal has to be rotated by the robot's heading first. Note that a goal is
    [x, y, radius], not a pose -- the third value is the goal radius.
    """
    x, y, rotation = robot.pose
    goal_x, goal_y, _ = robot.goal
    distance = math.hypot(goal_x - x, goal_y - y)
    bearing = math.atan2(goal_y - y, goal_x - x) - rotation
    speed = min(SPEED, distance)  # proportional, so it settles instead of oscillating
    robot.command_velocity = [speed * math.cos(bearing), speed * math.sin(bearing), 0.0]


def main():
    # An empty square room; pass a .world file or a murosimproc generator
    # output instead to simulate something less bare.
    world = murosimpy.models.world.empty(WORLD_SIZE, space=1.0)
    simulation = murosimpy.core.Simulation(world)

    # pose is [x, y, rotation], goal is [x, y, radius]. Two parallel lanes, so
    # the robots do not drive into each other.
    for index, (pose, goal) in enumerate([((2.0, 2.0, 0.0), (8.0, 2.0, GOAL_RADIUS)),
                                          ((2.0, 8.0, 0.0), (8.0, 8.0, GOAL_RADIUS))]):
        robot = build_robot(f"robot_{index}")
        robot.pose = pose
        robot.goal = goal
        simulation.add_robot(robot)

    # reset() places the robots and clears their state; always call it before
    # the first step.
    simulation.reset()
    print(f"world '{simulation.name}', {len(simulation)} robots, "
          f"time step {simulation.config.time_step:.3f} s")

    for _ in range(MAX_STEPS):
        for robot in simulation.robots:
            drive_towards_goal(robot)
        simulation.step()

        step = simulation.steps  # steps taken since reset()
        if step % 20 == 0:
            for robot in simulation.robots:
                x, y, _ = robot.pose
                nearest = robot.lidar_distance().min()
                print(f"  step {step:3d}  {robot.name}  at ({x:5.2f}, {y:5.2f})  "
                      f"nearest obstacle {nearest:4.2f} m")

        # A robot is finished as soon as it is inside its goal radius
        # (config.time_on_goal can require it to stay there a while) -- or
        # once it has collided, so always check the status as well.
        if simulation.finished:
            none = murosimpy.definitions.CollisionStatus.NONE
            reached = sum(robot.collision_status == none for robot in simulation.robots)
            print(f"all robots finished after {simulation.steps} steps: {reached} reached "
                  f"their goal, {len(simulation) - reached} collided")
            break
    else:
        print(f"stopped after {MAX_STEPS} steps with robots still driving")

    for robot in simulation.robots:
        print(f"  {robot.name}: {robot.traveled_distance:.2f} m travelled, "
              f"collision status {robot.collision_status}")

    # The occupancy grid the planner works on: 0 free, 128 the safety buffer
    # around obstacles, 255 occupied.
    grid = simulation.get_map()
    print(f"grid map {grid.shape[0]}x{grid.shape[1]}, {int((grid == 0).sum())} free cells")


if __name__ == "__main__":
    main()
