#!/usr/bin/env python3
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""Drive a MiR100 preset: a differential-drive robot with acceleration limits.

The presets in murosimpy.models.robot are complete robots -- body outline,
actuator limits, lidars -- so a realistic platform is one call. Unlike the
holonomic robot in simulation.py, this one cannot drive sideways, ramps its
speed at a finite acceleration and carries two 270-degree lidars on opposite
corners, all of which shows in how it has to be steered.

    pip install .
    python examples/python/mir100.py
"""

import math

import numpy as np

import murosimpy

WORLD_SIZE = 12.0
MAX_STEPS = 400
START = (2.0, 2.0, 0.0)  # pose: x, y, heading
GOAL = (10.0, 10.0, 0.5)  # goal: x, y, radius -- 45 degrees off the start heading
GAIN_ANGULAR = 2.0


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def steer(robot):
    """Turn towards the goal; drive forward only as far as the robot faces it.

    A differential drive takes [forward, 0, angular]: the lateral component is
    outside the actuator's limits and clamped to zero, so steering means
    turning. Forward speed scales with cos(bearing) and drops to zero when the
    goal is behind, which makes the robot turn on the spot first.
    """
    x, y, heading = robot.pose
    goal_x, goal_y, _ = robot.goal
    distance = math.hypot(goal_x - x, goal_y - y)
    bearing = wrap_angle(math.atan2(goal_y - y, goal_x - x) - heading)

    forward_max = robot.velocity_max[0]
    angular_max = robot.velocity_max[2]
    angular = max(-angular_max, min(angular_max, GAIN_ANGULAR * bearing))
    forward = forward_max * max(0.0, math.cos(bearing)) * min(1.0, distance)
    robot.command_velocity = [forward, 0.0, angular]


def main():
    robot = murosimpy.models.robot.mir100()
    print(f"preset '{robot.name}': actuator {robot.actuator_type.name}, "
          f"{robot.lidar_count} lidars, velocity limits "
          f"{np.round(robot.velocity_min, 2)} .. {np.round(robot.velocity_max, 2)}")

    simulation = murosimpy.core.Simulation(murosimpy.models.world.empty(WORLD_SIZE, space=1.0))
    robot.pose = START
    robot.goal = GOAL
    simulation.add_robot(robot)
    simulation.reset()

    # The lateral component of a command is clamped away before it is stored.
    robot.command_velocity = [1.0, 1.0, 0.0]
    print(f"command [1, 1, 0] is stored as {robot.command_velocity}")

    # The actuator ramps towards a command at its acceleration limit, so a
    # full-speed command takes a while to reach, as the printout shows.
    robot.command_velocity = [robot.velocity_max[0], 0.0, 0.0]
    for step in range(3):
        simulation.step()
        print(f"  after {(step + 1) * simulation.config.time_step:.1f} s: "
              f"forward velocity {robot.velocity[0]:.2f} m/s")

    for _ in range(MAX_STEPS):
        steer(robot)
        simulation.step()

        step = simulation.steps  # counts the three ramp steps above as well
        if step % 20 == 0:
            x, y, heading = robot.pose
            # Each lidar is read by index; the two cover 270 degrees each on
            # opposite corners, together the full circle.
            front, rear = robot.lidar_distance(0).min(), robot.lidar_distance(1).min()
            print(f"  step {step:3d}  at ({x:5.2f}, {y:5.2f}) "
                  f"heading {math.degrees(heading):6.1f} deg  "
                  f"velocity [{robot.velocity[0]:4.2f} m/s, {robot.velocity[2]:5.2f} rad/s]  "
                  f"nearest obstacle front {front:4.2f} m, rear {rear:4.2f} m")

        if simulation.finished:
            break

    none = murosimpy.definitions.CollisionStatus.NONE
    reached = robot.finished and robot.collision_status == none
    print(f"{'reached the goal' if reached else 'did not reach the goal'} "
          f"after {simulation.steps} steps, {robot.traveled_distance:.2f} m travelled, "
          f"collision status {robot.collision_status.name}")


if __name__ == "__main__":
    main()
