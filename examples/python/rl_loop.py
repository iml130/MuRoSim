#!/usr/bin/env python3
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""Reinforcement-learning episodes across all three modules.

murosimprocpy generates a world, murosimpy simulates it, murosimrlpy turns it
into observations, actions, rewards and done flags. `Environment` is the
reset()/step() shape a training framework wraps, `policy()` a hand-written
heuristic standing in for the network. Two episodes run in the same world to
show that reset() draws new tasks each time.

    pip install .
    python examples/python/rl_loop.py
"""

import math

import numpy as np

import murosimpy
import murosimprocpy
import murosimrlpy

SEED = 42
ROBOT_COUNT = 4
EPISODES = 2
EPISODE_STEPS = 300
SPEED = 1.0
LIDAR_RAYS = 32
LIDAR_RANGE = 10.0
LIDAR_RANGE_MIN = 0.05
BRAKE_RADIUS = 1.0  # OrientationToGoal shrinks inside this radius
AVOID_DISTANCE = 1.0  # the policy reacts to lidar readings closer than this
AVOID_PUSH = 0.5  # how hard it backs away from the nearest obstacle
AVOID_SIDESTEP = 0.5  # how hard it steps to the obstacle's right-hand side

# A full-circle lidar's rays are spread evenly, counter-clockwise from -pi, in
# the sensor frame, which is the robot frame for a lidar mounted at identity.
RAY_ANGLES = np.linspace(-math.pi, math.pi, LIDAR_RAYS, endpoint=False)
RAY_DIRECTIONS = np.stack([np.cos(RAY_ANGLES), np.sin(RAY_ANGLES)], axis=1).astype(np.float32)


def build_world(seed=SEED):
    """A procedurally generated world. The same seed always gives the same world.

    Swap in MazeGenerator, GraphGenerator, WFCGenerator, ... for a different
    kind of layout; they all take the same first four arguments and return a
    DataContainer.
    """
    generator = murosimprocpy.RandomSparseGenerator(
        min_node_count=2 * ROBOT_COUNT,  # spare nodes, so resets draw new tasks
        node_radius=0.25,
        world_size=murosimprocpy.RandomRange(12.0, 16.0),
        spacing=murosimprocpy.RandomRange(0.8, 1.25),
    )
    return generator(seed)


def build_robot(name):
    builder = murosimpy.core.RobotBuilder()
    builder.set_name(name)
    builder.set_actuator_holonomic_simple(-SPEED, SPEED, SPEED, SPEED)
    builder.set_shape_robot_rectangle(0.3, 0.3)
    builder.add_lidar(murosimpy.geometry.Transform(), fov=2.0 * math.pi, resolution=LIDAR_RAYS,
                      range_min=LIDAR_RANGE_MIN, range_max=LIDAR_RANGE, frequency=10.0)
    return builder.build()


def build_simulation(data):
    simulation = murosimpy.core.Simulation(data)
    for index in range(min(ROBOT_COUNT, data.node_count)):
        simulation.add_robot(build_robot(f"robot_{index}"))

    # NODE_ALL_RANDOM draws a start and a goal node per robot on every reset,
    # so each episode sees a different task in the same world. The seed makes
    # that draw reproducible -- leave it out and every run differs.
    config = simulation.config
    config.init_mode = murosimpy.definitions.InitMode.NODE_ALL_RANDOM
    config.seed = SEED
    simulation.set_config(config)
    return simulation


# Positions of the observations in build_observation_set(), in insertion order.
LIDAR_OBSERVATION, ORIENTATION_OBSERVATION = 1, 2


def build_observation_set():
    """What the policy sees.

    Every observation works on one robot, a robot container or a whole
    simulation; a set computes them all in one call and keeps insertion order.
    """
    observation_set = murosimrlpy.observation.ObservationSet()
    observation_set.add(murosimrlpy.observation.robot.Pose())
    observation_set.add(murosimrlpy.observation.robot.Lidar(0, LIDAR_RANGE))
    observation_set.add(murosimrlpy.observation.robot.OrientationToGoal(BRAKE_RADIUS))
    return observation_set


def build_reward_set():
    """What the policy is scored on. A RewardSet sums its members per robot."""
    reward_set = murosimrlpy.reward.RewardSet()
    # Progress towards the goal is scaled by the first factor, moving away by
    # the second -- which has to be negative to be a penalty.
    reward_set.add(murosimrlpy.reward.robot.DeltaDistanceToGoal(1.0, -1.0))
    reward_set.add(murosimrlpy.reward.robot.OnGoal(0.2, 1.0))
    reward_set.add(murosimrlpy.reward.robot.MinimumLidarDistance(0.5, -1.0))
    return reward_set


class Environment:
    """reset()/step() around one simulation: the shape a training loop wraps.

    Observations are what the ObservationSet computes, one array per
    observation with the robots along the first axis. step() returns them
    together with one reward and one done flag per robot. A robot that had
    already finished before the step gets reward 0, so a crashed robot does
    not keep collecting the lidar penalty for the rest of the episode.

    done means terminated: the robot reached its goal or collided. The step
    limit in main() is a truncation that only the loop knows about; a
    Gymnasium wrapper reports it separately as `truncated`, so a timeout is
    not mistaken for a terminal state.
    """

    def __init__(self, simulation, observation_set, reward_set, thread_pool):
        self.simulation = simulation
        self.robots = simulation.robots
        self.observation_set = observation_set
        self.reward_set = reward_set
        self.thread_pool = thread_pool
        self.action = murosimrlpy.core.Action(normalize=True)
        self.done = murosimrlpy.core.Done()

    def observe(self):
        return self.observation_set.compute(self.thread_pool, self.robots)

    def reset(self):
        # reset() places the robots and computes the first state, which the
        # observations need -- without it they raise.
        self.simulation.reset()
        return self.observe()

    def step(self, commands):
        active = ~self.done(self.robots).ravel()
        self.action(self.robots, commands)
        self.simulation.step()
        reward = self.reward_set.compute(self.thread_pool, self.robots).ravel() * active
        return self.observe(), reward, self.done(self.robots).ravel()


def policy(lidar, orientation_to_goal):
    """Head for the goal, sliding along whatever the lidar sees nearby.

    Replace this with your network. Both inputs are in the robot frame:
    orientation_to_goal is a unit vector towards the goal (shrinking inside
    BRAKE_RADIUS), lidar is the normalized scan (LIDAR_RANGE_MIN reads 0,
    LIDAR_RANGE and "no return" read 1). Only the nearest reading
    matters: once it is closer than AVOID_DISTANCE, the part of the goal pull
    that points into that obstacle is removed (so the robot slides along it
    instead of pushing against it), a small push keeps some clearance, and a
    small step to the obstacle's right-hand side breaks the symmetry when two
    robots meet head-on -- both step right and pass each other. A pure
    repulsion stalls between two close walls; this does not.
    """
    distance = LIDAR_RANGE_MIN + lidar * (LIDAR_RANGE - LIDAR_RANGE_MIN)
    rows = np.arange(len(lidar))
    nearest = distance.argmin(axis=1)
    normal = RAY_DIRECTIONS[nearest]  # unit vector towards the nearest obstacle
    weight = np.clip(1.0 - distance[rows, nearest] / AVOID_DISTANCE, 0.0, 1.0)[:, None]
    into_obstacle = np.clip((orientation_to_goal * normal).sum(axis=1, keepdims=True), 0.0, None)
    sidestep = np.stack([normal[:, 1], -normal[:, 0]], axis=1)

    move = orientation_to_goal - weight * (into_obstacle * normal + AVOID_PUSH * normal
                                           - AVOID_SIDESTEP * sidestep)
    commands = np.zeros((len(lidar), 3), dtype=np.float32)
    commands[:, :2] = np.clip(move, -1.0, 1.0)  # normalized actions live in [-1, 1] per axis
    return commands


def main():
    data = build_world()
    simulation = build_simulation(data)
    robots = simulation.robots
    print(f"world '{data.name}': {data.line_count} lines, {data.node_count} nodes, "
          f"{len(robots)} robots")

    observation_set = build_observation_set()
    reward_set = build_reward_set()
    thread_pool = murosimpy.core.ThreadPool(0)  # 0: one thread per core
    environment = Environment(simulation, observation_set, reward_set, thread_pool)
    print(f"observation shapes {observation_set.shapes(robots)}, "
          f"action shape {environment.action.shape(robots)}")

    none = murosimpy.definitions.CollisionStatus.NONE
    for episode in range(EPISODES):
        observations = environment.reset()
        starts = ", ".join(f"({x:.1f}, {y:.1f})" for x, y in observations[0][:, :2])
        print(f"episode {episode}: robots start at {starts}")

        episode_return = np.zeros(len(robots), dtype=np.float32)
        for step in range(EPISODE_STEPS):
            commands = policy(observations[LIDAR_OBSERVATION], observations[ORIENTATION_OBSERVATION])
            observations, reward, flags = environment.step(commands)
            episode_return += reward

            if step % 50 == 0:
                print(f"  step {step:3d}  return {episode_return.round(2)}  done {flags}")
            if flags.all():
                break

        # A robot is "finished" either by arriving or by crashing, and one that
        # simply ran out of steps is neither -- so all three have to be counted
        # apart.
        reached = sum(robot.finished and robot.collision_status == none for robot in robots)
        crashed = sum(robot.collision_status != none for robot in robots)
        print(f"  over after {simulation.steps} steps: {reached} reached their goal, "
              f"{crashed} hit something, {len(robots) - reached - crashed} still driving")
        for index, robot in enumerate(robots):
            print(f"    {robot.name}: return {episode_return[index]:7.2f}, "
                  f"{robot.traveled_distance:5.2f} m travelled, "
                  f"collision status {robot.collision_status.name}")


if __name__ == "__main__":
    main()
