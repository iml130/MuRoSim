#!/usr/bin/env python3
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""Step a batch of simulations in parallel from one thread pool.

This is what MuRoSim is built for: simulations are independent, so a whole
batch is stepped, observed and rewarded in one call each, spread over every
core. Each simulation here runs on its own procedurally generated world. The
batch is rolling -- a simulation that finishes or times out is reset on the
spot instead of waiting for the slowest episode in the batch -- which is the
shape an on-policy training loop has. World, robots, observations, rewards
and policy are the ones from rl_loop.py.

    pip install .
    python examples/python/batch.py
"""

import time

import numpy as np

import murosimpy
import murosimrlpy

from rl_loop import (LIDAR_OBSERVATION, ORIENTATION_OBSERVATION, ROBOT_COUNT, SEED,
                     build_observation_set, build_reward_set, build_robot, build_world, policy)

SIMULATION_COUNT = 64
TOTAL_STEPS = 1000  # batch steps to run in total
EPISODE_STEPS = 150  # a simulation is reset after this many steps even if nobody finished


def build_simulation(index):
    """One simulation on its own world. The seed makes world and tasks reproducible."""
    simulation = murosimpy.core.Simulation(build_world(SEED + index))
    for robot_index in range(ROBOT_COUNT):
        simulation.add_robot(build_robot(f"robot_{robot_index}"))

    config = simulation.config
    config.init_mode = murosimpy.definitions.InitMode.NODE_ALL_RANDOM
    config.seed = SEED + index
    simulation.set_config(config)
    return simulation


def main():
    thread_pool = murosimpy.core.ThreadPool(0)  # 0: one thread per core

    simulations = murosimpy.containers.Simulations()
    for index in range(SIMULATION_COUNT):
        simulations.add(build_simulation(index))
    # The parallel form of calling reset() on each simulation.
    thread_pool.execute_reset(simulations)

    # A flat view over every robot of every simulation, in simulation-then-robot
    # order. Observations, actions and rewards work on it like on a single
    # simulation's robots, so the batch costs no extra code.
    robots = simulations.all_robots
    print(f"{len(simulations)} simulations x {ROBOT_COUNT} robots = {len(robots)} robots, "
          f"{thread_pool.thread_count} threads, "
          f"largest world {simulations.max_world_size:.1f} m")

    observation_set = build_observation_set()
    reward_set = build_reward_set()

    action = murosimrlpy.core.Action(normalize=True)
    done = murosimrlpy.core.Done()
    none = murosimpy.definitions.CollisionStatus.NONE

    print(f"observation shapes {observation_set.shapes(robots)}, "
          f"action shape {action.shape(robots)}")

    outcomes = {"reached the goal": 0, "collided": 0, "timed out": 0}
    episode_return = np.zeros(len(robots), dtype=np.float32)
    returns = []  # one entry per finished robot episode

    start = time.perf_counter()
    for _ in range(TOTAL_STEPS):
        # A robot that already finished keeps its done flag until its simulation
        # is reset; its reward must not be counted any more.
        active = ~done(simulations).ravel()

        observations = observation_set.compute(thread_pool, robots)
        action(robots, policy(observations[LIDAR_OBSERVATION], observations[ORIENTATION_OBSERVATION]))
        thread_pool.execute_simulate(simulations)
        episode_return += reward_set.compute(thread_pool, robots).ravel() * active

        # Settle the episodes that end this step, then reset those simulations
        # in parallel. reset() re-draws start and goal nodes and zeroes the
        # step counter, so the batch rolls on without a pause. The condition
        # is the one execute_reset_by_finished() applies.
        offset = 0
        for simulation in simulations:
            count = len(simulation.robots)
            if simulation.finished or simulation.steps >= EPISODE_STEPS:
                for robot_index, robot in enumerate(simulation.robots):
                    if robot.collision_status != none:
                        outcomes["collided"] += 1
                    elif robot.finished:
                        outcomes["reached the goal"] += 1
                    else:
                        outcomes["timed out"] += 1
                    returns.append(episode_return[offset + robot_index])
                episode_return[offset:offset + count] = 0.0
            offset += count
        thread_pool.execute_reset_by_finished(simulations, EPISODE_STEPS)
    elapsed = time.perf_counter() - start

    # End to end: observing, the Python policy, stepping, rewarding and the
    # bookkeeping above, not the bare simulation speed.
    robot_steps = TOTAL_STEPS * len(robots)
    print(f"{TOTAL_STEPS} batch steps in {elapsed:.2f} s end to end: "
          f"{TOTAL_STEPS * len(simulations) / elapsed:,.0f} simulation steps/s, "
          f"{robot_steps / elapsed:,.0f} robot steps/s")
    episodes = sum(outcomes.values())
    print(f"{episodes} robot episodes finished, mean return {np.mean(returns):.2f}")
    for outcome, count in outcomes.items():
        print(f"  {count:4d} {outcome}")


if __name__ == "__main__":
    main()
