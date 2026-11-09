#!/usr/bin/env python3
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""Watch the rl_loop.py episodes in a window.

The other Python examples are headless on purpose; this one draws the same
world, robots and policy as rl_loop.py with the on-screen renderer, one frame
per simulation step. Needs a display. render() also returns each frame as an
RGB numpy array, so saving images or video is a matter of handing that array
to whatever image library you already use -- MuRoSim ships none.

    pip install .
    python examples/python/render.py

Any key in the window ends the current episode, closing the window ends the
program.
"""

import sys

import murosimpy

# Shares the world, robots, observations, rewards, Environment and policy with
# rl_loop.py.
from rl_loop import (EPISODE_STEPS, LIDAR_OBSERVATION, ORIENTATION_OBSERVATION, Environment,
                     build_observation_set, build_reward_set, build_simulation, build_world, policy)

EPISODES = 5
RESOLUTION = 800  # window size in pixels
FRAME_MS = 50  # pause per step, so 0.1 s of simulated time shows for 0.05 s


def main():
    try:
        renderer = murosimpy.rendering.OnlineRenderer(RESOLUTION)
    except RuntimeError as error:
        print(f"cannot open a window ({error}); this example needs a display", file=sys.stderr)
        return 1

    simulation = build_simulation(build_world())
    environment = Environment(simulation, build_observation_set(), build_reward_set(),
                              murosimpy.core.ThreadPool(0))

    # Robots are drawn with their goals; lidar rays only once lidar_skip_steps
    # says which of them to draw (1: every ray, 0: none, the default). The
    # colours are robot properties and can be changed per robot (color_robot,
    # color_lidar, ...).
    renderer.lidar_skip_steps = 1
    frame = renderer.render(simulation)
    print(f"frames are {frame.shape[1]}x{frame.shape[0]} RGB arrays, top row first; "
          f"any key ends the episode, closing the window ends the program")

    for episode in range(EPISODES):
        observations = environment.reset()
        for _ in range(EPISODE_STEPS):
            commands = policy(observations[LIDAR_OBSERVATION], observations[ORIENTATION_OBSERVATION])
            observations, _, flags = environment.step(commands)
            renderer.render(simulation)

            # wait_key() returns a key code, or -1 on timeout and on a closed
            # window alike; is_open tells the two apart.
            key = renderer.wait_key(FRAME_MS)
            if not renderer.is_open:
                print("window closed")
                return 0
            if key != -1 or flags.all():
                break
        print(f"episode {episode} over after {simulation.steps} steps")
    return 0


if __name__ == "__main__":
    sys.exit(main())
