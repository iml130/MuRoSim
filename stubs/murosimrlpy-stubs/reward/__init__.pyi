# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Reward functions for reinforcement learning.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.core
import numpy
import numpy.typing
import typing
from . import robot
from . import simulation
__all__: list[str] = ['RewardSet', 'robot', 'simulation']
class RewardSet:
    """
    Collection of robot rewards summed in one thread-pool batch.
    The sum runs in add() order per robot, so the result is
    bit-identical to summing the rewards serially.
    """
    def __init__(self, min_chunk_size: typing.SupportsInt | typing.SupportsIndex = 16) -> None:
        """
        min_chunk_size: smallest robot count worth its own thread-pool job; small lists collapse into fewer chunks.
        """
    def __len__(self) -> int:
        """
        Number of rewards currently in the set.
        """
    def add(self, reward: robot.RewardRobot) -> None:
        """
        Append a reward; compute() sums all rewards in add() order.
        """
    def compute(self, thread_pool: murosimpy.core.ThreadPool, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Sum all rewards for `robots` on the pool; returns a (N, 1) array.
        """
