# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Observation functions for extracting state information.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.core
import typing
from . import robot
from . import robot_simulation
from . import simulation
__all__: list[str] = ['ObservationSet', 'robot', 'robot_simulation', 'simulation']
class ObservationSet:
    """
    Collection of robot observations computed in one thread-pool batch.
    Results are bit-identical to calling each observation serially.
    """
    def __init__(self, min_chunk_size: typing.SupportsInt | typing.SupportsIndex = 16) -> None:
        """
        min_chunk_size: smallest robot count worth its own thread-pool job; small lists collapse into fewer chunks.
        """
    def __len__(self) -> int:
        """
        Number of observations currently in the set.
        """
    def add(self, observation: robot.ObservationRobot) -> None:
        """
        Append an observation; compute() returns arrays in add() order.
        """
    def compute(self, thread_pool: murosimpy.core.ThreadPool, robots: murosimpy.containers.Robots) -> list:
        """
        Compute all observations for `robots` on the pool; returns a list of
        arrays in add() order.
        """
    def shapes(self, robots: murosimpy.containers.Robots) -> list[list[int]]:
        """
        One shape per operator, in add() order, for observing `robots`.
        """
