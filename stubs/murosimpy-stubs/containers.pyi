# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Container types for managing multiple simulations or robots.
"""
from __future__ import annotations
import collections.abc
import murosimpy.core
import typing
__all__: list[str] = ['Robots', 'Simulations']
class Robots:
    """
    A container for managing multiple Robot instances, e.g. to collect
    robots gathered from several simulations for batched observation/reward
    computation.
    """
    def __copy__(self) -> Robots:
        """
        Support for copy.copy(): returns a shallow copy sharing the same Robot instances.
        """
    def __deepcopy__(self, memo: dict) -> Robots:
        """
        Support for copy.deepcopy(): returns a new container holding deep copies of every robot.
        """
    def __getitem__(self, index: typing.SupportsInt | typing.SupportsIndex) -> murosimpy.core.Robot:
        """
        Get robot at the specified index.
        """
    def __init__(self) -> None:
        """
        Create an empty robot container.
        """
    def __iter__(self) -> collections.abc.Iterator[murosimpy.core.Robot]:
        """
        Iterate over all robots.

        The container is kept alive for as long as the returned iterator lives.
        """
    def __len__(self) -> int:
        """
        Return the number of robots.
        """
    def __repr__(self) -> str:
        """
        Return a short debug string with the number of contained robots.
        """
    def active_robots(self) -> tuple[Robots, list[int]]:
        """
        Get active (non-finished) robots and their indices.

        Returns:
            Tuple of (active_robots, indices)
        """
    @typing.overload
    def add(self, robot: murosimpy.core.Robot) -> None:
        """
        Add a single robot to the container.
        """
    @typing.overload
    def add(self, container: Robots) -> None:
        """
        Add all robots from another container.
        """
    def clear(self) -> None:
        """
        Remove all robots from the container.
        """
    @property
    def as_list(self) -> list:
        """
        Container as a Python list.
        """
class Simulations:
    """
    A container for managing multiple Simulation instances, e.g. for use
    with ThreadPool.execute_simulate() to step them all in parallel.
    """
    def __copy__(self) -> Simulations:
        """
        Support for copy.copy(): returns a shallow copy sharing the same Simulation instances.
        """
    def __deepcopy__(self, memo: dict) -> Simulations:
        """
        Support for copy.deepcopy(): returns a new container holding deep copies of every simulation.
        """
    def __getitem__(self, index: typing.SupportsInt | typing.SupportsIndex) -> murosimpy.core.Simulation:
        """
        Get simulation at the specified index.
        """
    def __init__(self) -> None:
        """
        Create an empty simulation container.
        """
    def __iter__(self) -> collections.abc.Iterator[murosimpy.core.Simulation]:
        """
        Iterate over all simulations.

        The container is kept alive for as long as the returned iterator lives.
        """
    def __len__(self) -> int:
        """
        Return the number of simulations.
        """
    def __repr__(self) -> str:
        """
        Return a short debug string with the number of contained simulations.
        """
    def active_robots(self) -> tuple[Robots, list[int]]:
        """
        Get active (non-finished) robots and their global indices.

        Returns:
            Tuple of (active_robots, indices)
        """
    def active_simulations(self) -> tuple[Simulations, list[int]]:
        """
        Get active (non-finished) simulations and their indices.

        Returns:
            Tuple of (active_simulations, indices)
        """
    @typing.overload
    def add(self, simulation: murosimpy.core.Simulation) -> None:
        """
        Add a single simulation to the container.
        """
    @typing.overload
    def add(self, container: Simulations) -> None:
        """
        Add all simulations from another container.
        """
    def clear(self) -> None:
        """
        Remove all simulations from the container.
        """
    @property
    def active_robot_counts(self) -> list[int]:
        """
        Number of active (non-finished) robots in each simulation as a list.
        """
    @property
    def all_robots(self) -> Robots:
        """
        All robots from all simulations as a flat list.
        """
    @property
    def as_list(self) -> list:
        """
        Container as a Python list.
        """
    @property
    def max_world_size(self) -> float:
        """
        Maximum world size (x-axis) across all simulations.
        """
    @property
    def robots_per_simulation(self) -> list[int]:
        """
        Number of robots in each simulation as a list.
        """
