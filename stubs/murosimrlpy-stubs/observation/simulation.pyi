# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Simulation-level observations.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.core
import numpy
import numpy.typing
import typing
__all__: list[str] = ['OccupancyMap', 'PathMap', 'VelocityMap']
class OccupancyMap:
    """
    Full occupancy grid map of each simulation.

    Embedded centered in a map_size x map_size buffer, with values from the
    grid value lookup table (0.0 free, 1.0 occupied/unreachable, 0.5 buffer if
    with_buffer); cells outside the map's own extent (padding, or when the map
    is smaller than map_size) read 1.0. Feature shape is (1, map_size,
    map_size) per simulation (channel-first for CNN input, not stacked, since
    the map does not change within an episode). Batch size is the number of
    simulations. The map must fit within map_size x map_size (raises
    otherwise).

    Args:
        map_size: Side length of the square output buffer, in cells; must be
            at least the largest grid map's size. Raises if 0.
        with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
    """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `simulations` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> OccupancyMap:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> OccupancyMap:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, map_size: typing.SupportsInt | typing.SupportsIndex, with_buffer: bool = False) -> None:
        """
        Args:
            map_size: Side length of the square output buffer, in cells; must be at least the largest grid map's size. Raises if 0.
            with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
        """
    def map_size(self) -> int:
        """
        The configured output buffer side length, in cells.
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
    def with_buffer(self) -> bool:
        """
        Whether BUFFER cells map to 0.5 instead of free.
        """
class VelocityMap:
    """
    Rasterized world-frame velocity field of each simulation's robots.

    Embedded centered in a map_size x map_size grid (same centering as
    OccupancyMap). Channel 0 and 1 are the x and y world-frame velocity
    components (metres/second; row order is y-flipped like the grid map, so
    channel 1 stores the negated y velocity) written at each robot's occupied
    cells; channel 2 (present only if history_size > 0) is a recency weight in
    (0, 1] that fades older states in the same stacked frame towards 0.
    Feature shape per stacked state is (2, map_size, map_size) (history_size
    == 0) or (3, map_size, map_size). Cells with no robot stay 0. Each robot
    occupies either just its center cell, or every cell within its collision
    radius if use_collision_radius is set. If history_size > 0, up to that
    many past states per stacked index are blended into the same frame;
    robots without a recorded state yet are skipped. The map must fit within
    map_size x map_size (raises otherwise).

    Args:
        map_size: Side length of the square output buffer, in cells; must be
            at least the largest grid map's size. Raises if 0.
        history_size: Number of past states blended per stacked frame via a
            fading recency channel; 0 disables the history channel.
        use_collision_radius: Rasterize each robot's full collision-radius
            footprint instead of just its center cell.
        stack_size: Number of stacked past frames per simulation; 0 and 1 are
            equivalent (no stacking).
    """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `simulations` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> VelocityMap:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> VelocityMap:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, map_size: typing.SupportsInt | typing.SupportsIndex, history_size: typing.SupportsInt | typing.SupportsIndex = 0, use_collision_radius: bool = False, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            map_size: Side length of the square output buffer, in cells; must be at least the largest grid map's size. Raises if 0.
            history_size: Number of past states blended per stacked frame via a fading recency channel; 0 disables the history channel.
            use_collision_radius: Rasterize each robot's full collision-radius footprint instead of just its center cell.
            stack_size: Number of stacked past frames per simulation; 0 and 1 are equivalent (no stacking).
        """
    def history_size(self) -> int:
        """
        The configured history depth (0 means "no history channel").
        """
    def map_size(self) -> int:
        """
        The configured output buffer side length, in cells.
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
    def use_collision_radius(self) -> bool:
        """
        Whether each robot's full collision-radius footprint is rasterized.
        """
class PathMap:
    """
    Occupancy grid map of each simulation with robot paths overlaid.

    Same embedding and value convention as OccupancyMap, but with every
    robot's planned path cells overwritten to -1.0. Feature shape is (1,
    map_size, map_size) per simulation (channel-first for CNN input, not
    stacked, since path points are not part of the robot state). Batch size is
    the number of simulations. The map must fit within map_size x map_size
    (raises otherwise). Path points outside the grid are silently skipped.

    Args:
        map_size: Side length of the square output buffer, in cells; must be
            at least the largest grid map's size. Raises if 0.
        with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
    """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `simulations` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> PathMap:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> PathMap:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, map_size: typing.SupportsInt | typing.SupportsIndex, with_buffer: bool = False) -> None:
        """
        Args:
            map_size: Side length of the square output buffer, in cells; must be at least the largest grid map's size. Raises if 0.
            with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
        """
    def map_size(self) -> int:
        """
        The configured output buffer side length, in cells.
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
    def with_buffer(self) -> bool:
        """
        Whether BUFFER cells map to 0.5 instead of free.
        """
