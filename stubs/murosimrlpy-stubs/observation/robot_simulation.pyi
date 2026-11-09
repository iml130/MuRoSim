# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Robot simulation-level observations.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.core
import numpy
import numpy.typing
import typing
__all__: list[str] = ['GoalMap', 'GoalNormalized', 'PoseMap', 'PoseNormalized']
class PoseNormalized:
    """
    Per-robot pose, normalized against the simulation world extent.

    Position is measured relative to the simulation world's AABB center and,
    if normalize_distance is positive, divided by it and shifted by 0.5 (so
    the world center maps to (0.5, 0.5)); with normalize_distance <= 0 the raw
    metre offset from the world center is reported instead. Orientation is
    appended as either a single angle in radians or a (cos, sin) unit vector.
    Feature size is 3 (x, y, angle) or 4 (x, y, cos, sin) per stacked state.
    Batch size is the total robot count across all simulations.

    Args:
        normalize_distance: Distance (metres) the offset from world center is
            divided by; <= 0 disables normalization.
        orientation_as_unit_vector: Encode orientation as (cos, sin) instead
            of a single angle.
        stack_size: Number of stacked past poses per robot; 0 and 1 are
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
    def __copy__(self) -> PoseNormalized:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> PoseNormalized:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, normalize_distance: typing.SupportsFloat | typing.SupportsIndex = 0.0, orientation_as_unit_vector: bool = False, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            normalize_distance: Distance (metres) the offset from world center is divided by; <= 0 disables normalization.
            orientation_as_unit_vector: Encode orientation as (cos, sin) instead of a single angle.
            stack_size: Number of stacked past poses per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def normalize_distance(self) -> float:
        """
        The configured normalization distance, in metres (<= 0 means "no normalization").
        """
    def orientation_as_unit_vector(self) -> bool:
        """
        Whether orientation is encoded as a (cos, sin) unit vector instead of an angle.
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
class GoalNormalized:
    """
    Per-robot goal position, normalized against the simulation world extent.

    Goal (x, y) is measured relative to the simulation world's AABB center
    and, if normalize_distance is positive, divided by it and shifted by 0.5
    (so the world center maps to (0.5, 0.5)); with normalize_distance <= 0 the
    raw metre offset from the world center is reported instead. Feature size
    is 2 per stacked state. Batch size is the total robot count across all
    simulations.

    Args:
        normalize_distance: Distance (metres) the offset from world center is
            divided by; <= 0 disables normalization.
        stack_size: Number of stacked past goals per robot; 0 and 1 are
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
    def __copy__(self) -> GoalNormalized:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> GoalNormalized:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, normalize_distance: typing.SupportsFloat | typing.SupportsIndex = 0.0, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            normalize_distance: Distance (metres) the offset from world center is divided by; <= 0 disables normalization.
            stack_size: Number of stacked past goals per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def normalize_distance(self) -> float:
        """
        The configured normalization distance, in metres (<= 0 means "no normalization").
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
class PoseMap:
    """
    Per-robot occupancy grid patch centered on the robot's own position.

    The patch is patch_dim x patch_dim grid cells, per stacked state, with
    values from the grid value lookup table (0.0 free, 1.0
    occupied/unreachable/out-of-map, 0.5 buffer if with_buffer). Feature shape
    is (patch_dim, patch_dim) per stacked state. Batch size is the total robot
    count across all simulations.

    Args:
        patch_dim: Side length of the square patch, in grid cells.
        with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
        stack_size: Number of stacked past patches per robot; 0 and 1 are
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
    def __copy__(self) -> PoseMap:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> PoseMap:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, patch_dim: typing.SupportsInt | typing.SupportsIndex, with_buffer: bool = False, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            patch_dim: Side length of the square patch, in grid cells.
            with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
            stack_size: Number of stacked past patches per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def patch_dim(self) -> int:
        """
        The configured patch side length, in grid cells.
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
class GoalMap:
    """
    Per-robot occupancy grid patch centered on the robot's goal.

    The patch is patch_dim x patch_dim grid cells, with values from the grid
    value lookup table (0.0 free, 1.0 occupied/unreachable/out-of-map, 0.5
    buffer if with_buffer). Feature shape is (1, patch_dim, patch_dim)
    (channel-first for CNN input). Batch size is the total robot count across
    all simulations. The goal patch is static over time steps, so there is no
    stacking.

    Args:
        patch_dim: Side length of the square patch, in grid cells.
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
    def __copy__(self) -> GoalMap:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> GoalMap:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, patch_dim: typing.SupportsInt | typing.SupportsIndex, with_buffer: bool = False) -> None:
        """
        Args:
            patch_dim: Side length of the square patch, in grid cells.
            with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def patch_dim(self) -> int:
        """
        The configured patch side length, in grid cells.
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
