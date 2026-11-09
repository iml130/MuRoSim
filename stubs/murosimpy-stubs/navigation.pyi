# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Navigation nodes for goal specification and path planning.
"""
from __future__ import annotations
import collections.abc
import murosimpy.geometry
import typing
__all__: list[str] = ['Node', 'NodeContainer']
class Node:
    """
    A named navigation point: a circular area a robot can spawn at, drive
    through as a waypoint, or use as a goal.

    Carries a preferred start heading/bias and an optional group id for
    clustered start/goal placement. Angles are in radians. Node ids are
    assigned/used by NodeContainer; group ids are remapped to compact bucket
    indices when a NodeContainer is built from a list of nodes.
    """
    def __copy__(self) -> Node:
        """
        Support copy.copy(); returns an independent copy.
        """
    def __deepcopy__(self, memo: dict) -> Node:
        """
        Support copy.deepcopy(); returns an independent copy.
        """
    @typing.overload
    def __init__(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, radius: typing.SupportsFloat | typing.SupportsIndex, start_angle: typing.SupportsFloat | typing.SupportsIndex = 0.0, start_bias: typing.SupportsFloat | typing.SupportsIndex = 6.2831854820251465, id: typing.SupportsInt | typing.SupportsIndex = 0, group: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Create a navigation node from center coordinates and radius.

        Args:
            x: X coordinate of the center, in metres
            y: Y coordinate of the center, in metres
            radius: Node radius, in metres
            start_angle: Initial orientation angle in radians, wrapped into (-2*pi, 2*pi) (default: 0)
            start_bias: Angular range in radians around start_angle a robot may spawn within, clamped to [0, 2*pi] (default: 2*pi)
            id: Unique node identifier (default: 0)
            group: Group identifier for node clustering; 0 means ungrouped (default: 0)
        """
    @typing.overload
    def __init__(self, circle: murosimpy.geometry.Circle, start_angle: typing.SupportsFloat | typing.SupportsIndex = 0.0, start_bias: typing.SupportsFloat | typing.SupportsIndex = 6.2831854820251465, id: typing.SupportsInt | typing.SupportsIndex = 0, group: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Create a navigation node from a circle.

        Args:
            circle: Circle defining position and radius, in metres
            start_angle: Initial orientation angle in radians, wrapped into (-2*pi, 2*pi) (default: 0)
            start_bias: Angular range in radians around start_angle a robot may spawn within, clamped to [0, 2*pi] (default: 2*pi)
            id: Unique node identifier (default: 0)
            group: Group identifier for node clustering; 0 means ungrouped (default: 0)
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def rotate(self, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Rotate the node's circle around the origin by angle radians and advance the start angle by the same amount.
        """
    def transform(self, transform: murosimpy.geometry.Transform) -> None:
        """
        Apply an affine transform to the node's circle and keep the start angle in sync with the transform's rotation component.
        """
    def translate(self, dx: typing.SupportsFloat | typing.SupportsIndex, dy: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Translate the node's circle by (dx, dy), in metres.
        """
    @property
    def circle(self) -> murosimpy.geometry.Circle:
        """
        The node's area as a Circle.
        """
    @property
    def group(self) -> int:
        """
        Group identifier for node clustering (a compact bucket index once owned by a NodeContainer).
        """
    @property
    def id(self) -> int:
        """
        Unique node identifier.
        """
    @property
    def radius(self) -> float:
        """
        Node radius, in metres.
        """
    @property
    def start_angle(self) -> float:
        """
        Initial orientation angle, in radians.
        """
    @property
    def start_bias(self) -> float:
        """
        Angular range in radians around start_angle for random start orientations (in [0, 2*pi]).
        """
    @property
    def x(self) -> float:
        """
        X coordinate of the node center, in metres.
        """
    @property
    def y(self) -> float:
        """
        Y coordinate of the node center, in metres.
        """
class NodeContainer:
    """
    An owning, grouped collection of Nodes used as the set of possible
    spawn/waypoint/goal locations for a simulation.

    On construction, nodes are sorted by id and their (possibly sparse,
    arbitrary) group ids are validated and compacted into contiguous bucket
    indices: group 0 is the default/ungrouped bucket (any size), while any
    other group must contain at least 2 nodes. Node.group reflects the
    compacted bucket index after construction, not the original id passed in.
    """
    def __copy__(self) -> NodeContainer:
        """
        Support copy.copy(); returns an independent copy.
        """
    def __deepcopy__(self, memo: dict) -> NodeContainer:
        """
        Support copy.deepcopy(); returns an independent copy.
        """
    def __getitem__(self, index: typing.SupportsInt | typing.SupportsIndex) -> Node:
        """
        Get a copy of the node at the specified index (into the id-sorted node list).

        Args:
            index: Zero-based index into the container's nodes.

        Raises:
            IndexError: if index is out of range.
        """
    def __init__(self, nodes: collections.abc.Sequence[Node]) -> None:
        """
        Create a node container from a list of nodes (copied, then sorted by id and grouped).

        Args:
            nodes: List of Node objects

        Raises:
            ValueError: if a non-default group (id != 0) contains fewer than 2 nodes.
        """
    def __iter__(self) -> collections.abc.Iterator[Node]:
        """
        Iterate over independent copies of all nodes, in id-sorted order. The returned iterator keeps this container alive for its lifetime.
        """
    def __len__(self) -> int:
        """
        Return the number of nodes.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def node_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> Node:
        """
        Get a copy of the node at the specified index (into the id-sorted node list).

        Args:
            index: Zero-based index into the container's nodes.

        Raises:
            IndexError: if index is out of range.
        """
    @property
    def empty(self) -> bool:
        """
        Whether the container holds no nodes.
        """
    @property
    def nodes(self) -> list[Node]:
        """
        List of all nodes in the container, sorted by id.
        """
    @property
    def size(self) -> int:
        """
        Number of nodes in the container.
        """
