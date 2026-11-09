# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
File I/O for world data and data containers.
"""
from __future__ import annotations
import collections.abc
import murosimpy.geometry
import murosimpy.navigation
import typing
__all__: list[str] = ['DataContainer', 'load_world', 'save_world']
class DataContainer:
    """
    Plain-data holder for a world description: obstacle lines/circles plus
    navigation nodes, and the name it was loaded/created with.

    This is the serialization-neutral form produced by load_world() and
    consumed to build a Simulation. Use create_world()/create_shape()/
    create_node_container() to convert it into the runtime geometry/navigation
    types used by the simulator.
    """
    @typing.overload
    def __init__(self, lines: collections.abc.Sequence[murosimpy.geometry.Line], circles: collections.abc.Sequence[murosimpy.geometry.Circle], nodes: collections.abc.Sequence[murosimpy.navigation.Node], name: str) -> None:
        """
        Create a DataContainer from lists of lines, circles, and nodes.

        Args:
            lines: List of Line objects
            circles: List of Circle objects
            nodes: List of Node objects
            name: Container name
        """
    @typing.overload
    def __init__(self, shape: murosimpy.geometry.Shape, nodes: collections.abc.Sequence[murosimpy.navigation.Node], name: str) -> None:
        """
        Create a DataContainer from a Shape and list of nodes.

        Args:
            shape: Shape containing lines and circles
            nodes: List of Node objects
            name: Container name
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def circle_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> murosimpy.geometry.Circle:
        """
        Get a copy of the circle at the specified index.

        Args:
            index: Zero-based index into the container's circles.

        Raises:
            IndexError: if index is out of range.
        """
    def create_node_container(self) -> murosimpy.navigation.NodeContainer:
        """
        Wrap the stored node list in a NodeContainer (sorts by id and builds group indices).
        """
    def create_shape(self) -> murosimpy.geometry.Shape:
        """
        Build a composite Shape from the stored lines and circles.
        """
    def create_world(self) -> murosimpy.geometry.World:
        """
        Build the runtime World geometry from the stored lines and circles.
        """
    def line_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> murosimpy.geometry.Line:
        """
        Get a copy of the line at the specified index.

        Args:
            index: Zero-based index into the container's lines.

        Raises:
            IndexError: if index is out of range.
        """
    def node_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> murosimpy.navigation.Node:
        """
        Get a copy of the node at the specified index.

        Args:
            index: Zero-based index into the container's nodes.

        Raises:
            IndexError: if index is out of range.
        """
    @property
    def circle_count(self) -> int:
        """
        Number of circles in the container.
        """
    @property
    def circles(self) -> list[murosimpy.geometry.Circle]:
        """
        List of all circles in the container.
        """
    @property
    def line_count(self) -> int:
        """
        Number of lines in the container.
        """
    @property
    def lines(self) -> list[murosimpy.geometry.Line]:
        """
        List of all lines in the container.
        """
    @property
    def name(self) -> str:
        """
        Container name identifier.
        """
    @property
    def node_count(self) -> int:
        """
        Number of nodes in the container.
        """
    @property
    def nodes(self) -> list[murosimpy.navigation.Node]:
        """
        List of all nodes in the container.
        """
def load_world(filename: str, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, mirror_x: bool = False, mirror_y: bool = False) -> DataContainer:
    """
    Load world data from a file, named after the file's stem.

    Args:
        filename: Input file path; must have a ".world" extension
        scale: Scale factor applied to every line, circle and node after loading (default: 1.0)
        mirror_x: Mirror along the X axis (negates the X scale factor, default: False)
        mirror_y: Mirror along the Y axis (negates the Y scale factor, default: False)

    Returns:
        DataContainer with the loaded world data.

    Raises:
        RuntimeError: if filename does not end in ".world", the file cannot be opened, any line fails to parse or validate, or the file contains no lines, circles or nodes at all.
    """
@typing.overload
def save_world(filename: str, world: murosimpy.geometry.World, nodes: murosimpy.navigation.NodeContainer) -> None:
    """
    Save world geometry and nodes to a file.

    Args:
        filename: Output file path
        world: World geometry to save
        nodes: Navigation nodes to save

    Raises:
        RuntimeError: if the file could not be written.
    """
@typing.overload
def save_world(filename: str, shape: murosimpy.geometry.Shape, nodes: murosimpy.navigation.NodeContainer) -> None:
    """
    Save shape geometry and nodes to a file.

    Args:
        filename: Output file path
        shape: Shape geometry to save
        nodes: Navigation nodes to save

    Raises:
        RuntimeError: if the file could not be written.
    """
@typing.overload
def save_world(filename: str, data: DataContainer) -> None:
    """
    Save a DataContainer to a file.

    Args:
        filename: Output file path
        data: DataContainer to save

    Raises:
        RuntimeError: if the file could not be written.
    """
