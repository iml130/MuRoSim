# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Geometric primitives, transformations, and world representation.
"""
from __future__ import annotations
import typing
from . import primitives
__all__: list[str] = ['Circle', 'Line', 'Shape', 'Transform', 'World', 'primitives']
class Transform:
    """
    2D affine transformation matrix for translating, rotating, and scaling shapes.
    """
    def __copy__(self) -> Transform:
        """
        Support copy.copy(); returns an independent copy.
        """
    def __deepcopy__(self, memo: dict) -> Transform:
        """
        Support copy.deepcopy(); returns an independent copy.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Create an identity transform.
        """
    @typing.overload
    def __init__(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Create a transform with translation and rotation.

        Args:
            x: Translation in x
            y: Translation in y
            angle: Rotation angle in radians
        """
    def __mul__(self, arg0: Transform) -> Transform:
        """
        Compose two transforms (matrix product); the right-hand transform is applied first.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def inverse(self) -> Transform:
        """
        Return the inverse of this transform.
        """
    def rotate(self, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Apply a rotation to this transform.
        """
    @typing.overload
    def scale(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Apply non-uniform scaling.
        """
    @typing.overload
    def scale(self, scale: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Apply uniform scaling.
        """
    def translate(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Apply a translation to this transform.
        """
class Line:
    """
    A 2D line segment defined by two endpoints, in metres.

    The basic straight-edge obstacle/wall primitive used throughout geometry,
    collision, grid and navigation code.
    """
    def __copy__(self) -> Line:
        """
        Support copy.copy(); returns an independent copy.
        """
    def __deepcopy__(self, memo: dict) -> Line:
        """
        Support copy.deepcopy(); returns an independent copy.
        """
    def __init__(self, x1: typing.SupportsFloat | typing.SupportsIndex, y1: typing.SupportsFloat | typing.SupportsIndex, x2: typing.SupportsFloat | typing.SupportsIndex, y2: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Create a line from (x1, y1) to (x2, y2), in metres.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def __rmul__(self, arg0: Transform) -> Line:
        """
        Return a new line with the affine transform applied to both endpoints (transform * line).
        """
    def __str__(self) -> str:
        """
        Return a human-readable "line(x1=.., y1=.., x2=.., y2=..)" representation.
        """
    def rotate(self, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Rotate both endpoints around the origin by angle radians.
        """
    def scale(self, factor: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Uniformly scale both endpoints about the origin by factor.
        """
    @typing.overload
    def transform(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Place at pose (x, y, angle): rotate about the origin, then translate (T * R). x and y are in metres, angle in radians.
        """
    @typing.overload
    def transform(self, transform: Transform) -> None:
        """
        Apply an affine transform to both endpoints.
        """
    def translate(self, dx: typing.SupportsFloat | typing.SupportsIndex, dy: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Translate both endpoints by (dx, dy), in metres.
        """
    @property
    def x1(self) -> float:
        """
        X coordinate of the first endpoint, in metres.
        """
    @property
    def x2(self) -> float:
        """
        X coordinate of the second endpoint, in metres.
        """
    @property
    def y1(self) -> float:
        """
        Y coordinate of the first endpoint, in metres.
        """
    @property
    def y2(self) -> float:
        """
        Y coordinate of the second endpoint, in metres.
        """
class Circle:
    """
    A circle defined by a center point and a radius (metres).

    The basic round obstacle/robot-shape primitive used throughout geometry,
    collision, grid and navigation code. The radius is always clamped to be
    non-negative.
    """
    def __copy__(self) -> Circle:
        """
        Support copy.copy(); returns an independent copy.
        """
    def __deepcopy__(self, memo: dict) -> Circle:
        """
        Support copy.deepcopy(); returns an independent copy.
        """
    def __init__(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, radius: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Create a circle at (x, y) with given radius, in metres. A negative radius is clamped to 0.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def __rmul__(self, arg0: Transform) -> Circle:
        """
        Return a new circle with the affine transform applied (transform * circle); see transform() for how the radius is scaled.
        """
    def __str__(self) -> str:
        """
        Return a human-readable "circle(x=.., y=.., r=..)" representation.
        """
    def rotate(self, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Rotate the circle's center around the origin by angle radians (the radius is unaffected).
        """
    def scale(self, factor: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Uniformly scale the center (about the origin) and the radius by factor.
        """
    @typing.overload
    def transform(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Place at pose (x, y, angle): rotate about the origin, then translate (T * R). x and y are in metres, angle in radians; the radius is unaffected.
        """
    @typing.overload
    def transform(self, transform: Transform) -> None:
        """
        Apply an affine transform to the center, and shrink the radius by the smaller of the transform's two axis scale factors so a non-uniform scale does not grow the circle.
        """
    def translate(self, dx: typing.SupportsFloat | typing.SupportsIndex, dy: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Translate the circle by (dx, dy), in metres.
        """
    @property
    def radius(self) -> float:
        """
        Circle radius in metres (always >= 0).
        """
    @property
    def x(self) -> float:
        """
        X coordinate of the center, in metres.
        """
    @property
    def y(self) -> float:
        """
        Y coordinate of the center, in metres.
        """
class Shape:
    """
    A composite 2D shape made of line segments and circles.

    Used to describe robot/obstacle footprints (e.g. built from the
    triangle/rectangle/square primitives or assembled piece by piece) that are
    later transformed into the world frame and turned into a World.
    """
    def __copy__(self) -> Shape:
        """
        Support copy.copy(); returns an independent copy.
        """
    def __deepcopy__(self, memo: dict) -> Shape:
        """
        Support copy.deepcopy(); returns an independent copy.
        """
    def __init__(self) -> None:
        """
        Create an empty shape.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def __rmul__(self, arg0: Transform) -> Shape:
        """
        Return a new shape with the affine transform applied to every line and circle (transform * shape).
        """
    def __str__(self) -> str:
        """
        Return a human-readable "Shape(lines={...}, circles={...})" representation.
        """
    @typing.overload
    def add_circle(self, circle: Circle) -> None:
        """
        Add a circle to this shape.
        """
    @typing.overload
    def add_circle(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, radius: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Add a circle at (x, y) with given radius, in metres.
        """
    @typing.overload
    def add_line(self, line: Line) -> None:
        """
        Add a line to this shape.
        """
    @typing.overload
    def add_line(self, x1: typing.SupportsFloat | typing.SupportsIndex, y1: typing.SupportsFloat | typing.SupportsIndex, x2: typing.SupportsFloat | typing.SupportsIndex, y2: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Add a line from (x1, y1) to (x2, y2), in metres.
        """
    def add_shape(self, shape: Shape) -> None:
        """
        Add all elements from another shape.
        """
    def circle_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> Circle:
        """
        Get a copy of the circle at the specified index.

        Args:
            index: Zero-based index into the shape's circles.

        Raises:
            IndexError: if index is out of range.
        """
    def line_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> Line:
        """
        Get a copy of the line at the specified index.

        Args:
            index: Zero-based index into the shape's lines.

        Raises:
            IndexError: if index is out of range.
        """
    def rotate(self, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Rotate every line and circle about the origin by angle radians.
        """
    def scale(self, factor: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Uniformly scale every line and circle about the origin by factor.
        """
    @typing.overload
    def transform(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, angle: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Place every line and circle at pose (x, y, angle): rotate about the origin, then translate (T * R). x and y are in metres, angle in radians.
        """
    @typing.overload
    def transform(self, transform: Transform) -> None:
        """
        Apply an affine transform to every line and circle.
        """
    def translate(self, dx: typing.SupportsFloat | typing.SupportsIndex, dy: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Translate every line and circle by (dx, dy), in metres.
        """
    @property
    def circle_count(self) -> int:
        """
        Number of circles in this shape.
        """
    @property
    def circles(self) -> list[Circle]:
        """
        List of all circles in this shape.
        """
    @property
    def line_count(self) -> int:
        """
        Number of lines in this shape.
        """
    @property
    def lines(self) -> list[Line]:
        """
        List of all lines in this shape.
        """
class World:
    """
    The static world geometry: all lines and circles making up the simulated environment.

    Built once from a Shape (or from explicit line/circle lists) and immutable
    afterwards. Also tracks the overall axis-aligned bounding box (in metres) of
    all geometry.
    """
    def __copy__(self) -> World:
        """
        Support copy.copy(); returns an independent copy.
        """
    def __deepcopy__(self, memo: dict) -> World:
        """
        Support copy.deepcopy(); returns an independent copy.
        """
    def __init__(self, shape: Shape) -> None:
        """
        Build the world from a shape's lines and circles.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    def circle_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> Circle:
        """
        Get a copy of the circle at the specified index.

        Args:
            index: Zero-based index into the world's circles.

        Raises:
            IndexError: if index is out of range.
        """
    def line_at(self, index: typing.SupportsInt | typing.SupportsIndex) -> Line:
        """
        Get a copy of the line at the specified index.

        Args:
            index: Zero-based index into the world's lines.

        Raises:
            IndexError: if index is out of range.
        """
    @property
    def aabb_x(self) -> tuple[float, float]:
        """
        X-axis bounds of the axis-aligned bounding box as (min, max), in metres.
        """
    @property
    def aabb_y(self) -> tuple[float, float]:
        """
        Y-axis bounds of the axis-aligned bounding box as (min, max), in metres.
        """
    @property
    def area_x(self) -> tuple[float, float]:
        """
        Deprecated alias for aabb_x.
        """
    @property
    def area_y(self) -> tuple[float, float]:
        """
        Deprecated alias for aabb_y.
        """
    @property
    def circle_count(self) -> int:
        """
        Number of circles in this world.
        """
    @property
    def height(self) -> float:
        """
        World height (y-axis extent of the bounding box), in metres.
        """
    @property
    def line_count(self) -> int:
        """
        Number of lines in this world.
        """
    @property
    def shape(self) -> Shape:
        """
        Rebuilds and returns a Shape containing all of this world's lines and circles.
        """
    @property
    def width(self) -> float:
        """
        World width (x-axis extent of the bounding box), in metres.
        """
