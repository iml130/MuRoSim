# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Primitive geometric shapes: triangle, rectangle, square.
"""
from __future__ import annotations
import murosimpy.geometry
import typing
__all__: list[str] = ['rectangle', 'square', 'triangle']
def rectangle(width: typing.SupportsFloat | typing.SupportsIndex, height: typing.SupportsFloat | typing.SupportsIndex, transform: murosimpy.geometry.Transform = ...) -> murosimpy.geometry.Shape:
    """
    Create a rectangle shape (four axis-aligned lines) centered on the origin, then apply transform.

    Args:
        width: Rectangle width, in metres
        height: Rectangle height, in metres
        transform: Transform to apply after construction (default: identity)
    """
def square(length: typing.SupportsFloat | typing.SupportsIndex, transform: murosimpy.geometry.Transform = ...) -> murosimpy.geometry.Shape:
    """
    Create a square shape (four axis-aligned lines) centered on the origin, then apply transform.

    Args:
        length: Side length, in metres
        transform: Transform to apply after construction (default: identity)
    """
def triangle(length: typing.SupportsFloat | typing.SupportsIndex, transform: murosimpy.geometry.Transform = ...) -> murosimpy.geometry.Shape:
    """
    Create an equilateral triangle shape (three lines) centered on its centroid, with one vertex pointing along +y, then apply transform.

    Args:
        length: Side length, in metres
        transform: Transform to apply after construction (default: identity)
    """
