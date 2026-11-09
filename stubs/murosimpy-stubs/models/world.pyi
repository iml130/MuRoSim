# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Factory functions for creating predefined world geometries.
"""
from __future__ import annotations
import murosimpy.io
import typing
__all__: list[str] = ['bone', 'curve', 'empty', 'intersection', 'line', 'roundabout']
def bone(world_size: typing.SupportsFloat | typing.SupportsIndex = 5.0, space: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> murosimpy.io.DataContainer:
    """
    Create a bone-shaped world with two open areas connected by a corridor.

    Args:
        world_size: Overall extent of the world, in metres; must be positive (default: 5.0)
        space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)

    Returns:
        DataContainer with world geometry.

    Raises:
        ValueError: if world_size or space is not positive, or space exceeds world_size.
    """
def curve(world_size: typing.SupportsFloat | typing.SupportsIndex = 5.0, space: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> murosimpy.io.DataContainer:
    """
    Create a curved corridor world.

    Args:
        world_size: Overall extent of the world, in metres; must be positive (default: 5.0)
        space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)

    Returns:
        DataContainer with world geometry.

    Raises:
        ValueError: if world_size or space is not positive, or space exceeds world_size.
    """
def empty(world_size: typing.SupportsFloat | typing.SupportsIndex = 5.0, space: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> murosimpy.io.DataContainer:
    """
    Create an empty rectangular world with only the outer boundary, no internal obstacles.

    Args:
        world_size: Overall extent of the world, in metres; must be positive (default: 5.0)
        space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)

    Returns:
        DataContainer with world geometry.

    Raises:
        ValueError: if world_size or space is not positive, or space exceeds world_size.
    """
def intersection(world_size: typing.SupportsFloat | typing.SupportsIndex = 5.0, space: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> murosimpy.io.DataContainer:
    """
    Create a four-way intersection world.

    Args:
        world_size: Overall extent of the world, in metres; must be positive (default: 5.0)
        space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)

    Returns:
        DataContainer with world geometry.

    Raises:
        ValueError: if world_size or space is not positive, or space exceeds world_size.
    """
def line(world_size: typing.SupportsFloat | typing.SupportsIndex = 5.0, space: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> murosimpy.io.DataContainer:
    """
    Create a simple corridor world.

    Args:
        world_size: Overall extent of the world, in metres; must be positive (default: 5.0)
        space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)

    Returns:
        DataContainer with world geometry.

    Raises:
        ValueError: if world_size or space is not positive, or space exceeds world_size.
    """
def roundabout(world_size: typing.SupportsFloat | typing.SupportsIndex = 5.0, space: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> murosimpy.io.DataContainer:
    """
    Create a roundabout world.

    Args:
        world_size: Overall extent of the world, in metres; must be positive (default: 5.0)
        space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)

    Returns:
        DataContainer with world geometry.

    Raises:
        ValueError: if world_size or space is not positive, or space exceeds world_size.
    """
