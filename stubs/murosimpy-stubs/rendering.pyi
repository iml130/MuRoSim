# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Rendering and visualization tools for offline and online display.
"""
from __future__ import annotations
import collections.abc
import murosimpy.containers
import murosimpy.core
import murosimpy.geometry
import murosimpy.io
import numpy
import numpy.typing
import typing
__all__: list[str] = ['OfflineRenderer', 'OnlineRenderer']
class OfflineRenderer:
    """
    Headless renderer using an EGL pbuffer surface and OpenGL ES 3, with no
    visible window.

    Prefers the surfaceless EGL platform where available (typical of CI and
    server environments without a display), falling back to the default EGL
    display otherwise; needs no X11/Wayland/GLFW window. Use OnlineRenderer
    instead for an interactive on-screen window. A renderer instance owns one GL
    context and must only be driven from one thread at a time.
    """
    def __init__(self, resolution: typing.SupportsInt | typing.SupportsIndex = 600, samples: typing.SupportsInt | typing.SupportsIndex = 8) -> None:
        """
        Create an offline renderer and its EGL context.

        Args:
            resolution: Square output image resolution in pixels (default: 600)
            samples: Multisampling sample count for anti-aliasing, 0 disables it
                (default: 8), clamped to the driver's maximum (Mesa's CPU renderer
                llvmpipe supports 4). See the samples property for the count in use.

        Raises:
            ValueError: if resolution is not positive or samples is negative.
            RuntimeError: if EGL display/context/surface creation fails.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    @typing.overload
    def render(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a simulation and return the image as a numpy array.

        Args:
            simulation: Simulation to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render multiple simulations in a grid and return the image as a numpy array.

        Args:
            simulations: List of simulations to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a robot and return the image as a numpy array.

        Args:
            robot: Robot to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, shape: murosimpy.geometry.Shape) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a shape and return the image as a numpy array.

        Args:
            shape: Shape to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, world: murosimpy.geometry.World) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a world and return the image as a numpy array.

        Args:
            world: World to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, world: murosimpy.geometry.World, data: murosimpy.io.DataContainer) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a DataContainer and return the image as a numpy array.

        Args:
            world: World (unused, for API compatibility)
            data: DataContainer to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @property
    def samples(self) -> int:
        """
        Multisampling sample count actually in use (0 = no anti-aliasing).
        """
class OnlineRenderer:
    """
    On-screen renderer backed by a GLFW window and OpenGL ES 3, for interactive
    real-time display.

    Requires a display/windowing system to be available (unlike OfflineRenderer).
    GLFW is initialized process-wide on first use; vsync is disabled. Each
    render() call redraws and immediately presents the frame (swaps buffers and
    polls window/input events) before returning the image. A renderer instance
    owns one GL context/window and must only be driven from one thread at a
    time.
    """
    def __init__(self, resolution: typing.SupportsInt | typing.SupportsIndex = 600, samples: typing.SupportsInt | typing.SupportsIndex = 8) -> None:
        """
        Create an online renderer, its window, and its GL context.

        Args:
            resolution: Square window resolution in pixels (default: 600)
            samples: Multisampling sample count for anti-aliasing, 0 disables it
                (default: 8), clamped to the driver's maximum. See the samples property.

        Raises:
            ValueError: if resolution is not positive or samples is negative.
            RuntimeError: if the GLFW window cannot be created.
        """
    def __repr__(self) -> str:
        """
        Return a debug string representation.
        """
    @typing.overload
    def render(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a simulation to window and return the image.

        Args:
            simulation: Simulation to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render multiple simulations in a grid layout to window and return the image.

        Args:
            simulations: List of simulations to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a robot to window and return the image.

        Args:
            robot: Robot to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, shape: murosimpy.geometry.Shape) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a shape to window and return the image.

        Args:
            shape: Shape to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, world: murosimpy.geometry.World) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a world to window and return the image.

        Args:
            world: World to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, world: murosimpy.geometry.World, data: murosimpy.io.DataContainer) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render a DataContainer to window and return the image.

        Args:
            world: World (unused, for API compatibility)
            data: DataContainer to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    @typing.overload
    def render(self, containers: collections.abc.Sequence[murosimpy.io.DataContainer]) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Render multiple DataContainers in a grid layout to window and return the image.

        Args:
            containers: Vector of DataContainers to render

        Returns:
            numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.
        """
    def wait_key(self, timeout_ms: typing.SupportsInt | typing.SupportsIndex = 0) -> int:
        """
        Wait for a key press in the renderer window (like cv2.waitKey).

        Args:
            timeout_ms: Timeout in milliseconds. 0 = wait forever (default).

        Returns:
            GLFW key code of the pressed key, or -1 on timeout/window close.
        """
    @property
    def is_open(self) -> bool:
        """
        False once the user has closed the window.
        """
    @property
    def samples(self) -> int:
        """
        Multisampling sample count actually in use (0 = no anti-aliasing).
        """
