# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
"""murosimpy.rendering: the offline (EGL) renderer.

Frame tests skip where no OpenGL ES context can be created (no EGL driver);
the argument checks run everywhere.
"""

import gc

import numpy as np
import pytest

import murosimpy
from conftest import build_simulation

RESOLUTION = 64
SAMPLES = 4
WHITE = 255


@pytest.fixture
def renderer():
    try:
        return murosimpy.rendering.OfflineRenderer(RESOLUTION, SAMPLES)
    except RuntimeError as error:
        pytest.skip(f"no OpenGL ES context: {error}")


def test_negative_samples_are_rejected():
    with pytest.raises(ValueError):
        murosimpy.rendering.OfflineRenderer(RESOLUTION, -1)


def test_frame_is_an_rgb_uint8_image(renderer, simulation):
    frame = renderer.render(simulation)
    assert frame.shape == (RESOLUTION, RESOLUTION, 3)
    assert frame.dtype == np.uint8
    assert 0 <= renderer.samples <= SAMPLES


def test_rendering_is_deterministic(renderer, simulation):
    np.testing.assert_array_equal(renderer.render(simulation), renderer.render(simulation))


def test_robots_change_the_frame(renderer, world_data):
    empty = renderer.render(murosimpy.core.Simulation(world_data))
    with_robots = renderer.render(build_simulation(world_data))
    assert empty.shape == with_robots.shape
    assert (empty == WHITE).mean() > 0.5  # an empty world is mostly background
    assert not np.array_equal(empty, with_robots)


def test_every_render_overload_returns_a_frame(renderer, world_data, simulation):
    simulations = murosimpy.containers.Simulations()
    simulations.add(simulation)
    targets = [simulation, simulations, simulation[0], world_data.create_shape(), world_data.create_world()]
    for target in targets:
        assert renderer.render(target).shape == (RESOLUTION, RESOLUTION, 3)
    assert renderer.render(world_data.create_world(), world_data).shape == (RESOLUTION, RESOLUTION, 3)


def test_renderer_outlives_the_scene(renderer, world_data):
    frame = renderer.render(build_simulation(world_data))
    gc.collect()
    assert frame.shape == (RESOLUTION, RESOLUTION, 3)
    assert renderer.render(build_simulation(world_data)).shape == frame.shape


def test_frame_rows_run_top_down(renderer, world_data):
    # An obstacle in the bottom half of the world must land in the bottom half
    # of the frame: rows are stored top row first, as on screen.
    size = max(line.x2 for line in world_data.lines)
    pillar = murosimpy.geometry.Circle(0.5 * size, 0.25 * size, 0.1 * size)
    data = murosimpy.io.DataContainer(list(world_data.lines), [pillar], [], "pillar")
    without = renderer.render(world_data.create_world())
    with_pillar = renderer.render(data.create_world())
    rows = np.nonzero((without != with_pillar).any(axis=(1, 2)))[0]
    assert rows.size > 0
    assert rows.min() > RESOLUTION // 2
