# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
"""murosimprocpy: ranges, generators and the factory."""

import numpy as np
import pytest

import murosimprocpy
import murosimpy

MIN_NODE_COUNT = 5
NODE_RADIUS = 0.25
SEED = 42


def rr(*args):
    return murosimprocpy.RandomRange(*args)


GENERATORS = {
    "maze": lambda: murosimprocpy.MazeGenerator(MIN_NODE_COUNT, NODE_RADIUS, rr(10.0, 12.0), rr(0.7, 1.25),
                                                rr(0.1, 0.3), rr(0.0, 0.3)),
    "graph": lambda: murosimprocpy.GraphGenerator(MIN_NODE_COUNT, NODE_RADIUS, rr(10.0, 12.0), rr(0.7, 1.25),
                                                  rr(1.5, 4.0), rr(0.05, 0.75), rr(0.05, 0.75)),
    "random_sparse": lambda: murosimprocpy.RandomSparseGenerator(MIN_NODE_COUNT, NODE_RADIUS, rr(10.0, 12.0),
                                                                 rr(0.7, 1.25)),
    "wfc": lambda: murosimprocpy.WFCGenerator(MIN_NODE_COUNT, NODE_RADIUS, rr(12.0, 14.0), rr(0.7, 1.25),
                                              murosimprocpy.PresetType.BLOCK, rr(0.3, 1.5), rr(0.3, 0.7),
                                              murosimprocpy.ShapeDistribution()),
}


def node_positions(data):
    return np.array([[node.x, node.y] for node in data.nodes])


# ------------------------------------------------------------ random range

def test_random_range_constructors():
    assert (rr(0.5).min, rr(0.5).max) == (0.5, 0.5)
    assert (rr(0.7, 1.25).min, rr(0.7, 1.25).max) == pytest.approx((0.7, 1.25))
    assert rr((1.0, 2.0)).max == 2.0
    with pytest.raises(ValueError):
        rr(2.0, 1.0)


def test_shape_distribution_defaults_and_setters():
    shape = murosimprocpy.ShapeDistribution()
    assert (shape.circle.min, shape.polygon.min, shape.decompose.min) == pytest.approx((0.5, 0.5, 0.3))
    shape.polygon = rr(0.0)
    assert shape.polygon.max == 0.0
    mapped = murosimprocpy.RandomSparseGenerator.entropy_to_shape_distribution(rr(0.4, 0.6))
    assert (mapped.polygon.min, mapped.polygon.max) == pytest.approx((0.2, 0.3))


# -------------------------------------------------------------- generators

@pytest.mark.parametrize("make", GENERATORS.values(), ids=GENERATORS.keys())
def test_generator_yields_a_usable_world(make):
    generator = make()
    data = generator(SEED)
    assert isinstance(data, murosimpy.io.DataContainer)
    assert data.line_count >= 4
    assert data.node_count >= generator.min_node_count == MIN_NODE_COUNT
    assert generator.node_radius == pytest.approx(NODE_RADIUS)
    world = data.create_world()
    positions = node_positions(data)
    assert np.all(positions[:, 0] >= world.aabb_x[0]) and np.all(positions[:, 0] <= world.aabb_x[1])
    assert np.all(positions[:, 1] >= world.aabb_y[0]) and np.all(positions[:, 1] <= world.aabb_y[1])
    simulation = murosimpy.core.Simulation(data)
    assert simulation.max_robots == data.node_count


def test_same_seed_gives_the_same_world():
    generator = GENERATORS["maze"]()
    first, second = generator(SEED), generator(SEED)
    assert first.name == second.name
    assert first.line_count == second.line_count
    np.testing.assert_array_equal(node_positions(first), node_positions(second))
    assert generator(SEED + 1).name != first.name


def test_generator_configuration_is_exposed():
    generator = GENERATORS["maze"]()
    assert (generator.world_size.min, generator.world_size.max) == pytest.approx((10.0, 12.0))
    assert (generator.spacing.min, generator.spacing.max) == pytest.approx((0.7, 1.25))
    assert generator.wall_thickness.max == pytest.approx(0.3)
    assert generator.wall_removal_rate.max == pytest.approx(0.3)


def test_invalid_configuration_raises_value_error():
    with pytest.raises(ValueError):
        murosimprocpy.MazeGenerator(1, NODE_RADIUS, rr(10.0, 12.0), rr(0.7, 1.25), rr(0.1, 0.3))
    with pytest.raises(ValueError):
        murosimprocpy.GraphGenerator(MIN_NODE_COUNT, NODE_RADIUS, rr(10.0, 12.0), rr(0.7, 1.25), rr(1.5, 4.0),
                                     rr(0.0, 1.5), rr(0.0, 0.5))
    with pytest.raises(ValueError):
        murosimprocpy.WFCGenerator(MIN_NODE_COUNT, NODE_RADIUS, rr(8.0), rr(1.0), murosimprocpy.PresetType.BLOCK,
                                   rr(0.3, 1.5), rr(0.3, 0.7), murosimprocpy.ShapeDistribution())


def test_preset_type_has_eight_members():
    assert len(murosimprocpy.PresetType.__members__) == 8
    assert murosimprocpy.PresetType.LINES == murosimprocpy.PresetType(7)


# ----------------------------------------------------------------- factory

def test_factory_generates_data_containers_and_simulations():
    factory = murosimprocpy.Factory(2)
    factory.add(GENERATORS["maze"](), 1.0)
    factory.add(GENERATORS["random_sparse"]())
    assert factory.num_threads == 2 and factory.max_retries == 100
    containers = factory.generate_data_containers(4)
    assert len(containers) == 4
    assert all(container.node_count >= MIN_NODE_COUNT for container in containers)
    assert all(container.name.split("_")[0] in ("maze", "random") for container in containers)
    simulations = factory.generate_simulations(3, grid_scale=0.1, buffer_scale=0.2)
    assert isinstance(simulations, murosimpy.containers.Simulations)
    assert len(simulations) == 3
    for simulation in simulations:
        assert simulation.max_robots >= MIN_NODE_COUNT
        assert simulation.get_map().shape[0] > 100


def test_empty_factory_raises():
    with pytest.raises(RuntimeError):
        murosimprocpy.Factory().generate_data_containers(1)
    with pytest.raises(ValueError):
        murosimprocpy.Factory(0, 0)
