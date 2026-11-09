# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
"""The three extension modules load together and agree on their version."""

import re

import murosimpy
import murosimprocpy
import murosimrlpy

MODULES = [murosimpy, murosimrlpy, murosimprocpy]


def test_versions_are_equal_and_well_formed():
    versions = {module.__version__ for module in MODULES}
    assert len(versions) == 1
    assert re.fullmatch(r"\d+\.\d+\.\d+", murosimpy.__version__)


def test_murosimprocpy_exports_its_generators():
    for name in ["Factory", "Generator", "GraphGenerator", "MazeGenerator", "RandomSparseGenerator", "WFCGenerator",
                 "RandomRange", "ShapeDistribution", "PresetType"]:
        assert hasattr(murosimprocpy, name), name
    assert murosimprocpy.BLOCK == murosimprocpy.PresetType.BLOCK


def test_murosimpy_submodules_are_reachable():
    for name in ["containers", "core", "definitions", "geometry", "io", "models", "navigation", "rendering"]:
        assert hasattr(murosimpy, name), name
    assert hasattr(murosimpy.geometry, "primitives")
    assert hasattr(murosimpy.models, "world")
    assert hasattr(murosimpy.models, "robot")


def test_murosimrlpy_submodules_are_reachable():
    assert hasattr(murosimrlpy.observation, "robot")
    assert hasattr(murosimrlpy.observation, "robot_simulation")
    assert hasattr(murosimrlpy.observation, "simulation")
    assert hasattr(murosimrlpy.reward, "robot")


def test_cross_module_types_are_shared():
    # murosimprocpy returns murosimpy types, which only works because it
    # imports murosimpy itself; the user's import order must not matter.
    generator = murosimprocpy.MazeGenerator(
        5, 0.25, murosimprocpy.RandomRange(10.0, 12.0), murosimprocpy.RandomRange(0.7, 1.25),
        murosimprocpy.RandomRange(0.1, 0.3))
    data = generator(42)
    assert isinstance(data, murosimpy.io.DataContainer)
    assert isinstance(data.node_at(0), murosimpy.navigation.Node)
