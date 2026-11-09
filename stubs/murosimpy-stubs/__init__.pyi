# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Python bindings for MuRoSim - Multi-Robot Simulation.

MuRoSim provides efficient simulation of multiple robots with
lidar sensors, collision detection, and path planning capabilities.
"""
from __future__ import annotations
from . import definitions
from . import core
from . import geometry
from . import navigation
from . import models
from . import containers
from . import rendering
from . import io
__all__: list[str] = ['containers', 'core', 'definitions', 'geometry', 'io', 'models', 'navigation', 'rendering']
__version__: str = '1.0.0'
