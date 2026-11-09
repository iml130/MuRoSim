# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Python bindings for MuRoSimProc.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.io
import typing
__all__: list[str] = ['BLOCK', 'CAVERN', 'Factory', 'Generator', 'GraphGenerator', 'LABYRINTH', 'LINES', 'MazeGenerator', 'OBSTACLE', 'PresetType', 'ROOM', 'RandomRange', 'RandomSparseGenerator', 'SPARSE', 'ShapeDistribution', 'WAREHOUSE', 'WFCGenerator']
class RandomRange:
    """
    A closed interval [min, max] used throughout the generators to describe either a fixed
    value (min == max) or a range a parameter is drawn uniformly from at generation time
    (domain randomization).

    A plain float or a 1- or 2-element tuple is implicitly convertible to a RandomRange
    wherever one is expected, so most constructors can be called with e.g. ``0.5`` or
    ``(0.5, 1.5)`` directly.
    """
    @typing.overload
    def __init__(self, value: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Constructs a fixed (non-random) value: min == max == value.
        """
    @typing.overload
    def __init__(self, min: typing.SupportsFloat | typing.SupportsIndex, max: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Constructs a range [min, max] to draw uniformly from.

        Args:
            min: Lower bound.
            max: Upper bound; must be >= min.

        Raises:
            ValueError: If max < min.
        """
    @typing.overload
    def __init__(self, range: tuple) -> None:
        """
        Constructs a RandomRange from a 1-element (fixed value) or 2-element (min, max) tuple.

        Raises:
            ValueError: If the tuple does not have 1 or 2 elements, or if max < min.
        """
    def __repr__(self) -> str:
        """
        Return a readable representation, e.g. RandomRange(1.0, 2.5) or RandomRange(1.0) for a fixed value.
        """
    @property
    def max(self) -> float:
        """
        The upper bound of the range.
        """
    @property
    def min(self) -> float:
        """
        The lower bound of the range.
        """
class Generator:
    """
    Abstract base class for all world generators (maze, graph, line, intersection,
    random-sparse, WFC, ...).

    A generator turns a random seed into obstacle geometry (lines/circles) plus navigation
    nodes, retrying with different seeds internally if the requested minimum node count is
    not reached. This class cannot be instantiated directly; use one of its subclasses.
    """
    def __call__(self, seed: typing.SupportsInt | typing.SupportsIndex) -> murosimpy.io.DataContainer:
        """
        Generates a world for the given seed.

        If the first attempt yields fewer than `min_node_count` nodes, retries with
        seed + 1, seed + 2, ... up to 100 attempts total before giving up. Generation
        is deterministic: calling this again with the same seed reproduces the exact
        same lines, circles and nodes.

        Args:
            seed: Random seed to generate the world from.

        Returns:
            A DataContainer holding the generated lines, circles and nodes, named
            from the generator's name and the seed.

        Raises:
            RuntimeError: If no attempt reaches the minimum node count within 100 tries.
        """
    @property
    def min_node_count(self) -> int:
        """
        The minimum number of navigation nodes a generated world must contain.
        """
    @property
    def node_radius(self) -> float:
        """
        The configured navigation node radius, in metres.
        """
    @property
    def spacing(self) -> RandomRange:
        """
        The configured node-spacing range, in metres, as a RandomRange.
        """
    @property
    def world_size(self) -> RandomRange:
        """
        The configured world-size range, in metres, as a RandomRange (worlds are square).
        """
class MazeGenerator(Generator):
    """
    Generates perfect-maze worlds using Prim's algorithm on a grid, then randomly punches
    holes in walls to create loops, converts the result to obstacle geometry and places
    navigation nodes, and finally scales everything to the sampled world size.

    Deterministic for a given seed (aside from the retry loop in the base Generator, which
    advances the seed if the minimum node count is not met).
    """
    def __init__(self, min_node_count: typing.SupportsInt | typing.SupportsIndex, node_radius: typing.SupportsFloat | typing.SupportsIndex, world_size: RandomRange, spacing: RandomRange, wall_thickness: RandomRange, wall_removal_rate: RandomRange = 0.0) -> None:
        """
        Constructs a maze generator.

        Args:
            min_node_count: Minimum number of navigation nodes a generated world must contain.
            node_radius: Radius of navigation nodes, in metres.
            world_size: Side length of the (square) world, in metres.
            spacing: Minimum desired spacing between navigation nodes, in metres.
            wall_thickness: Thickness of maze walls, in metres.
            wall_removal_rate: Fraction in [0, 1] of interior walls to additionally remove
                after maze generation, creating loops; 0 (default) yields a perfect maze
                (no loops).

        Raises:
            ValueError: If the shared configuration is infeasible (e.g. spacing smaller
                than twice the node radius).
            RuntimeError: If the worst-case maze grid (smallest world, largest spacing and
                wall thickness) would be smaller than the minimum viable maze size.
        """
    @property
    def wall_removal_rate(self) -> RandomRange:
        """
        The configured wall-removal-rate range ([0, 1]).
        """
    @property
    def wall_thickness(self) -> RandomRange:
        """
        The configured wall-thickness range, in metres.
        """
class ShapeDistribution:
    """
    Configuration for obstacle shape selection probabilities.

    `circle` and `polygon` are relative weights (normalized internally); obstacles with a
    radius below `min_polygon_radius` are always rendered as circles regardless of these
    weights.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructs a ShapeDistribution with default weights (circle=0.5, polygon=0.5, decompose=0.3).
        """
    @typing.overload
    def __init__(self, circle: RandomRange, polygon: RandomRange, decompose: RandomRange) -> None:
        """
        Constructs a ShapeDistribution with explicit circle/polygon/decompose ranges. `min_polygon_radius` keeps its default (0.05).
        """
    def __repr__(self) -> str:
        """
        Return a readable representation listing the circle, polygon, decompose ranges and min_polygon_radius.
        """
    @property
    def circle(self) -> RandomRange:
        """
        Relative weight (range) for selecting a circle shape.
        """
    @circle.setter
    def circle(self, arg0: RandomRange) -> None:
        ...
    @property
    def decompose(self) -> RandomRange:
        """
        Relative weight (range) for decomposing a shape into simpler parts.
        """
    @decompose.setter
    def decompose(self, arg0: RandomRange) -> None:
        ...
    @property
    def polygon(self) -> RandomRange:
        """
        Relative weight (range) for selecting a polygon shape.
        """
    @polygon.setter
    def polygon(self, arg0: RandomRange) -> None:
        ...
class RandomSparseGenerator(Generator):
    """
    Generates worlds with obstacles scattered randomly across open space (no walls or
    corridors), placed without overlap up to a density-dependent number of attempts, plus
    navigation nodes.

    Deterministic for a given seed (aside from the retry loop in the base Generator, which
    advances the seed if the minimum node count is not met).
    """
    @staticmethod
    def entropy_to_shape_distribution(entropy: RandomRange) -> ShapeDistribution:
        """
        Converts a shape-entropy range in [0, 1] to a ShapeDistribution (circle weight = entropy, polygon weight = (1 - entropy) / 2, decompose weight = entropy / 2).
        """
    @typing.overload
    def __init__(self, min_node_count: typing.SupportsInt | typing.SupportsIndex, node_radius: typing.SupportsFloat | typing.SupportsIndex, world_size: RandomRange, spacing: RandomRange, obstacle_size: RandomRange, placement_density_rate: RandomRange, shape_distribution: ShapeDistribution) -> None:
        """
        Constructs a random-sparse generator with an explicit shape distribution.

        Args:
            min_node_count: Minimum number of navigation nodes a generated world must contain.
            node_radius: Radius of navigation nodes, in metres.
            world_size: Side length of the (square) world, in metres.
            spacing: Minimum desired spacing between navigation nodes, in metres.
            obstacle_size: Size range for placed obstacles, in metres; must be > 0.
            placement_density_rate: Controls how many placement attempts are made relative
                to world size before giving up on filling further obstacles.
            shape_distribution: Relative weights and thresholds used to choose obstacle shapes.

        Raises:
            ValueError: If the configuration is infeasible (e.g. obstacle_size <= 0, or a
                negative shape-distribution weight).
        """
    @typing.overload
    def __init__(self, min_node_count: typing.SupportsInt | typing.SupportsIndex, node_radius: typing.SupportsFloat | typing.SupportsIndex, world_size: RandomRange, spacing: RandomRange, obstacle_size: RandomRange = ..., placement_density_rate: RandomRange = 1.0, shape_entropy: RandomRange = ...) -> None:
        """
        Constructs a random-sparse generator, deriving the shape distribution from a single
        shape_entropy value via `entropy_to_shape_distribution` instead of specifying weights
        directly.

        Args:
            min_node_count: Minimum number of navigation nodes a generated world must contain.
            node_radius: Radius of navigation nodes, in metres.
            world_size: Side length of the (square) world, in metres.
            spacing: Minimum desired spacing between navigation nodes, in metres.
            obstacle_size: Size range for placed obstacles, in metres; must be > 0.
            placement_density_rate: Controls how many placement attempts are made relative
                to world size before giving up on filling further obstacles.
            shape_entropy: Range in [0, 1] converted to circle/polygon/decompose weights via
                `entropy_to_shape_distribution`.

        Raises:
            ValueError: If the configuration is infeasible (e.g. obstacle_size <= 0).
        """
    @property
    def obstacle_size(self) -> RandomRange:
        """
        The configured obstacle-size range, in metres.
        """
    @property
    def placement_density_rate(self) -> RandomRange:
        """
        The configured placement-density-rate range.
        """
    @property
    def shape_distribution(self) -> ShapeDistribution:
        """
        The configured obstacle shape distribution.
        """
class GraphGenerator(Generator):
    """
    Generates corridor-network worlds: Poisson-disk sampled points are triangulated
    (Delaunay), randomly thinned, converted into rounded-corner corridor walls ("tubes"),
    and navigation nodes are placed at the surviving graph vertices.

    Deterministic for a given seed (aside from the retry loop in the base Generator, which
    advances the seed if the minimum node count is not met).
    """
    def __init__(self, min_node_count: typing.SupportsInt | typing.SupportsIndex, node_radius: typing.SupportsFloat | typing.SupportsIndex, world_size: RandomRange, spacing: RandomRange, edge_length: RandomRange, point_removal_rate: RandomRange, edge_removal_rate: RandomRange, tube_num_buckets: typing.SupportsInt | typing.SupportsIndex = 5, tube_arc_segments: typing.SupportsInt | typing.SupportsIndex = 8, tube_simplify_tolerance: typing.SupportsFloat | typing.SupportsIndex = 0.0) -> None:
        """
        Constructs a graph-network generator.

        Args:
            min_node_count: Minimum number of navigation nodes a generated world must contain.
            node_radius: Radius of navigation nodes, in metres.
            world_size: Side length of the (square) world, in metres.
            spacing: Minimum desired spacing between navigation nodes, in metres.
            edge_length: Min/max spacing, in metres, used both for the Poisson-disk point
                sampling radius and as the minimum allowed edge length.
            point_removal_rate: Fraction in [0, 1] of sampled points to discard before
                triangulation (subject to a floor of 2 * min_node_count surviving points).
            edge_removal_rate: Fraction in [0, 1] of Delaunay edges to randomly remove
                afterwards; low values give a dense graph, high values a sparse one.
            tube_num_buckets: Number of spatial buckets used when merging corridor-wall
                polygons; must be > 0.
            tube_arc_segments: Number of segments used to approximate rounded corridor
                corners; must be > 0.
            tube_simplify_tolerance: Tolerance, in metres, for simplifying the resulting
                corridor outlines; must be >= 0, 0 disables simplification.

        Raises:
            ValueError: If the configuration is infeasible (e.g. removal rates outside
                [0, 1], zero buckets/segments, or a world too small to fit the requested
                edge_length).
        """
class PresetType:
    """
    Built-in tile presets for WfcGenerator, each producing a different style of world.

    Members:

      BLOCK : Balanced layout with walls and obstacles

      OBSTACLE : Open space with scattered obstacle clusters

      LABYRINTH : Dense maze with narrow corridors

      WAREHOUSE : Mixed walls with obstacle zones

      ROOM : Connected rooms with doorways

      SPARSE : Minimal walls, scattered L-shaped obstacles

      CAVERN : Organic thick walls with obstacle pockets

      LINES : Parallel wall and obstacle lines
    """
    BLOCK: typing.ClassVar[PresetType]
    CAVERN: typing.ClassVar[PresetType]
    LABYRINTH: typing.ClassVar[PresetType]
    LINES: typing.ClassVar[PresetType]
    OBSTACLE: typing.ClassVar[PresetType]
    ROOM: typing.ClassVar[PresetType]
    SPARSE: typing.ClassVar[PresetType]
    WAREHOUSE: typing.ClassVar[PresetType]
    __members__: typing.ClassVar[dict[str, PresetType]]
    @typing.overload
    def __eq__(self, other: PresetType) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: PresetType) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class WFCGenerator(Generator):
    """
    Generates worlds via Wave Function Collapse (WFC): a tile-pattern grid is collapsed
    into a layout of empty/wall/obstacle cells, then converted to obstacle geometry and
    navigation nodes, additionally scattering extra small obstacles.

    Deterministic for a given seed (aside from the retry loop in the base Generator); if
    WFC hits a contradiction, that attempt silently produces an empty world and the retry
    loop tries the next seed.
    """
    def __init__(self, min_node_count: typing.SupportsInt | typing.SupportsIndex, node_radius: typing.SupportsFloat | typing.SupportsIndex, world_size: RandomRange, spacing: RandomRange, preset_type: PresetType, obstacle_size: RandomRange, obstacle_density: RandomRange, shape_distribution: ShapeDistribution) -> None:
        """
        Constructs a WFC generator using a named built-in tile preset.

        Args:
            min_node_count: Minimum number of navigation nodes a generated world must contain.
            node_radius: Radius of navigation nodes, in metres.
            world_size: Side length of the (square) world, in metres.
            spacing: Minimum desired spacing between navigation nodes, in metres.
            preset_type: Built-in tile preset (see PresetType) determining the style of
                the generated layout.
            obstacle_size: Size range, in metres, for the extra obstacles scattered on top
                of the WFC layout.
            obstacle_density: Density range controlling how many extra obstacles are
                scattered.
            shape_distribution: Relative weights and thresholds used to choose obstacle
                shapes.

        Raises:
            ValueError: If the configuration is infeasible (e.g. the world is too small to
                fit the minimum WFC grid size for the given spacing and tile pattern).
        """
    @property
    def obstacle_density(self) -> RandomRange:
        """
        The configured obstacle-density range.
        """
    @property
    def obstacle_size(self) -> RandomRange:
        """
        The configured obstacle-size range, in metres.
        """
    @property
    def shape_distribution(self) -> ShapeDistribution:
        """
        The configured obstacle shape distribution.
        """
class Factory:
    """
    Holds a weighted pool of Generators and produces batches of worlds (or ready-to-use
    Simulations) from it in parallel across a thread pool, retrying a fresh random pick if
    a chosen generator fails to meet its minimum node count.

    Each call to generate_data_containers()/generate_simulations() seeds itself from a
    default-constructed random generator, so results are **not reproducible** from run to
    run: no seed can be passed in.
    """
    def __init__(self, num_threads: typing.SupportsInt | typing.SupportsIndex = 0, max_retries: typing.SupportsInt | typing.SupportsIndex = 100) -> None:
        """
        Constructs a factory with no generators yet (add them via add()).

        Args:
            num_threads: Number of worker threads to use for batch generation; 0 (default)
                uses the number of hardware threads (or 1 if that cannot be determined).
            max_retries: Maximum number of generator picks to try per requested world before
                giving up; must be > 0.

        Raises:
            ValueError: If max_retries == 0.
        """
    def add(self, generator: Generator, weight: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        """
        Adds a generator to the pool with a relative selection weight.

        Args:
            generator: The generator to add.
            weight: Relative selection weight; must be > 0. Defaults to 1.0.

        Raises:
            ValueError: If generator is None or weight <= 0.
        """
    def generate_data_containers(self, count: typing.SupportsInt | typing.SupportsIndex) -> list[murosimpy.io.DataContainer]:
        """
        Generates count worlds in parallel (using num_threads threads), each from an
        independently, non-reproducibly seeded weighted-random generator pick.

        Args:
            count: Number of worlds to generate.

        Returns:
            A list of DataContainer objects.

        Raises:
            RuntimeError: If no generator has been added, or generation fails for any
                of the requested worlds after max_retries attempts.
        """
    def generate_simulations(self, count: typing.SupportsInt | typing.SupportsIndex, grid_scale: typing.SupportsFloat | typing.SupportsIndex = 0.10000000149011612, buffer_scale: typing.SupportsFloat | typing.SupportsIndex = 0.20000000298023224) -> murosimpy.containers.Simulations:
        """
        Same as generate_data_containers(), additionally wrapping each result in a
        manually-initialized Simulation.

        Args:
            count: Number of simulations to generate.
            grid_scale: Metres per occupancy-grid cell, used to initialize each
                Simulation.
            buffer_scale: Extra clearance, in metres, added around obstacles in the
                grid, used to initialize each Simulation.

        Returns:
            A list of Simulation objects.

        Raises:
            RuntimeError: If no generator has been added, or generation fails for any
                of the requested worlds after max_retries attempts.
        """
    @property
    def max_retries(self) -> int:
        """
        The configured maximum number of retries per requested world.
        """
    @property
    def num_threads(self) -> int:
        """
        The configured worker-thread count, or the resolved hardware-concurrency value if 0 was passed to the constructor.
        """
BLOCK: PresetType
CAVERN: PresetType
LABYRINTH: PresetType
LINES: PresetType
OBSTACLE: PresetType
ROOM: PresetType
SPARSE: PresetType
WAREHOUSE: PresetType
__version__: str = '1.0.0'
