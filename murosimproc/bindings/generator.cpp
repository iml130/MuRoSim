// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "generator.hpp"

#include <pybind11/stl.h>

#include <murosimproc/generators/random_range.hpp>
#include <murosimproc/generators/generator.hpp>
#include <murosimproc/generators/maze_generator.hpp>
#include <murosimproc/generators/random_sparse_generator.hpp>
#include <murosimproc/generators/graph_generator.hpp>
#include <murosimproc/generators/wfc_generator.hpp>
#include <murosimproc/obstacle/shape_distribution.hpp>
#include <murosimproc/wfc/preset.hpp>

namespace murosimprocpy {

void initModuleGenerator(pybind11::module &m) {
    namespace py = pybind11;
    using RandomRange = murosimproc::generators::RandomRange;

    // RandomRange
    auto pyRandomRange = py::class_<RandomRange>(m, "RandomRange", R"doc(
A closed interval [min, max] used throughout the generators to describe either a fixed
value (min == max) or a range a parameter is drawn uniformly from at generation time
(domain randomization).

A plain float or a 1- or 2-element tuple is implicitly convertible to a RandomRange
wherever one is expected, so most constructors can be called with e.g. ``0.5`` or
``(0.5, 1.5)`` directly.
)doc");
    pyRandomRange.def(py::init<float>(), "Constructs a fixed (non-random) value: min == max == value.", py::arg("value"));
    pyRandomRange.def(py::init<float, float>(),
        "Constructs a range [min, max] to draw uniformly from.\n\n"
        "Args:\n"
        "    min: Lower bound.\n"
        "    max: Upper bound; must be >= min.\n\n"
        "Raises:\n"
        "    ValueError: If max < min.",
        py::arg("min"), py::arg("max"));
    pyRandomRange.def(py::init([](py::tuple t) {
        if (t.size() == 1)
            return RandomRange(t[0].cast<float>());
        if (t.size() == 2)
            return RandomRange(t[0].cast<float>(), t[1].cast<float>());
        throw std::invalid_argument("RandomRange tuple must have 1 or 2 elements");
    }),
        "Constructs a RandomRange from a 1-element (fixed value) or 2-element (min, max) tuple.\n\n"
        "Raises:\n"
        "    ValueError: If the tuple does not have 1 or 2 elements, or if max < min.",
        py::arg("range"));
    pyRandomRange.def_property_readonly("min", &RandomRange::min, "The lower bound of the range.");
    pyRandomRange.def_property_readonly("max", &RandomRange::max, "The upper bound of the range.");
    pyRandomRange.def("__repr__", [](const RandomRange &r) {
        if (r.min() == r.max())
            return std::string("RandomRange(") + std::to_string(r.min()) + ")";
        return std::string("RandomRange(") + std::to_string(r.min()) + ", " + std::to_string(r.max()) + ")";
    }, "Return a readable representation, e.g. RandomRange(1.0, 2.5) or RandomRange(1.0) for a fixed value.");
    py::implicitly_convertible<float, RandomRange>();
    py::implicitly_convertible<py::tuple, RandomRange>();

    // Generator base
    auto pyGenerator = py::class_<murosimproc::generators::Generator, std::shared_ptr<murosimproc::generators::Generator>>(m, "Generator", R"doc(
Abstract base class for all world generators (maze, graph, line, intersection,
random-sparse, WFC, ...).

A generator turns a random seed into obstacle geometry (lines/circles) plus navigation
nodes, retrying with different seeds internally if the requested minimum node count is
not reached. This class cannot be instantiated directly; use one of its subclasses.
)doc");
    // gil_scoped_release: generation is pure C++ and can take a while for
    // large worlds; holding the GIL would serialize all Python threads.
    // (No overload for murosim::random::RandomGenerator - the type is not
    // registered with pybind11 in any module, so it could never match.)
    pyGenerator.def("__call__", [] (const murosimproc::generators::Generator &self, murosim::random::RandomGenerator::result_type seed) {return self(seed);},
        "Generates a world for the given seed.\n\n"
        "If the first attempt yields fewer than `min_node_count` nodes, retries with\n"
        "seed + 1, seed + 2, ... up to 100 attempts total before giving up. Generation\n"
        "is deterministic: calling this again with the same seed reproduces the exact\n"
        "same lines, circles and nodes.\n\n"
        "Args:\n"
        "    seed: Random seed to generate the world from.\n\n"
        "Returns:\n"
        "    A DataContainer holding the generated lines, circles and nodes, named\n"
        "    from the generator's name and the seed.\n\n"
        "Raises:\n"
        "    RuntimeError: If no attempt reaches the minimum node count within 100 tries.",
        py::arg("seed"), py::call_guard<py::gil_scoped_release>());
    pyGenerator.def_property_readonly("min_node_count", &murosimproc::generators::Generator::getMinNodeCount,
        "The minimum number of navigation nodes a generated world must contain.");
    pyGenerator.def_property_readonly("node_radius", &murosimproc::generators::Generator::getNodeRadius,
        "The configured navigation node radius, in metres.");
    // Config getters return const& — bind them as copies so Python cannot
    // mutate (or race with Factory worker threads on) the generator's
    // internal configuration through a reference_internal alias.
    pyGenerator.def_property_readonly("spacing", &murosimproc::generators::Generator::getSpacing, py::return_value_policy::copy,
        "The configured node-spacing range, in metres, as a RandomRange.");
    pyGenerator.def_property_readonly("world_size", &murosimproc::generators::Generator::getWorldSize, py::return_value_policy::copy,
        "The configured world-size range, in metres, as a RandomRange (worlds are square).");

    // MazeGenerator
    auto pyMaze = py::class_<murosimproc::generators::MazeGenerator, murosimproc::generators::Generator, std::shared_ptr<murosimproc::generators::MazeGenerator>>(m, "MazeGenerator", R"doc(
Generates perfect-maze worlds using Prim's algorithm on a grid, then randomly punches
holes in walls to create loops, converts the result to obstacle geometry and places
navigation nodes, and finally scales everything to the sampled world size.

Deterministic for a given seed (aside from the retry loop in the base Generator, which
advances the seed if the minimum node count is not met).
)doc");
    pyMaze.def(py::init<size_t, float, RandomRange, RandomRange, RandomRange, RandomRange>(),
        R"doc(
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
)doc",
        py::arg("min_node_count"),
        py::arg("node_radius"),
        py::arg("world_size"),
        py::arg("spacing"),
        py::arg("wall_thickness"),
        py::arg("wall_removal_rate") = 0.0f
    );
    pyMaze.def_property_readonly("wall_thickness", &murosimproc::generators::MazeGenerator::getWallThickness, py::return_value_policy::copy,
        "The configured wall-thickness range, in metres.");
    pyMaze.def_property_readonly("wall_removal_rate", &murosimproc::generators::MazeGenerator::getWallRemovalRate, py::return_value_policy::copy,
        "The configured wall-removal-rate range ([0, 1]).");

    // ShapeDistribution
    using ShapeDistribution = murosimproc::obstacle::ShapeDistribution;
    auto pyShapeDistribution = py::class_<ShapeDistribution>(m, "ShapeDistribution", R"doc(
Configuration for obstacle shape selection probabilities.

`circle` and `polygon` are relative weights (normalized internally); obstacles with a
radius below `min_polygon_radius` are always rendered as circles regardless of these
weights.
)doc");
    pyShapeDistribution.def(py::init<>(), "Constructs a ShapeDistribution with default weights (circle=0.5, polygon=0.5, decompose=0.3).");
    pyShapeDistribution.def(py::init<RandomRange, RandomRange, RandomRange>(),
        "Constructs a ShapeDistribution with explicit circle/polygon/decompose ranges. "
        "`min_polygon_radius` keeps its default (0.05).",
        py::arg("circle"), py::arg("polygon"), py::arg("decompose"));
    pyShapeDistribution.def_readwrite("circle", &ShapeDistribution::circle, "Relative weight (range) for selecting a circle shape.");
    pyShapeDistribution.def_readwrite("polygon", &ShapeDistribution::polygon, "Relative weight (range) for selecting a polygon shape.");
    pyShapeDistribution.def_readwrite("decompose", &ShapeDistribution::decompose, "Relative weight (range) for decomposing a shape into simpler parts.");
    pyShapeDistribution.def("__repr__", [](const ShapeDistribution &s) {
        auto rr_str = [](const RandomRange &r) {
            if (r.min() == r.max())
                return std::string("RandomRange(") + std::to_string(r.min()) + ")";
            return std::string("RandomRange(") + std::to_string(r.min()) + ", " + std::to_string(r.max()) + ")";
        };
        return std::string("ShapeDistribution(circle=") + rr_str(s.circle) +
               ", polygon=" + rr_str(s.polygon) +
               ", decompose=" + rr_str(s.decompose) + ")";
    }, "Return a readable representation listing the circle, polygon, decompose ranges and min_polygon_radius.");

    // RandomSparseGenerator
    auto pyRandomSparse = py::class_<murosimproc::generators::RandomSparseGenerator, murosimproc::generators::Generator, std::shared_ptr<murosimproc::generators::RandomSparseGenerator>>(m, "RandomSparseGenerator", R"doc(
Generates worlds with obstacles scattered randomly across open space (no walls or
corridors), placed without overlap up to a density-dependent number of attempts, plus
navigation nodes.

Deterministic for a given seed (aside from the retry loop in the base Generator, which
advances the seed if the minimum node count is not met).
)doc");
    // Primary constructor: with ShapeDistribution
    pyRandomSparse.def(py::init<size_t, float, RandomRange, RandomRange, RandomRange, RandomRange, ShapeDistribution>(),
        R"doc(
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
)doc",
        py::arg("min_node_count"),
        py::arg("node_radius"),
        py::arg("world_size"),
        py::arg("spacing"),
        py::arg("obstacle_size"),
        py::arg("placement_density_rate"),
        py::arg("shape_distribution")
    );
    // Convenience constructor: with shape_entropy (converts to ShapeDistribution)
    pyRandomSparse.def(py::init<size_t, float, RandomRange, RandomRange, RandomRange, RandomRange, RandomRange>(),
        R"doc(
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
)doc",
        py::arg("min_node_count"),
        py::arg("node_radius"),
        py::arg("world_size"),
        py::arg("spacing"),
        py::arg("obstacle_size") = RandomRange(murosimproc::generators::defaults::MIN_OBSTACLE_SCALE, murosimproc::generators::defaults::MAX_OBSTACLE_SCALE),
        py::arg("placement_density_rate") = murosimproc::generators::defaults::PLACEMENT_DENSITY_RATE,
        py::arg("shape_entropy") = RandomRange(0.5f)
    );
    pyRandomSparse.def_static("entropy_to_shape_distribution",
        &murosimproc::generators::RandomSparseGenerator::entropyToShapeDistribution,
        "Converts a shape-entropy range in [0, 1] to a ShapeDistribution "
        "(circle weight = entropy, polygon weight = (1 - entropy) / 2, decompose weight = entropy / 2).",
        py::arg("entropy"));
    pyRandomSparse.def_property_readonly("obstacle_size", &murosimproc::generators::RandomSparseGenerator::getObstacleSize, py::return_value_policy::copy,
        "The configured obstacle-size range, in metres.");
    pyRandomSparse.def_property_readonly("shape_distribution", &murosimproc::generators::RandomSparseGenerator::getShapeDistribution, py::return_value_policy::copy,
        "The configured obstacle shape distribution.");
    pyRandomSparse.def_property_readonly("placement_density_rate", &murosimproc::generators::RandomSparseGenerator::getPlacementDensityRate, py::return_value_policy::copy,
        "The configured placement-density-rate range.");

    // GraphGenerator
    auto pyGraph = py::class_<murosimproc::generators::GraphGenerator, murosimproc::generators::Generator, std::shared_ptr<murosimproc::generators::GraphGenerator>>(m, "GraphGenerator", R"doc(
Generates corridor-network worlds: Poisson-disk sampled points are triangulated
(Delaunay), randomly thinned, converted into rounded-corner corridor walls ("tubes"),
and navigation nodes are placed at the surviving graph vertices.

Deterministic for a given seed (aside from the retry loop in the base Generator, which
advances the seed if the minimum node count is not met).
)doc");
    pyGraph.def(py::init<size_t, float, RandomRange, RandomRange, RandomRange, RandomRange, RandomRange, size_t, size_t, float>(),
        R"doc(
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
)doc",
        py::arg("min_node_count"),
        py::arg("node_radius"),
        py::arg("world_size"),
        py::arg("spacing"),
        py::arg("edge_length"),
        py::arg("point_removal_rate"),
        py::arg("edge_removal_rate"),
        py::arg("tube_num_buckets") = 5,
        py::arg("tube_arc_segments") = 8,
        py::arg("tube_simplify_tolerance") = 0.0f
    );

    // WFC PresetType enum
    py::enum_<murosimproc::wfc::preset::PresetType>(m, "PresetType",
        "Built-in tile presets for WfcGenerator, each producing a different style of world.")
        .value("BLOCK", murosimproc::wfc::preset::PresetType::BLOCK, "Balanced layout with walls and obstacles")
        .value("OBSTACLE", murosimproc::wfc::preset::PresetType::OBSTACLE, "Open space with scattered obstacle clusters")
        .value("LABYRINTH", murosimproc::wfc::preset::PresetType::LABYRINTH, "Dense maze with narrow corridors")
        .value("WAREHOUSE", murosimproc::wfc::preset::PresetType::WAREHOUSE, "Mixed walls with obstacle zones")
        .value("ROOM", murosimproc::wfc::preset::PresetType::ROOM, "Connected rooms with doorways")
        .value("SPARSE", murosimproc::wfc::preset::PresetType::SPARSE, "Minimal walls, scattered L-shaped obstacles")
        .value("CAVERN", murosimproc::wfc::preset::PresetType::CAVERN, "Organic thick walls with obstacle pockets")
        .value("LINES", murosimproc::wfc::preset::PresetType::LINES, "Parallel wall and obstacle lines")
        .export_values();

    // WfcGenerator
    auto pyWFC = py::class_<murosimproc::generators::WfcGenerator, murosimproc::generators::Generator, std::shared_ptr<murosimproc::generators::WfcGenerator>>(m, "WFCGenerator", R"doc(
Generates worlds via Wave Function Collapse (WFC): a tile-pattern grid is collapsed
into a layout of empty/wall/obstacle cells, then converted to obstacle geometry and
navigation nodes, additionally scattering extra small obstacles.

Deterministic for a given seed (aside from the retry loop in the base Generator); if
WFC hits a contradiction, that attempt silently produces an empty world and the retry
loop tries the next seed.
)doc");
    pyWFC.def(py::init<
            size_t, float,
            RandomRange, RandomRange,
            murosimproc::wfc::preset::PresetType,
            RandomRange, RandomRange,
            ShapeDistribution
        >(),
        R"doc(
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
)doc",
        py::arg("min_node_count"),
        py::arg("node_radius"),
        py::arg("world_size"),
        py::arg("spacing"),
        py::arg("preset_type"),
        py::arg("obstacle_size"),
        py::arg("obstacle_density"),
        py::arg("shape_distribution")
    );
    pyWFC.def_property_readonly("obstacle_density", &murosimproc::generators::WfcGenerator::getObstacleDensity, py::return_value_policy::copy,
        "The configured obstacle-density range.");
    pyWFC.def_property_readonly("obstacle_size", &murosimproc::generators::WfcGenerator::getObstacleSize, py::return_value_policy::copy,
        "The configured obstacle-size range, in metres.");
    pyWFC.def_property_readonly("shape_distribution", &murosimproc::generators::WfcGenerator::getShapeDistribution, py::return_value_policy::copy,
        "The configured obstacle shape distribution.");
}

} // namespace murosimprocpy
