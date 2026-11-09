// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_GRAPH_GENERATOR_HPP_
#define MUROSIMPROC_GENERATORS_GRAPH_GENERATOR_HPP_

#include "murosimproc/generators/generator.hpp"

namespace murosimproc::generators
{

/// Generates corridor-network worlds: Poisson-disk sampled points are triangulated (Delaunay), randomly
/// thinned, converted into rounded-corner corridor walls ("tubes"), and navigation nodes are placed at
/// the surviving graph vertices. Deterministic for a given seed (aside from `operator()`'s own retry
/// loop in the base `Generator`, which advances the seed if the minimum node count is not met).
class GraphGenerator : public murosimproc::generators::Generator
{
private:
    RandomRange point_removal_rate_; // [0,1] — fraction of Poisson disk points to remove
    RandomRange edge_removal_rate_;  // [0,1] — fraction of Delaunay edges to remove; low = dense, high = sparse
    RandomRange edge_length_;            // max allowed edge length (min/max in world units)
    size_t tube_num_buckets_;
    size_t tube_arc_segments_;
    float tube_simplify_tolerance_;

    /// Validates the graph-specific configuration on top of the base `Generator` checks, throwing
    /// std::invalid_argument if: `point_removal_rate`/`edge_removal_rate` are outside `[0, 1]`;
    /// `edge_length.min() <= 0` or is smaller than half the maximum spacing (tube half-width);
    /// `tube_num_buckets`/`tube_arc_segments` is 0; `tube_simplify_tolerance < 0`; or the world is too
    /// small (after margins) to fit points spaced by `edge_length`.
    void validateGraphConfig() const;

protected:
    void generate(
        murosim::random::RandomGenerator::result_type seed,
        std::vector<murosim::geometry::Line> &lines,
        std::vector<murosim::geometry::Circle> &circles,
        std::vector<murosim::navigation::Node> &nodes
    ) const override;

public:
    /// Constructs a graph-network generator.
    /// @param edge_length Min/max spacing (metres) used both for the Poisson-disk point sampling radius and as the minimum allowed edge length.
    /// @param point_removal_rate Fraction in [0, 1] of sampled points to discard before triangulation (subject to a floor of `2 * min_node_count` surviving points).
    /// @param edge_removal_rate Fraction in [0, 1] of Delaunay edges to randomly remove afterwards; low values give a dense graph, high values a sparse one.
    /// @param tube_num_buckets Number of spatial buckets used when merging corridor-wall polygons (see `graph::converter::convertToTubes`); must be > 0.
    /// @param tube_arc_segments Number of segments used to approximate rounded corridor corners; must be > 0.
    /// @param tube_simplify_tolerance Tolerance (metres) for simplifying the resulting corridor outlines; must be >= 0, 0 disables simplification.
    /// @throws std::invalid_argument via `validateGraphConfig()` if the configuration is infeasible.
    GraphGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        RandomRange edge_length,
        RandomRange point_removal_rate,
        RandomRange edge_removal_rate,
        size_t tube_num_buckets = 5,
        size_t tube_arc_segments = 8,
        float tube_simplify_tolerance = 0.0f
    );
};

} // namespace murosimproc::generators

#endif // MUROSIMPROC_GENERATORS_GRAPH_GENERATOR_HPP_
