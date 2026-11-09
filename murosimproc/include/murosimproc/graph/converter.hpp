// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GRAPH_CONVERTER_HPP_
#define MUROSIMPROC_GRAPH_CONVERTER_HPP_

#include "graph.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/random/random_generator.hpp"
#include <vector>

namespace murosimproc::graph::converter
{

/**
 * @brief Convert graph to tube corridors with variable spacing
 *
 * Groups edges into discrete spacing buckets, buffers each bucket as a multi-linestring,
 * then unions the results. This provides variable tube widths for generalization while
 * maintaining good performance (O(num_buckets) unions instead of O(num_edges)).
 *
 * @param graph Input graph
 * @param min_spacing Minimum tube half-width (buffer distance)
 * @param max_spacing Maximum tube half-width
 * @param seed Random seed for spacing variation
 * @param num_buckets Number of discrete spacing levels (default 4, higher = more variation but slower)
 * @param arc_segments Number of segments for rounded corners/ends (default 8)
 * @param simplify_tolerance Simplification tolerance (0 = no simplification, default 0.0)
 * @return Lines representing the tube walls
 */
std::vector<murosim::geometry::Line> convertToTubes(
    const Graph& graph,
    float min_spacing,
    float max_spacing,
    murosim::random::RandomGenerator::result_type seed,
    size_t num_buckets = 4,
    size_t arc_segments = 8,
    float simplify_tolerance = 0.0f);


} // namespace murosimproc::graph::converter

#endif // MUROSIMPROC_GRAPH_CONVERTER_HPP_
