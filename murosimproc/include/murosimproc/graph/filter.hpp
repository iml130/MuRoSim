// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GRAPH_FILTER_HPP_
#define MUROSIMPROC_GRAPH_FILTER_HPP_

#include "graph.hpp"
#include "murosim/random/random_generator.hpp"

namespace murosimproc::graph::filter
{

/**
 * @brief Randomly remove edges while keeping the graph connected
 *
 * Uses a spanning tree approach: first builds a random spanning tree to ensure
 * connectivity, then randomly removes remaining edges based on keep_probability.
 *
 * @param graph Graph to modify (in-place)
 * @param random_generator Random number generator
 * @param keep_probability Probability to keep non-spanning-tree edges [0.0, 1.0]
 */
void byEdgeChoiceRandom(Graph& graph, murosim::random::RandomGenerator& random_generator, float keep_probability);

/**
 * @brief Select graph vertices suitable as navigation nodes
 *
 * A robot parked on a junction vertex (degree >= 3) blocks several tubes at
 * once, so junctions are only used as fallback: the result contains all
 * vertices of degree 1 (dead ends) and 2 (bends), and is filled up with
 * junction vertices (lowest degree first, random order within equal degree)
 * only while fewer than min_node_count vertices are selected. Isolated
 * vertices (degree 0) are never selected.
 */
std::vector<Vertex> selectNodeVertices(const Graph& graph, size_t min_node_count, murosim::random::RandomGenerator& random_generator);

} // namespace murosimproc::graph::filter

#endif // MUROSIMPROC_GRAPH_FILTER_HPP_
