// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GRAPH_ALGORITHM_HPP_
#define MUROSIMPROC_GRAPH_ALGORITHM_HPP_

#include "murosimproc/graph/graph.hpp"
#include "murosim/random/random_generator.hpp"

namespace murosimproc::graph::algorithm
{

/**
 * @brief Construct a Delaunay triangulation graph from 2D points
 *
 * Uses Boost.Polygon's Voronoi diagram to compute the Delaunay triangulation.
 * Points are normalized and scaled to integers for numerical precision.
 * Near-duplicate points are filtered to prevent degenerate cases.
 *
 * @param points Input points (at least 2 required for edges)
 * @return Graph with bidirectional Delaunay triangulation edges.
 *         Returns empty graph if fewer than 2 distinct points.
 */
Graph makeDelaunayGraph(const std::vector<Eigen::Vector2f> &points);

} // namespace murosimproc::graph::algorithm

#endif // MUROSIMPROC_GRAPH_ALGORITHM_HPP_
