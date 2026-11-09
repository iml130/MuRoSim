// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_PLANNING_UTIL_HPP_
#define MUROSIM_PLANNING_UTIL_HPP_

#include "murosim/grid/map.hpp"
#include <unordered_map>
#include <vector>


namespace murosim::planning::util
{

/// Breadth-first, 4-connected search from point (world coordinates) for the
/// nearest FREE or BUFFER grid cell. If point already lies on a FREE cell,
/// returns immediately with just that point. Otherwise returns the shortest
/// escape path (world coordinates) from point to the nearest reachable FREE
/// cell, inclusive of both ends; the search itself may pass through BUFFER
/// cells but only stops at a FREE one.
/// @return the escape path, or an empty vector if no FREE cell is reachable.
/// @throws std::runtime_error if point lies outside the map.
std::vector<Eigen::Vector2f> findNearestPathToFreePoint(const murosim::grid::Map& map, const Eigen::Vector2f point);
/// Same as the world-coordinate overload, for a grid cell point.
std::vector<Eigen::Vector2f> findNearestPathToFreePoint(const murosim::grid::Map& map, const Eigen::Vector2i point);

} // namespace murosim::planning::util

#endif // MUROSIM_PLANNING_UTIL_HPP_
