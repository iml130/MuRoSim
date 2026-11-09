// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/planning/voronoi.hpp"

#include <vector>

namespace murosim::planning
{

std::vector<Eigen::Vector2f> Voronoi::computePath(const murosim::grid::Map& map, const Eigen::Vector2f start, const Eigen::Vector2f goal)
{
    (void) map;
    (void) start;
    (void) goal;

    throw std::runtime_error("[planning::Voronoi]: The Voronoi planner is not implemented yet!");

    return {};
}

std::unique_ptr<Planner> Voronoi::clone() const
{
    return std::make_unique<Voronoi>(*this);
}

Planner::TYPE Voronoi::getType() const
{
    return Planner::TYPE::VORONOI;
}

} // namespace murosim::planning
