// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_PLANNING_FACTORY_HPP_
#define MUROSIM_PLANNING_FACTORY_HPP_

#include "astar.hpp"
#include "voronoi.hpp"
#include "planner.hpp"
#include <memory>

namespace murosim::planning::factory
{

/// Constructs a new planner instance for the given type: AStar(false) for
/// ASTAR4, AStar(true) for ASTAR8, a Voronoi placeholder for VORONOI (whose
/// computePath()/plan() always throws -- see murosim::planning::Voronoi), or
/// nullptr for NONE.
/// @throws std::runtime_error for an unrecognized type.
inline std::unique_ptr<murosim::planning::Planner> createPlanner(const murosim::planning::Planner::TYPE type)
{
    switch(type)
    {
        case murosim::planning::Planner::TYPE::NONE:
            return nullptr;
        case murosim::planning::Planner::TYPE::ASTAR4:
            return std::make_unique<AStar>(false);
        case murosim::planning::Planner::TYPE::ASTAR8:
            return std::make_unique<AStar>(true);
        case murosim::planning::Planner::TYPE::VORONOI:
            // NOTE: Voronoi is not implemented yet; the returned planner throws
            // on plan()/computePath(). See murosim::planning::Voronoi.
            return std::make_unique<Voronoi>();
        default:
            throw std::runtime_error("Unknown Planner type!");
    }
}

} // namespace murosim::planning::factory

#endif // MUROSIM_PLANNING_FACTORY_HPP_
