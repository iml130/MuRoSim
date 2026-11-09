// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_PLANNING_PLANNER_HPP_
#define MUROSIM_PLANNING_PLANNER_HPP_

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <murosim/grid/map.hpp>
#include <vector>
#include <memory>
#include <string>

namespace murosim::planning
{
/// Base class for grid-based path planners (see AStar, Voronoi). Concrete
/// planners implement computePath() to search between two points already
/// known to be free/reachable on the map; plan() is the public entry point
/// that additionally handles start/goal points lying in non-free cells and
/// input validation. Planners are cloned (not shared) when a simulation needs
/// one instance per robot; see murosim::planning::factory::createPlanner().
class Planner
{
public:
    /// Identifies a concrete planner implementation; NONE means no planner is configured.
    enum class TYPE{NONE, ASTAR4, ASTAR8, VORONOI};

    /// Returns the canonical string for a planner type (e.g. "ASTAR4"). @throws std::runtime_error for an unknown type.
    static std::string getTypeAsString(const TYPE type);
    /// Returns the canonical string for this planner's type; equivalent to getTypeAsString(getType()).
    std::string getTypeAsString() const;
    /// Parses a planner type from its string name (case-insensitive). @throws std::runtime_error if the name is unrecognized.
    static TYPE getTypeFromString(std::string type);

    /// Plans a path from start to goal (world coordinates) on map.
    /// If start or goal fall on a non-FREE cell, first walks the shortest
    /// 4-connected escape route to the nearest FREE/BUFFER cell
    /// (see planning::util::findNearestPathToFreePoint) and prepends/appends
    /// that detour to the computed path.
    /// @return the path as a sequence of world points from start to goal (goal-to-start order is never returned).
    /// @throws std::out_of_range if start or goal lies outside the map.
    /// @throws std::runtime_error if no FREE cell is reachable from start or goal, or if start and goal lie in disconnected free regions.
    std::vector<Eigen::Vector2f> plan(const murosim::grid::Map& map, const Eigen::Vector2f start, const Eigen::Vector2f goal);
    /// Returns a deep copy of this planner (used to give each robot its own planner instance).
    /// Planners are owned and deleted through `std::unique_ptr<Planner>`.
    virtual ~Planner() = default;
    virtual std::unique_ptr<Planner> clone() const = 0;
    /// Returns this planner's TYPE.
    virtual TYPE getType() const = 0;
protected:
    /// Computes a path between start and goal, which are guaranteed to be
    /// (or to have been mapped to) FREE cells on map. Implementations return
    /// an empty vector if no path exists.
    virtual std::vector<Eigen::Vector2f> computePath(const murosim::grid::Map& map, const Eigen::Vector2f start, const Eigen::Vector2f goal) = 0;
};

} // namespace murosim::planning

#endif // MUROSIM_PLANNING_PLANNER_HPP_
