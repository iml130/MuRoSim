// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/planning/planner.hpp"

#include "murosim/planning/util.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace murosim::planning
{

std::string Planner::getTypeAsString(const TYPE type)
{
    switch(type)
    {
        case TYPE::NONE:    return "NONE";
        case TYPE::ASTAR4:  return "ASTAR4";
        case TYPE::ASTAR8:  return "ASTAR8";
        case TYPE::VORONOI: return "VORONOI";
        default:
            throw std::runtime_error("UNKNOWN_PLANNER");
    }
}

std::string Planner::getTypeAsString() const
{
    return getTypeAsString(getType());
}

Planner::TYPE Planner::getTypeFromString(std::string type)
{
    std::transform(type.begin(), type.end(), type.begin(), ::toupper);

    if(type == "NONE")
        return Planner::TYPE::NONE;
    else if(type == "ASTAR4")
        return Planner::TYPE::ASTAR4;
    else if(type == "ASTAR8")
        return Planner::TYPE::ASTAR8;
    else if(type == "VORONOI")
        return Planner::TYPE::VORONOI;
    else
        throw std::runtime_error("UNKNOWN_PLANNER");
}

std::vector<Eigen::Vector2f> Planner::plan(const murosim::grid::Map& map, const Eigen::Vector2f start, const Eigen::Vector2f goal)
{
    std::vector<Eigen::Vector2f> path;

    Eigen::Vector2i point_start;
    map.fromWorldIntoMap(start, point_start);
    Eigen::Vector2i point_goal;
    map.fromWorldIntoMap(goal, point_goal);

    // Invalid input (start/goal outside the grid) is a caller error -> out_of_range.
    if (!map.isInMap(point_start))
    {
        std::stringstream ss;
        ss << "[planning::Planner]: Start is not in the range of the gridmap! [0..x.." << map.sizeX() << "] != " << point_start.x() << ": [0..y.." << map.sizeY() << "] -> " << point_start.y();
        throw std::out_of_range(ss.str());
    }

    if (!map.isInMap(point_goal))
    {
        std::stringstream ss;
        ss << "[planning::Planner]: Goal is not in the range of the gridmap! [0..x.." << map.sizeX() << "] != " << point_goal.x() << ": [0..y.." << map.sizeY() << "] -> " << point_goal.y();
        throw std::out_of_range(ss.str());
    }

    if(point_start == point_goal)
    {
        Eigen::Vector2f point_world;
        map.fromMapIntoWorld(point_goal, point_world);
        path.push_back(point_world);
        return path;
    }

    std::vector<Eigen::Vector2f> path_start;
    std::vector<Eigen::Vector2f> path_goal;
    std::vector<Eigen::Vector2f> path_computed;

    path_start = murosim::planning::util::findNearestPathToFreePoint(map, point_start);
    path_goal = murosim::planning::util::findNearestPathToFreePoint(map, point_goal);

    // No free cell reachable from start/goal -> the map is unsolvable -> runtime_error.
    if (path_start.empty() || path_goal.empty())
    {
        std::stringstream ss;
        ss << "[planning::Planner]: No free cell reachable from "
           << (path_start.empty() ? "start" : "goal") << " position! "
           << "start=(" << start.x() << ", " << start.y() << ") "
           << "goal=(" << goal.x() << ", " << goal.y() << ")";
        throw std::runtime_error(ss.str());
    }

    // Both escape paths run from the original point towards the nearest FREE
    // cell; A* can only ever reach FREE cells, so it must connect the two
    // free endpoints (path_*.back()).
    path_computed = computePath(map, path_start.back(), path_goal.back());

    // An empty result means start and goal lie in disconnected free regions:
    // the map is unsolvable. plan() never returns an empty path -> runtime_error.
    if (path_computed.empty())
    {
        std::stringstream ss;
        ss << "[planning::Planner]: No path between the free cells nearest to "
           << "start=(" << start.x() << ", " << start.y() << ") and "
           << "goal=(" << goal.x() << ", " << goal.y() << "). "
           << "The positions lie in disconnected free regions of the map.";
        throw std::runtime_error(ss.str());
    }

    path.insert(std::end(path), std::cbegin(path_start), std::cend(path_start) - 1);
    path.insert(std::end(path), std::cbegin(path_computed), std::cend(path_computed));
    // walk the goal-side escape path backwards: free cell -> goal
    path.insert(std::end(path), std::crbegin(path_goal) + 1, std::crend(path_goal));

    return path;
}

} // murosim::planning
