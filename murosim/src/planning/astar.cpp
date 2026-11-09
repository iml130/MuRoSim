// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/planning/astar.hpp"
#include "murosim/planning/util.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>


// right, up, left, down
const static std::array<const Eigen::Vector2i, 4> NEIGHBOR_DIRECTIONS_STRAIGHT = {
    Eigen::Vector2i(1, 0), Eigen::Vector2i(0, 1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(0, -1)
};

// (right-up), (left-up), (left-down), (right-down)
const static std::array<const Eigen::Vector2i, 4> NEIGHBOR_DIRECTIONS_DIAGONAL = {
    Eigen::Vector2i(1, 1), Eigen::Vector2i(-1, 1), Eigen::Vector2i(-1, -1), Eigen::Vector2i(1, -1)
};

constexpr double COST_FREE = 1.0;
constexpr double COST_FREE_DIAGONAL = COST_FREE * M_SQRT2;

namespace {

// A* open-list entry: a cell index ordered by its f-score (g-score + heuristic).
struct FrontierEntry
{
    size_t index;
    double f_score;
};

// Orders the frontier as a min-heap on f-score (lowest f-score popped first).
struct FrontierCompare
{
    bool operator()(const FrontierEntry &lhs, const FrontierEntry &rhs) const
    {
        return lhs.f_score > rhs.f_score;
    }
};

// Back-link for path reconstruction plus the best known g-score of a cell.
struct CameFrom
{
    size_t predecessor;
    double g_score;
};

} // namespace

namespace murosim::planning
{

AStar::AStar(const bool use_neighbors8)
    : use_neighbors8_(use_neighbors8)
{

}

std::unique_ptr<Planner> AStar::clone() const
{
    return std::make_unique<AStar>(*this);
}

Planner::TYPE AStar::getType() const
{
    return (use_neighbors8_)? Planner::TYPE::ASTAR8: Planner::TYPE::ASTAR4;
}

std::vector<Eigen::Vector2f> AStar::computePath(const murosim::grid::Map& map, const Eigen::Vector2f start, const Eigen::Vector2f goal)
{
    std::vector<Eigen::Vector2f> path;

    Eigen::Vector2i point_start;
    map.fromWorldIntoMap(start, point_start);
    Eigen::Vector2i point_goal;
    map.fromWorldIntoMap(goal, point_goal);

    if(point_start == point_goal)
    {
        return {start};
    }

    const size_t index_start = map.pointToIndex(point_start);
    const size_t index_goal = map.pointToIndex(point_goal);
    bool has_found_path = false;
    std::priority_queue<FrontierEntry, std::vector<FrontierEntry>, FrontierCompare> frontier;
    std::unordered_map<size_t, CameFrom> came_from_with_cost;
    std::unordered_set<size_t> closed_set;

    // Pre-allocate memory for better performance on large maps
    const size_t estimated_nodes = map.sizeX() * map.sizeY() / 4;
    came_from_with_cost.reserve(estimated_nodes);
    closed_set.reserve(estimated_nodes);

    frontier.push(FrontierEntry{index_start, 0.0});
    came_from_with_cost[index_start] = CameFrom{index_start, 0.0};

    while(!frontier.empty())
    {
        const size_t index_current = frontier.top().index;
        frontier.pop();

        // Skip if already processed (handles duplicate queue entries)
        if (closed_set.count(index_current))
        {
            continue;
        }
        closed_set.insert(index_current);

        if (index_current == index_goal)
        {
            has_found_path = true;
            break;
        }

        const double current_g_score = came_from_with_cost[index_current].g_score;

        // astar with 8 neighbors
        if (use_neighbors8_)
        {
            const std::vector<std::pair<size_t, double>> neighbors = getNeighbors8(map, index_current);

            for (const auto& [index_next, cost] : neighbors)
            {
                // Skip already processed nodes
                if (closed_set.count(index_next))
                {
                    continue;
                }

                const double new_g_score = current_g_score + cost;

                auto [it, inserted] = came_from_with_cost.try_emplace(index_next, CameFrom{index_current, new_g_score});
                if (inserted || new_g_score < it->second.g_score)
                {
                    if (!inserted)
                    {
                        it->second = CameFrom{index_current, new_g_score};
                    }
                    const double f_score = new_g_score + heuristicOctile(map.indexToPoint(index_next), point_goal);
                    frontier.push(FrontierEntry{index_next, f_score});
                }
            }
        }
        // astar with 4 neighbors
        else
        {
            const std::vector<std::pair<size_t, double>> neighbors = getNeighbors4(map, index_current);

            for (const auto& [index_next, cost] : neighbors)
            {
                // Skip already processed nodes
                if (closed_set.count(index_next))
                {
                    continue;
                }

                const double new_g_score = current_g_score + cost;

                auto [it, inserted] = came_from_with_cost.try_emplace(index_next, CameFrom{index_current, new_g_score});
                if (inserted || new_g_score < it->second.g_score)
                {
                    if (!inserted)
                    {
                        it->second = CameFrom{index_current, new_g_score};
                    }
                    const double f_score = new_g_score + heuristicManhattanDistance(map.indexToPoint(index_next), point_goal);
                    frontier.push(FrontierEntry{index_next, f_score});
                }
            }
        }
    }

    // if a path is found, create the path else return empty path list!
    if (has_found_path)
    {
        // Path length is typically much smaller than visited nodes
        path.reserve(static_cast<size_t>(std::sqrt(came_from_with_cost.size()) * 2));
        Eigen::Vector2f pos;
        size_t index_current = index_goal;

        while(index_current != index_start)
        {
            map.fromMapIntoWorld(map.indexToPoint(index_current), pos);
            path.push_back(pos);
            index_current = came_from_with_cost[index_current].predecessor;
        }

        map.fromMapIntoWorld(map.indexToPoint(index_start), pos);
        path.push_back(pos);

        std::reverse(path.begin(), path.end());
    }

    return path;
}

double AStar::heuristicManhattanDistance(const Eigen::Vector2i p1, const Eigen::Vector2i p2)
{
    return std::abs(p1.x() - p2.x()) + std::abs(p1.y() - p2.y());
}

double AStar::heuristicOctile(const Eigen::Vector2i p1, const Eigen::Vector2i p2)
{
    const int dx = std::abs(p1.x() - p2.x());
    const int dy = std::abs(p1.y() - p2.y());
    // Octile distance: optimal heuristic for 8-direction movement with diagonal cost √2
    return std::max(dx, dy) + (M_SQRT2 - 1.0) * std::min(dx, dy);
}

std::vector<std::pair<size_t, double>> AStar::getNeighbors8(const murosim::grid::Map& map, const size_t index) const
{
    return getNeighbors8(map, map.indexToPoint(index));
}

std::vector<std::pair<size_t, double>> AStar::getNeighbors8(const murosim::grid::Map& map, const Eigen::Vector2i point) const
{
    std::vector<std::pair<size_t, double>> neighbors;
    neighbors.reserve(8);

    std::array<bool, 5> direction_is_added = {false, false, false, false, false};

    for (size_t i = 0; i < NEIGHBOR_DIRECTIONS_STRAIGHT.size(); i++)
    {
        const Eigen::Vector2i neighbor = point + NEIGHBOR_DIRECTIONS_STRAIGHT[i];

        if (map.isInMap(neighbor))
        {
            const size_t neighbor_index = map.pointToIndex(neighbor);
            const auto field = map[neighbor_index];  // Cache the lookup

            if (field == murosim::grid::Map::FIELD::FREE)
            {
                neighbors.emplace_back(neighbor_index, COST_FREE);
                direction_is_added[i] = true;
            }
        }
    }

    // add the last element to the first element to close the loop.
    direction_is_added[4] = direction_is_added[0];

    for (size_t i = 0; i < NEIGHBOR_DIRECTIONS_DIAGONAL.size(); i++)
    {
        // Check isInMap is not needed, because the diagonal neighbors depend on the straight neighbors which are added.
        if (direction_is_added[i] && direction_is_added[i + 1])
        {
            const size_t neighbor_index = map.pointToIndex(point + NEIGHBOR_DIRECTIONS_DIAGONAL[i]);
            const auto field = map[neighbor_index];  // Cache the lookup

            if (field == murosim::grid::Map::FIELD::FREE)
            {
                neighbors.emplace_back(neighbor_index, COST_FREE_DIAGONAL);
            }
        }
    }

    return neighbors;
}

std::vector<std::pair<size_t, double>> AStar::getNeighbors4(const murosim::grid::Map& map, const size_t index) const
{
    return getNeighbors4(map, map.indexToPoint(index));
}

std::vector<std::pair<size_t, double>> AStar::getNeighbors4(const murosim::grid::Map& map, const Eigen::Vector2i point) const
{
    std::vector<std::pair<size_t, double>> neighbors;
    neighbors.reserve(4);

    for (size_t i = 0; i < NEIGHBOR_DIRECTIONS_STRAIGHT.size(); i++)
    {
        const Eigen::Vector2i neighbor = point + NEIGHBOR_DIRECTIONS_STRAIGHT[i];

        if (map.isInMap(neighbor))
        {
            const size_t neighbor_index = map.pointToIndex(neighbor);
            const auto field = map[neighbor_index];  // Cache the lookup

            if (field == murosim::grid::Map::FIELD::FREE)
            {
                neighbors.emplace_back(neighbor_index, COST_FREE);
            }
        }
    }

    return neighbors;
}

} // namespace murosim::planning
