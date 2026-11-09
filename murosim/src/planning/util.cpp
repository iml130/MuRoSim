// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/planning/util.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <queue>
#include <vector>

// right, left, up, down. The escape search is intentionally 4-connected: it
// only needs the nearest free cell, and orthogonal steps keep the escape
// corridor away from obstacle corners.
const static std::array<const Eigen::Vector2i, 4> NEIGHBOR_DIRECTIONS_4 = {
            Eigen::Vector2i(1, 0), Eigen::Vector2i(-1, 0), Eigen::Vector2i(0, 1), Eigen::Vector2i(0, -1)
};

namespace murosim::planning::util
{

std::vector<Eigen::Vector2f> findNearestPathToFreePoint(const murosim::grid::Map& map, const Eigen::Vector2i point_start)
{
    Eigen::Vector2f start;
    map.fromMapIntoWorld(point_start, start);
    return findNearestPathToFreePoint(map, start);
}

std::vector<Eigen::Vector2f> findNearestPathToFreePoint(const murosim::grid::Map& map, const Eigen::Vector2f start)
{
    std::vector<Eigen::Vector2f> path;
    Eigen::Vector2i point_start;
    map.fromWorldIntoMap(start, point_start);

    if (!map.isInMap(point_start))
    {
        std::stringstream ss;
        ss << "[planning::util::findNearestPathToFreePoint]: Start is not in the range of the gridmap! [0..x.." << map.sizeX() << "] != " << point_start.x() << ": [0..y.." << map.sizeY() << "] -> " << point_start.y();
        throw std::runtime_error(ss.str());
    }

    if (map.isFree(point_start))
    {
        path.push_back(start);
        return path;
    }

    std::queue<size_t> frontier;
    std::unordered_map<size_t, size_t> came_from;

    const size_t index_start = map.pointToIndex(point_start);
    bool has_found_free_point = false;

    frontier.push(index_start);
    came_from[index_start] = index_start;

    size_t index_current = index_start;

    while (!frontier.empty())
    {
        index_current = frontier.front();
        frontier.pop();

        if (map.isFree(index_current))
        {
            has_found_free_point = true;
            break;
        }

        const Eigen::Vector2i point_current = map.indexToPoint(index_current);

        for (size_t i = 0; i < NEIGHBOR_DIRECTIONS_4.size(); i++)
        {
            const Eigen::Vector2i point_neighbor = point_current + NEIGHBOR_DIRECTIONS_4[i];

            if (map.isInMap(point_neighbor))
            {
                const size_t index_neighbor = map.pointToIndex(point_neighbor);

                if ((map[index_neighbor] == murosim::grid::Map::FIELD::BUFFER || map[index_neighbor] == murosim::grid::Map::FIELD::FREE) && came_from.find(index_neighbor) == came_from.end())
                {
                    came_from[index_neighbor] = index_current;
                    frontier.emplace(index_neighbor);
                }
            }
        }
    }

    if (has_found_free_point)
    {
        path.reserve(came_from.size());
        Eigen::Vector2f pos;

        while(index_current != index_start)
        {
            map.fromMapIntoWorld(map.indexToPoint(index_current), pos);
            path.push_back(pos);
            index_current = came_from[index_current];
        }

        map.fromMapIntoWorld(map.indexToPoint(index_start), pos);
        path.push_back(pos);

        std::reverse(path.begin(), path.end());
    }

    return path;
}

} // namespace murosim::planning::util
