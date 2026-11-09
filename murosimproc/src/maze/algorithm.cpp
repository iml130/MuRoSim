// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/maze/algorithm.hpp"
#include "murosimproc/util/image.hpp"

#include <array>
#include <vector>
#include <random>
#include <cassert>
#include <Eigen/Core>

namespace murosimproc::maze
{

murosimproc::util::Array2d<uint8_t> generateMazePrim(
    const size_t size_x,
    const size_t size_y,
    murosim::random::RandomGenerator &random_generator)
{
    assert(0 < size_x);
    assert(0 < size_y);

    const std::array<Eigen::Vector2i, 4> directions = {
        Eigen::Vector2i(2, 0),
        Eigen::Vector2i(0, 2),
        Eigen::Vector2i(-2, 0),
        Eigen::Vector2i(0, -2)
    };

    murosimproc::util::Array2d maze = murosimproc::util::Array2d<uint8_t>(size_x * 2 + 1, size_y * 2 + 1, 1);
    std::vector<Eigen::Vector2i> frontier = {
        Eigen::Vector2i(
            std::uniform_int_distribution<size_t>(0, size_x - 1)(random_generator) * 2 + 1,
            std::uniform_int_distribution<size_t>(0, size_y - 1)(random_generator) * 2 + 1
        )
    };

    murosimproc::util::image::at(maze, frontier.front()) = 0;

    std::vector<Eigen::Vector2i> neighbors;

    while (!frontier.empty())
    {
        const size_t index_frontier = std::uniform_int_distribution<size_t>(0, frontier.size() - 1)(random_generator);
        std::swap(frontier[index_frontier], frontier.back());
        const Eigen::Vector2i current = frontier.back();
        neighbors.clear();

        for (const Eigen::Vector2i &direction : directions)
        {
            const Eigen::Vector2i next = current + direction;
            if (0 <= next.x() && static_cast<size_t>(next.x()) < maze.sizeX() &&
                0 <= next.y() && static_cast<size_t>(next.y()) < maze.sizeY() &&
                murosimproc::util::image::at(maze, next) == 1)
            {
                neighbors.emplace_back(next);
            }
        }

        if (neighbors.empty())
        {
            frontier.pop_back();
        }
        else
        {
            const size_t index_neighbor = std::uniform_int_distribution<size_t>(0, neighbors.size() - 1)(random_generator);
            const Eigen::Vector2i next = neighbors[index_neighbor];
            const Eigen::Vector2i wall = current + (next - current) / 2;
            murosimproc::util::image::at(maze, next) = 0;
            murosimproc::util::image::at(maze, wall) = 0;
            frontier.emplace_back(next);
        }
    }

    return maze;
}

void removeWalls(
    murosimproc::util::Array2d<uint8_t> &maze,
    float rate,
    murosim::random::RandomGenerator &random_generator)
{
    assert(0 < maze.sizeX());
    assert(0 < maze.sizeY());

    if (rate <= 0.0f)
    {
        return;
    }

    std::uniform_real_distribution<float> unit_dist(0.0f, 1.0f);

    // Iterate over internal wall cells between passages.
    // In the Prim grid: odd coords are cell centers, even coords are walls/pillars.
    // Wall segments are at (even, odd) and (odd, even) — skip border (0 and max).
    const size_t sx = maze.sizeX();
    const size_t sy = maze.sizeY();

    for (size_t x = 1; x < sx - 1; ++x)
    {
        for (size_t y = 1; y < sy - 1; ++y)
        {
            // Only target wall cells between two passage cells
            const bool is_wall_segment = (x % 2 == 0) != (y % 2 == 0);
            if (!is_wall_segment || maze(x, y) == 0)
            {
                continue;
            }

            if (unit_dist(random_generator) < rate)
            {
                maze(x, y) = 0;
            }
        }
    }

    // Remove isolated pillar cells at (even, even) positions.
    // A pillar is isolated if all 4 adjacent wall segments are passages.
    for (size_t x = 2; x < sx - 1; x += 2)
    {
        for (size_t y = 2; y < sy - 1; y += 2)
        {
            if (maze(x, y) == 0)
            {
                continue;
            }

            const bool left  = maze(x - 1, y) == 0;
            const bool right = maze(x + 1, y) == 0;
            const bool up    = maze(x, y - 1) == 0;
            const bool down  = maze(x, y + 1) == 0;

            if (left && right && up && down)
            {
                maze(x, y) = 0;
            }
        }
    }
}

} // namespace murosimproc::maze
