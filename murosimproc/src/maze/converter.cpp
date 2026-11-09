// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/maze/converter.hpp"
#include "murosimproc/util/image.hpp"
#include "murosim/math/constants.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <Eigen/Core>

namespace murosimproc::maze
{

TilePointDirection scanMazeTilePointDirection(
    const murosimproc::util::Array2d<uint8_t> &maze,
    const size_t x,
    const size_t y)
{
    assert(x < maze.sizeX() - 1);
    assert(y < maze.sizeY() - 1);

    constexpr const TilePointDirection lookupTable[16] = {
        TilePointDirection::NONE,         // 0b0000 ..
                                          //        ..
        TilePointDirection::RIGHT_DOWN,   // 0b0001 ..
                                          //        .#
        TilePointDirection::LEFT_DOWN,    // 0b0010 ..
                                          //        #.
        TilePointDirection::NONE,         // 0b0011 ..
                                          //        ##
        TilePointDirection::RIGHT_UP,     // 0b0100 .#
                                          //        ..
        TilePointDirection::NONE,         // 0b0101 .#
                                          //        .#
        TilePointDirection::NONE,         // 0b0110 .#
                                          //        #.
        TilePointDirection::RIGHT_DOWN,   // 0b0111 .#
                                          //        ##
        TilePointDirection::LEFT_UP,      // 0b1000 #.
                                          //        ..
        TilePointDirection::NONE,         // 0b1001 #.
                                          //        .#
        TilePointDirection::NONE,         // 0b1010 #.
                                          //        #.
        TilePointDirection::LEFT_DOWN,    // 0b1011 #.
                                          //        ##
        TilePointDirection::NONE,         // 0b1100 ##
                                          //        ..
        TilePointDirection::RIGHT_UP,     // 0b1101 ##
                                          //        .#
        TilePointDirection::LEFT_UP,      // 0b1110 ##
                                          //        #.
        TilePointDirection::NONE          // 0b1111 ##
                                          //        ##
    };

    const bool x0y0 = maze(x,     y    ) != 0;
    const bool x1y0 = maze(x + 1, y    ) != 0;
    const bool x0y1 = maze(x,     y + 1) != 0;
    const bool x1y1 = maze(x + 1, y + 1) != 0;
    const uint8_t code = static_cast<uint8_t>((x0y0 << 3) | (x1y0 << 2) | (x0y1 << 1) | x1y1);
    return lookupTable[code];
}

void convertMazeToWorld(
    murosimproc::util::Array2d<uint8_t> &maze,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    std::vector<murosim::navigation::Node> &nodes,
    const size_t min_node_count,
    const float space_ratio,
    const float node_radius,
    murosim::random::RandomGenerator &random_generator)
{
    lines.clear();
    circles.clear();
    nodes.clear();

    std::array<Eigen::Vector2f, 5> offset_points = {
        Eigen::Vector2f::Zero(),
        Eigen::Vector2f(0.5f + space_ratio, 0.5f - space_ratio),
        Eigen::Vector2f(0.5f + space_ratio, 0.5f + space_ratio),
        Eigen::Vector2f(0.5f - space_ratio, 0.5f - space_ratio),
        Eigen::Vector2f(0.5f - space_ratio, 0.5f + space_ratio)
    };

    const size_t size_x = maze.sizeX() - 1;
    const size_t size_y = maze.sizeY() - 1;

    // Add border
    lines.emplace_back(0, 0, 0, size_y);
    lines.emplace_back(0, size_y, size_x, size_y);
    lines.emplace_back(size_x, size_y, size_x, 0);
    lines.emplace_back(size_x, 0, 0, 0);

    // Scan each column (fixed x, vary y) then each row (fixed y, vary x).
    // First pass finds vertical wall segments; second finds horizontal ones.
    auto scanPass = [&](const bool swap_xy) {
        const size_t outer_end = swap_xy ? size_y : size_x;
        const size_t inner_end = swap_xy ? size_x : size_y;
        for (size_t outer = 0; outer < outer_end; ++outer)
        {
            Eigen::Vector2f point_start = Eigen::Vector2f::Zero();
            bool line_started = false;
            for (size_t inner = 0; inner < inner_end; ++inner)
            {
                const size_t x = swap_xy ? inner : outer;
                const size_t y = swap_xy ? outer : inner;
                const TilePointDirection direction = scanMazeTilePointDirection(maze, x, y);
                if (direction != TilePointDirection::NONE)
                {
                    if (!line_started)
                    {
                        line_started = true;
                        point_start = Eigen::Vector2f(x, y) + offset_points[static_cast<uint8_t>(direction)];
                    }
                    else
                    {
                        line_started = false;
                        const Eigen::Vector2f point_end = Eigen::Vector2f(x, y) + offset_points[static_cast<uint8_t>(direction)];
                        lines.emplace_back(point_start.x(), point_start.y(), point_end.x(), point_end.y());
                    }
                }
            }
        }
    };

    scanPass(false); // columns → vertical segments
    scanPass(true);  // rows → horizontal segments

    // Add navigation nodes at dead ends
    for (size_t y = 1; y < size_y; y++)
    {
        for (size_t x = 1; x < size_x; x++)
        {
            if (maze(x, y) == 0)
            {
                size_t sum_free_neighbors = 0;
                float direction = 0.0f;

                if (maze(x + 1, y) == 0)
                {
                    direction = 0.0f;
                    sum_free_neighbors++;
                }

                if (maze(x, y + 1) == 0)
                {
                    direction = murosim::math::PI_2;
                    sum_free_neighbors++;
                }

                if (maze(x - 1, y) == 0)
                {
                    direction = murosim::math::PI;
                    sum_free_neighbors++;
                }

                if (maze(x, y - 1) == 0)
                {
                    direction = -murosim::math::PI_2;
                    sum_free_neighbors++;
                }

                // Dead end: exactly one free neighbor
                if (sum_free_neighbors == 1)
                {
                    nodes.emplace_back(
                        murosim::navigation::Node(
                            static_cast<float>(x), static_cast<float>(y),
                            node_radius,
                            direction,
                            murosim::math::PI_4,
                            0, 0
                        )
                    );
                }
            }
        }
    }

    if (nodes.size() < min_node_count)
    {
        // Build a set of grid positions already occupied by dead-end nodes.
        auto encodePos = [](size_t x, size_t y) -> uint64_t {
            return (static_cast<uint64_t>(x) << 32) | static_cast<uint64_t>(y);
        };
        std::unordered_set<uint64_t> used_positions;
        for (const auto &n : nodes)
        {
            const size_t nx = static_cast<size_t>(n.getCircle().getPoint().x());
            const size_t ny = static_cast<size_t>(n.getCircle().getPoint().y());
            used_positions.insert(encodePos(nx, ny));
        }

        // Collect all passage cells that aren't already nodes, tiered so that
        // cells where a parked robot blocks other robots come last: open 2x2
        // areas (created by removeWalls) first, then plain corridor cells,
        // junctions (3+ free neighbours) at the very end.
        std::array<std::vector<Eigen::Vector2f>, 3> tiers;
        for (size_t y = 1; y < size_y; y++)
        {
            for (size_t x = 1; x < size_x; x++)
            {
                if (maze(x, y) == 0 && used_positions.find(encodePos(x, y)) == used_positions.end())
                {
                    const size_t free_neighbors =
                        static_cast<size_t>(maze(x + 1, y) == 0) + static_cast<size_t>(maze(x - 1, y) == 0) +
                        static_cast<size_t>(maze(x, y + 1) == 0) + static_cast<size_t>(maze(x, y - 1) == 0);

                    size_t tier = 2;
                    if (murosimproc::util::image::isNodeCandidate(maze, static_cast<int>(x), static_cast<int>(y)))
                    {
                        tier = 0;
                    }
                    else if (free_neighbors <= 2)
                    {
                        tier = 1;
                    }
                    tiers[tier].emplace_back(static_cast<float>(x), static_cast<float>(y));
                }
            }
        }

        // Shuffle within each tier and pick non-colliding candidates in tier order
        std::vector<Eigen::Vector2f> candidates;
        for (auto &tier : tiers)
        {
            std::shuffle(tier.begin(), tier.end(), random_generator);
            candidates.insert(candidates.end(), tier.begin(), tier.end());
        }
        const float min_sep_sq = 4.0f * node_radius * node_radius;

        for (const auto &c : candidates)
        {
            if (nodes.size() >= min_node_count)
            {
                break;
            }

            bool collides = false;
            for (const auto &n : nodes)
            {
                const float dx = c.x() - n.getCircle().getPoint().x();
                const float dy = c.y() - n.getCircle().getPoint().y();
                if (dx * dx + dy * dy < min_sep_sq)
                {
                    collides = true;
                    break;
                }
            }

            if (!collides)
            {
                nodes.emplace_back(
                    murosim::navigation::Node(
                        c.x(), c.y(),
                        node_radius,
                        0.0f,
                        murosim::math::PI,
                        0, 0
                    )
                );
            }
        }
    }
}

} // namespace murosimproc::maze
