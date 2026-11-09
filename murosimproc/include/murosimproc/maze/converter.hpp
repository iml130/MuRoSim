// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_MAZE_CONVERTER_HPP_
#define MUROSIMPROC_MAZE_CONVERTER_HPP_

#include "murosimproc/util/array2d.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/navigation/node.hpp"
#include "murosim/random/random_generator.hpp"

#include <vector>
#include <cstdint>

namespace murosimproc::maze
{

/**
 * @brief Direction indicators for tile point scanning
 *
 * Used to determine line segment directions when converting maze to geometry
 */
enum class TilePointDirection : uint8_t
{
    NONE,       ///< No wall corner point in this 2x2 tile (all four cells agree, or a diagonal pair).
    RIGHT_UP,   ///< Wall corner point offset towards the upper-right of the tile.
    RIGHT_DOWN, ///< Wall corner point offset towards the lower-right of the tile.
    LEFT_UP,    ///< Wall corner point offset towards the upper-left of the tile.
    LEFT_DOWN   ///< Wall corner point offset towards the lower-left of the tile.
};

/**
 * @brief Scan a 2x2 tile in the maze to determine line direction
 *
 * Analyzes a 2x2 block of maze cells to determine if there's a line segment
 * and in which direction it should be drawn.
 *
 * @param maze The maze grid
 * @param x X coordinate of top-left corner of 2x2 tile
 * @param y Y coordinate of top-left corner of 2x2 tile
 * @return Direction indicator for line drawing
 */
TilePointDirection scanMazeTilePointDirection(
    const murosimproc::util::Array2d<uint8_t> &maze,
    const size_t x,
    const size_t y
);

/**
 * @brief Convert maze grid to world geometry (lines and navigation nodes)
 *
 * Transforms a maze grid into:
 * - Line segments representing walls
 * - Navigation nodes at dead ends
 * - Border lines around the maze
 *
 * @param maze The maze grid to convert
 * @param lines Output vector for wall lines (cleared first)
 * @param circles Output vector for circular obstacles; cleared but otherwise unused for mazes
 * @param nodes Output vector for navigation nodes (cleared first); one is placed at every dead end, and if that is fewer than min_node_count, more are added at other passage cells (preferring open areas and corridors over junctions)
 * @param min_node_count Minimum number of navigation nodes to produce, topped up with non-dead-end passage cells if needed
 * @param space_ratio Fraction of a cell's half-width, in [0, 0.5), that wall corner points are offset inward by, controlling wall/corridor thickness
 * @param node_radius Radius (in grid-cell units) used both for placed navigation nodes and as the minimum separation between fallback nodes
 * @param random_generator Random number generator used to shuffle candidate fallback node positions
 */
void convertMazeToWorld(
    murosimproc::util::Array2d<uint8_t> &maze,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    std::vector<murosim::navigation::Node> &nodes,
    const size_t min_node_count,
    const float space_ratio,
    const float node_radius,
    murosim::random::RandomGenerator &random_generator
);

} // namespace murosimproc::maze

#endif // MUROSIMPROC_MAZE_CONVERTER_HPP_
