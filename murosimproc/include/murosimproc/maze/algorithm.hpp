// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_MAZE_ALGORITHM_HPP_
#define MUROSIMPROC_MAZE_ALGORITHM_HPP_

#include "murosimproc/util/array2d.hpp"
#include "murosim/random/random_generator.hpp"

namespace murosimproc::maze
{

/**
 * @brief Generate a maze using Prim's algorithm
 *
 * Creates a perfect maze (all cells reachable, no loops) using randomized Prim's algorithm.
 * The resulting maze grid uses:
 * - 0: passage/free space
 * - 1: wall
 *
 * @param size_x Number of cells in x direction (actual grid will be 2*size_x + 1)
 * @param size_y Number of cells in y direction (actual grid will be 2*size_y + 1)
 * @param random_generator Random number generator for maze generation
 * @return 2D array representing the maze grid
 */
murosimproc::util::Array2d<uint8_t> generateMazePrim(
    const size_t size_x,
    const size_t size_y,
    murosim::random::RandomGenerator &random_generator
);

/**
 * @brief Remove random internal walls from a maze bitmap to create loops.
 *
 * Iterates over internal wall cells (between passages) and randomly removes
 * them based on the given rate. This turns a perfect maze into one with
 * multiple paths and shortcuts.
 *
 * @param maze The maze grid to modify in-place
 * @param rate Fraction of eligible internal walls to remove [0, 1]
 * @param random_generator Random number generator
 */
void removeWalls(
    murosimproc::util::Array2d<uint8_t> &maze,
    float rate,
    murosim::random::RandomGenerator &random_generator
);

} // namespace murosimproc::maze

#endif // MUROSIMPROC_MAZE_ALGORITHM_HPP_
