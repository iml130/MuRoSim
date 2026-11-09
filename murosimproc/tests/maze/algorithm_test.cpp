// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/maze/algorithm.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

namespace
{
using murosimproc::maze::generateMazePrim;
using murosimproc::maze::removeWalls;
using murosimproc::util::Array2d;
using murosim::random::RandomGenerator;

constexpr size_t CELLS_X = 6;
constexpr size_t CELLS_Y = 4;
constexpr size_t GRID_X = 2 * CELLS_X + 1;
constexpr size_t GRID_Y = 2 * CELLS_Y + 1;
constexpr uint8_t WALL = 1;
constexpr uint8_t FREE = 0;
// Small enough that no wall segment is drawn (fixed seed, so this is
// deterministic); lets the pillar pass be tested on its own.
constexpr float RATE_NEGLIGIBLE = 1e-6f;

Array2d<uint8_t> primMaze(const uint64_t seed = fixtures::SEED)
{
    RandomGenerator random_generator(seed);
    return generateMazePrim(CELLS_X, CELLS_Y, random_generator);
}

size_t countFree(const Array2d<uint8_t> &maze)
{
    size_t count = 0;
    for (size_t y = 0; y < maze.sizeY(); y++)
    {
        for (size_t x = 0; x < maze.sizeX(); x++)
        {
            if (maze(x, y) == FREE)
            {
                count++;
            }
        }
    }
    return count;
}

/// Number of free cells reachable from (1, 1) through 4-neighbour moves.
size_t countReachable(const Array2d<uint8_t> &maze)
{
    std::vector<bool> visited(maze.sizeX() * maze.sizeY(), false);
    std::queue<std::pair<size_t, size_t>> frontier;
    frontier.emplace(1, 1);
    visited[1 + maze.sizeX()] = true;
    size_t reached = 0;
    while (!frontier.empty())
    {
        const auto [x, y] = frontier.front();
        frontier.pop();
        reached++;
        const std::pair<size_t, size_t> neighbours[4] = {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
        for (const auto &[nx, ny] : neighbours)
        {
            if (nx < maze.sizeX() && ny < maze.sizeY() && maze(nx, ny) == FREE && !visited[nx + ny * maze.sizeX()])
            {
                visited[nx + ny * maze.sizeX()] = true;
                frontier.emplace(nx, ny);
            }
        }
    }
    return reached;
}

bool isBorder(const Array2d<uint8_t> &maze, const size_t x, const size_t y)
{
    return x == 0 || y == 0 || x == maze.sizeX() - 1 || y == maze.sizeY() - 1;
}
} // namespace

// ============================================================================
// Prim Tests
// ============================================================================

TEST(MazeAlgorithmPrim, GridIsTwiceTheCellCountPlusOne)
{
    const Array2d<uint8_t> maze = primMaze();

    EXPECT_EQ(maze.sizeX(), GRID_X);
    EXPECT_EQ(maze.sizeY(), GRID_Y);
}

TEST(MazeAlgorithmPrim, SingleCellMazeIsOneFreeCellInsideWalls)
{
    RandomGenerator random_generator(fixtures::SEED);
    const Array2d<uint8_t> maze = generateMazePrim(1, 1, random_generator);

    ASSERT_EQ(maze.sizeX(), static_cast<size_t>(3));
    ASSERT_EQ(maze.sizeY(), static_cast<size_t>(3));
    EXPECT_EQ(countFree(maze), static_cast<size_t>(1));
    EXPECT_EQ(maze(1, 1), FREE);
}

TEST(MazeAlgorithmPrim, BorderIsAllWall)
{
    const Array2d<uint8_t> maze = primMaze();

    for (size_t y = 0; y < GRID_Y; y++)
    {
        for (size_t x = 0; x < GRID_X; x++)
        {
            if (isBorder(maze, x, y))
            {
                ASSERT_EQ(maze(x, y), WALL) << "cell " << x << ", " << y;
            }
        }
    }
}

TEST(MazeAlgorithmPrim, CellCentresAreFreeAndPillarsAreWalls)
{
    const Array2d<uint8_t> maze = primMaze();

    for (size_t y = 1; y < GRID_Y - 1; y++)
    {
        for (size_t x = 1; x < GRID_X - 1; x++)
        {
            if (x % 2 == 1 && y % 2 == 1)
            {
                ASSERT_EQ(maze(x, y), FREE) << "cell centre " << x << ", " << y;
            }
            else if (x % 2 == 0 && y % 2 == 0)
            {
                ASSERT_EQ(maze(x, y), WALL) << "pillar " << x << ", " << y;
            }
        }
    }
}

// A perfect maze is a spanning tree of the cells: n cells joined by exactly
// n - 1 carved wall segments.
TEST(MazeAlgorithmPrim, PerfectMazeCarvesCellsMinusOneSegments)
{
    const Array2d<uint8_t> maze = primMaze();

    EXPECT_EQ(countFree(maze), 2 * CELLS_X * CELLS_Y - 1);
}

TEST(MazeAlgorithmPrim, EveryFreeCellIsReachable)
{
    const Array2d<uint8_t> maze = primMaze();

    EXPECT_EQ(countReachable(maze), countFree(maze));
}

TEST(MazeAlgorithmPrim, SameSeedGivesTheSameMaze)
{
    const Array2d<uint8_t> first = primMaze(7);
    const Array2d<uint8_t> second = primMaze(7);

    for (size_t y = 0; y < GRID_Y; y++)
    {
        for (size_t x = 0; x < GRID_X; x++)
        {
            ASSERT_EQ(first(x, y), second(x, y)) << "cell " << x << ", " << y;
        }
    }
}

TEST(MazeAlgorithmPrim, DifferentSeedsGiveDifferentMazes)
{
    const Array2d<uint8_t> first = primMaze(7);
    const Array2d<uint8_t> second = primMaze(8);

    bool differs = false;
    for (size_t y = 0; y < GRID_Y; y++)
    {
        for (size_t x = 0; x < GRID_X; x++)
        {
            differs = differs || first(x, y) != second(x, y);
        }
    }
    EXPECT_TRUE(differs);
}

// ============================================================================
// Remove Walls Tests
// ============================================================================

TEST(MazeAlgorithmRemoveWalls, RateZeroLeavesTheMazeUntouched)
{
    const Array2d<uint8_t> original = primMaze();
    Array2d<uint8_t> maze = original;
    RandomGenerator random_generator(fixtures::SEED);

    removeWalls(maze, 0.0f, random_generator);

    for (size_t y = 0; y < GRID_Y; y++)
    {
        for (size_t x = 0; x < GRID_X; x++)
        {
            ASSERT_EQ(maze(x, y), original(x, y)) << "cell " << x << ", " << y;
        }
    }
}

TEST(MazeAlgorithmRemoveWalls, RateOneClearsTheWholeInteriorAndKeepsTheBorder)
{
    Array2d<uint8_t> maze = primMaze();
    RandomGenerator random_generator(fixtures::SEED);

    removeWalls(maze, 1.0f, random_generator);

    for (size_t y = 0; y < GRID_Y; y++)
    {
        for (size_t x = 0; x < GRID_X; x++)
        {
            ASSERT_EQ(maze(x, y), isBorder(maze, x, y) ? WALL : FREE) << "cell " << x << ", " << y;
        }
    }
}

TEST(MazeAlgorithmRemoveWalls, HalfRateRemovesSomeButNotAllWalls)
{
    Array2d<uint8_t> maze = primMaze();
    const size_t free_before = countFree(maze);
    RandomGenerator random_generator(fixtures::SEED);

    removeWalls(maze, 0.5f, random_generator);

    const size_t free_after = countFree(maze);
    EXPECT_GT(free_after, free_before);
    EXPECT_LT(free_after, (GRID_X - 2) * (GRID_Y - 2));
}

TEST(MazeAlgorithmRemoveWalls, OnlyEverTurnsWallsIntoPassages)
{
    const Array2d<uint8_t> original = primMaze();
    Array2d<uint8_t> maze = original;
    RandomGenerator random_generator(fixtures::SEED);

    removeWalls(maze, 0.5f, random_generator);

    for (size_t y = 0; y < GRID_Y; y++)
    {
        for (size_t x = 0; x < GRID_X; x++)
        {
            if (original(x, y) == FREE)
            {
                ASSERT_EQ(maze(x, y), FREE) << "cell " << x << ", " << y;
            }
        }
    }
    EXPECT_EQ(countReachable(maze), countFree(maze));
}

TEST(MazeAlgorithmRemoveWalls, PillarSurroundedByPassagesIsRemoved)
{
    // 2 x 2 cells with all four inner segments already open around pillar (2, 2)
    Array2d<uint8_t> maze = fixtures::makeImage({
        "#####",
        "#   #",
        "# # #",
        "#   #",
        "#####",
    });
    RandomGenerator random_generator(fixtures::SEED);

    removeWalls(maze, RATE_NEGLIGIBLE, random_generator);

    EXPECT_EQ(maze(2, 2), FREE);
    EXPECT_EQ(countFree(maze), static_cast<size_t>(9));
}

TEST(MazeAlgorithmRemoveWalls, PillarWithOneWallSegmentStays)
{
    Array2d<uint8_t> maze = fixtures::makeImage({
        "#####",
        "# # #",
        "# # #",
        "#   #",
        "#####",
    });
    RandomGenerator random_generator(fixtures::SEED);

    removeWalls(maze, RATE_NEGLIGIBLE, random_generator);

    EXPECT_EQ(maze(2, 2), WALL);
    EXPECT_EQ(maze(2, 1), WALL);
    EXPECT_EQ(countFree(maze), static_cast<size_t>(7));
}
