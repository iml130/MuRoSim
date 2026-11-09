// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/maze/converter.hpp"
#include "murosimproc/maze/algorithm.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosimproc::maze::convertMazeToWorld;
using murosimproc::maze::scanMazeTilePointDirection;
using murosimproc::maze::TilePointDirection;
using murosimproc::util::Array2d;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::navigation::Node;
using murosim::random::RandomGenerator;

constexpr float SPACE_RATIO = 0.25f; // corner points sit 0.25 outside the cell boundary
constexpr float NODE_RADIUS = 0.4f;  // fallback nodes may sit on neighbouring cells (separation 0.8 < 1)
constexpr float NODE_RADIUS_WIDE = 0.6f; // neighbouring cells collide (separation 1.2 > 1)
constexpr size_t BORDER_LINES = 4;
constexpr uint32_t SEEDS = 20;

// Plus-shaped maze on a 7 x 7 grid: 4 dead ends, 4 corridor cells, 1 junction
// at (3, 3). Passages are ' ' (makeImage maps '.' to an obstacle zone).
const std::vector<std::string> PLUS_MAZE = {
    "#######",
    "### ###",
    "### ###",
    "#     #",
    "### ###",
    "### ###",
    "#######",
};

// A 2 x 2 tile from its 4-bit code: bit 3 = (0, 0), bit 2 = (1, 0),
// bit 1 = (0, 1), bit 0 = (1, 1), each set when the cell is a wall.
Array2d<uint8_t> tile(const uint8_t code)
{
    Array2d<uint8_t> maze(2, 2, static_cast<uint8_t>(0));
    maze(0, 0) = (code & 0b1000) ? 1 : 0;
    maze(1, 0) = (code & 0b0100) ? 1 : 0;
    maze(0, 1) = (code & 0b0010) ? 1 : 0;
    maze(1, 1) = (code & 0b0001) ? 1 : 0;
    return maze;
}

struct Conversion
{
    std::vector<Line> lines;
    std::vector<Circle> circles;
    std::vector<Node> nodes;
};

Conversion convert(Array2d<uint8_t> maze, const size_t min_node_count, const float node_radius = NODE_RADIUS,
                   const float space_ratio = SPACE_RATIO, const uint64_t seed = fixtures::SEED)
{
    Conversion result;
    RandomGenerator random_generator(seed);
    convertMazeToWorld(maze, result.lines, result.circles, result.nodes, min_node_count, space_ratio, node_radius,
                       random_generator);
    return result;
}

std::vector<Line> wallLines(const std::vector<Line> &lines)
{
    return std::vector<Line>(lines.begin() + BORDER_LINES, lines.end());
}

const Node *nodeAt(const std::vector<Node> &nodes, const float x, const float y)
{
    for (const Node &node : nodes)
    {
        if (std::abs(node.getX() - x) < fixtures::EPSILON && std::abs(node.getY() - y) < fixtures::EPSILON)
        {
            return &node;
        }
    }
    return nullptr;
}

void expectLine(const Line &line, const float x1, const float y1, const float x2, const float y2)
{
    EXPECT_FLOAT_EQ(line.getPoint1X(), x1);
    EXPECT_FLOAT_EQ(line.getPoint1Y(), y1);
    EXPECT_FLOAT_EQ(line.getPoint2X(), x2);
    EXPECT_FLOAT_EQ(line.getPoint2Y(), y2);
}
} // namespace

// ============================================================================
// Scan Tile Tests
// ============================================================================

TEST(MazeConverterScanTile, UniformTileHasNoCorner)
{
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0000), 0, 0), TilePointDirection::NONE);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1111), 0, 0), TilePointDirection::NONE);
}

TEST(MazeConverterScanTile, SingleWallCellPointsTowardsThatCell)
{
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1000), 0, 0), TilePointDirection::LEFT_UP);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0100), 0, 0), TilePointDirection::RIGHT_UP);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0010), 0, 0), TilePointDirection::LEFT_DOWN);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0001), 0, 0), TilePointDirection::RIGHT_DOWN);
}

TEST(MazeConverterScanTile, SingleFreeCellPointsAwayFromThatCell)
{
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0111), 0, 0), TilePointDirection::RIGHT_DOWN);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1011), 0, 0), TilePointDirection::LEFT_DOWN);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1101), 0, 0), TilePointDirection::RIGHT_UP);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1110), 0, 0), TilePointDirection::LEFT_UP);
}

TEST(MazeConverterScanTile, StraightEdgesHaveNoCorner)
{
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0011), 0, 0), TilePointDirection::NONE);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1100), 0, 0), TilePointDirection::NONE);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0101), 0, 0), TilePointDirection::NONE);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1010), 0, 0), TilePointDirection::NONE);
}

TEST(MazeConverterScanTile, DiagonalPairsHaveNoCorner)
{
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b0110), 0, 0), TilePointDirection::NONE);
    EXPECT_EQ(scanMazeTilePointDirection(tile(0b1001), 0, 0), TilePointDirection::NONE);
}

TEST(MazeConverterScanTile, ScansTheTileAtTheGivenOffset)
{
    const Array2d<uint8_t> maze = fixtures::makeImage(PLUS_MAZE);

    EXPECT_EQ(scanMazeTilePointDirection(maze, 0, 0), TilePointDirection::NONE);   // all wall
    EXPECT_EQ(scanMazeTilePointDirection(maze, 2, 0), TilePointDirection::LEFT_UP); // (3, 1) is the only free cell
    EXPECT_EQ(scanMazeTilePointDirection(maze, 2, 2), TilePointDirection::LEFT_UP); // (2, 2) is the only wall
    EXPECT_EQ(scanMazeTilePointDirection(maze, 3, 3), TilePointDirection::RIGHT_DOWN);
    EXPECT_EQ(scanMazeTilePointDirection(maze, 3, 1), TilePointDirection::NONE);   // free column, straight edge
}

// ============================================================================
// Border Tests
// ============================================================================

TEST(MazeConverterBorder, FirstFourLinesFrameTheGrid)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 0);

    ASSERT_GE(result.lines.size(), BORDER_LINES);
    expectLine(result.lines[0], 0.0f, 0.0f, 0.0f, 6.0f);
    expectLine(result.lines[1], 0.0f, 6.0f, 6.0f, 6.0f);
    expectLine(result.lines[2], 6.0f, 6.0f, 6.0f, 0.0f);
    expectLine(result.lines[3], 6.0f, 0.0f, 0.0f, 0.0f);
}

TEST(MazeConverterBorder, BorderSpansGridSizeMinusOne)
{
    RandomGenerator random_generator(fixtures::SEED);
    const Array2d<uint8_t> maze = murosimproc::maze::generateMazePrim(5, 3, random_generator); // 11 x 7 grid
    const Conversion result = convert(maze, 0);

    ASSERT_GE(result.lines.size(), BORDER_LINES);
    const std::vector<Line> border(result.lines.begin(), result.lines.begin() + BORDER_LINES);
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::closedLoops(border);
    ASSERT_EQ(loops.size(), static_cast<size_t>(1));
    EXPECT_NEAR(std::abs(fixtures::signedArea(loops[0])), 10.0f * 6.0f, fixtures::EPSILON);
}

// ============================================================================
// Wall Line Tests
// ============================================================================

TEST(MazeConverterLines, OutputsAreClearedFirst)
{
    Conversion result;
    result.lines.emplace_back(9.0f, 9.0f, 8.0f, 8.0f);
    result.circles.emplace_back(1.0f, 1.0f, 1.0f);
    result.nodes.emplace_back(1.0f, 1.0f, 1.0f);
    Array2d<uint8_t> maze = fixtures::makeImage(PLUS_MAZE);
    RandomGenerator random_generator(fixtures::SEED);

    convertMazeToWorld(maze, result.lines, result.circles, result.nodes, 0, SPACE_RATIO, NODE_RADIUS,
                       random_generator);

    EXPECT_EQ(result.lines.size(), BORDER_LINES + 12);
    EXPECT_TRUE(result.circles.empty());
    EXPECT_EQ(result.nodes.size(), static_cast<size_t>(4));
}

// A single free cell becomes a square of side 1 + 2 * space_ratio around it.
TEST(MazeConverterLines, SingleCellMazeYieldsOneSquare)
{
    const Conversion result = convert(fixtures::makeImage({"###", "# #", "###"}), 0);

    ASSERT_EQ(result.lines.size(), BORDER_LINES + 4);
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::undirectedLoops(wallLines(result.lines));
    ASSERT_EQ(loops.size(), static_cast<size_t>(1));
    ASSERT_EQ(loops[0].size(), static_cast<size_t>(4));
    const float side = 1.0f + 2.0f * SPACE_RATIO;
    EXPECT_NEAR(std::abs(fixtures::signedArea(loops[0])), side * side, fixtures::EPSILON);
    for (size_t i = 0; i < loops[0].size(); i++)
    {
        EXPECT_TRUE(loops[0][i].x() == 0.5f - SPACE_RATIO || loops[0][i].x() == 1.5f + SPACE_RATIO) << "point " << i;
        EXPECT_TRUE(loops[0][i].y() == 0.5f - SPACE_RATIO || loops[0][i].y() == 1.5f + SPACE_RATIO) << "point " << i;
    }
}

// The plus is two bars of 5 x 1 cells (inflated by space_ratio on every
// side) minus their overlap, outlined by 12 corner points.
TEST(MazeConverterLines, PlusMazeOutlineIsOneLoopWithTwelveCorners)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 0);

    ASSERT_EQ(result.lines.size(), BORDER_LINES + 12);
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::undirectedLoops(wallLines(result.lines));
    ASSERT_EQ(loops.size(), static_cast<size_t>(1));
    EXPECT_EQ(loops[0].size(), static_cast<size_t>(12));
    const float bar_width = 1.0f + 2.0f * SPACE_RATIO;
    const float bar_length = 5.0f + 2.0f * SPACE_RATIO;
    const float expected_area = 2.0f * bar_length * bar_width - bar_width * bar_width;
    EXPECT_NEAR(std::abs(fixtures::signedArea(loops[0])), expected_area, fixtures::EPSILON);
}

TEST(MazeConverterLines, WallLinesAreAxisAlignedAndInsideTheBorder)
{
    RandomGenerator random_generator(fixtures::SEED);
    const Array2d<uint8_t> maze = murosimproc::maze::generateMazePrim(6, 4, random_generator); // 13 x 9 grid
    const Conversion result = convert(maze, 0);

    const std::vector<Line> walls = wallLines(result.lines);
    ASSERT_GT(walls.size(), static_cast<size_t>(20));
    for (size_t i = 0; i < walls.size(); i++)
    {
        ASSERT_TRUE(walls[i].getPoint1X() == walls[i].getPoint2X() || walls[i].getPoint1Y() == walls[i].getPoint2Y())
            << "line " << i;
        ASSERT_GT(walls[i].getLength(), 0.0f) << "line " << i;
        for (const Eigen::Vector2f &point : {walls[i].getPoint1(), walls[i].getPoint2()})
        {
            ASSERT_GE(point.x(), 0.0f) << "line " << i;
            ASSERT_LE(point.x(), 12.0f) << "line " << i;
            ASSERT_GE(point.y(), 0.0f) << "line " << i;
            ASSERT_LE(point.y(), 8.0f) << "line " << i;
        }
    }
}

// A perfect maze is one connected free region without holes, so its wall
// lines chain into exactly one loop. With space_ratio 0 the corner points
// lie on the cell boundaries and the enclosed area is the free cell count.
TEST(MazeConverterLines, PerfectMazeOutlineIsOneLoopEnclosingTheFreeCells)
{
    RandomGenerator random_generator(fixtures::SEED);
    const Array2d<uint8_t> maze = murosimproc::maze::generateMazePrim(6, 4, random_generator);
    const Conversion result = convert(maze, 0, NODE_RADIUS, 0.0f);

    const std::vector<Line> walls = wallLines(result.lines);
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::undirectedLoops(walls);
    ASSERT_EQ(loops.size(), static_cast<size_t>(1));
    EXPECT_EQ(loops[0].size(), walls.size());
    const float free_cells = static_cast<float>(2 * 6 * 4 - 1); // spanning tree: cells + carved segments
    EXPECT_NEAR(std::abs(fixtures::signedArea(loops[0])), free_cells, fixtures::EPSILON);
}

// ============================================================================
// Node Tests
// ============================================================================

TEST(MazeConverterNodes, DeadEndsAlwaysGetANode)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 0);

    ASSERT_EQ(result.nodes.size(), static_cast<size_t>(4));
    EXPECT_NE(nodeAt(result.nodes, 3.0f, 1.0f), nullptr);
    EXPECT_NE(nodeAt(result.nodes, 3.0f, 5.0f), nullptr);
    EXPECT_NE(nodeAt(result.nodes, 1.0f, 3.0f), nullptr);
    EXPECT_NE(nodeAt(result.nodes, 5.0f, 3.0f), nullptr);
}

TEST(MazeConverterNodes, DeadEndNodesFaceTheirOpenSide)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 0);

    ASSERT_EQ(result.nodes.size(), static_cast<size_t>(4));
    EXPECT_FLOAT_EQ(nodeAt(result.nodes, 3.0f, 1.0f)->getStartAngle(), PI_2);
    EXPECT_FLOAT_EQ(nodeAt(result.nodes, 3.0f, 5.0f)->getStartAngle(), -PI_2);
    EXPECT_FLOAT_EQ(nodeAt(result.nodes, 1.0f, 3.0f)->getStartAngle(), 0.0f);
    EXPECT_FLOAT_EQ(nodeAt(result.nodes, 5.0f, 3.0f)->getStartAngle(), PI);
    for (size_t i = 0; i < result.nodes.size(); i++)
    {
        EXPECT_FLOAT_EQ(result.nodes[i].getStartBias(), PI_4) << "node " << i;
        EXPECT_FLOAT_EQ(result.nodes[i].getRadius(), NODE_RADIUS) << "node " << i;
        EXPECT_EQ(result.nodes[i].getGroup(), static_cast<size_t>(0)) << "node " << i;
    }
}

TEST(MazeConverterNodes, FallbackNodesFaceAnywhere)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 6);

    ASSERT_EQ(result.nodes.size(), static_cast<size_t>(6));
    for (size_t i = 4; i < result.nodes.size(); i++)
    {
        EXPECT_FLOAT_EQ(result.nodes[i].getStartAngle(), 0.0f) << "node " << i;
        EXPECT_FLOAT_EQ(result.nodes[i].getStartBias(), PI) << "node " << i;
        EXPECT_FLOAT_EQ(result.nodes[i].getRadius(), NODE_RADIUS) << "node " << i;
    }
}

TEST(MazeConverterNodes, FallbackPrefersCorridorsOverTheJunction)
{
    const Array2d<uint8_t> maze = fixtures::makeImage(PLUS_MAZE);

    for (uint32_t seed = 0; seed < SEEDS; seed++)
    {
        // 4 dead ends + 4 fallback nodes use every corridor cell before the junction
        const Conversion result = convert(maze, 8, NODE_RADIUS, SPACE_RATIO, seed);
        ASSERT_EQ(result.nodes.size(), static_cast<size_t>(8)) << "seed " << seed;
        ASSERT_EQ(nodeAt(result.nodes, 3.0f, 3.0f), nullptr) << "seed " << seed;
    }
}

TEST(MazeConverterNodes, JunctionIsUsedAsLastResort)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 9);

    ASSERT_EQ(result.nodes.size(), static_cast<size_t>(9));
    EXPECT_NE(nodeAt(result.nodes, 3.0f, 3.0f), nullptr);
}

TEST(MazeConverterNodes, NodesLieOnDistinctPassageCells)
{
    const Array2d<uint8_t> maze = fixtures::makeImage(PLUS_MAZE);
    const Conversion result = convert(maze, 9);

    ASSERT_EQ(result.nodes.size(), static_cast<size_t>(9));
    for (size_t i = 0; i < result.nodes.size(); i++)
    {
        const size_t x = static_cast<size_t>(result.nodes[i].getX());
        const size_t y = static_cast<size_t>(result.nodes[i].getY());
        EXPECT_FLOAT_EQ(result.nodes[i].getX(), static_cast<float>(x)) << "node " << i;
        EXPECT_FLOAT_EQ(result.nodes[i].getY(), static_cast<float>(y)) << "node " << i;
        EXPECT_EQ(maze(x, y), static_cast<uint8_t>(0)) << "node " << i;
        for (size_t j = 0; j < i; j++)
        {
            EXPECT_FALSE(x == static_cast<size_t>(result.nodes[j].getX()) &&
                         y == static_cast<size_t>(result.nodes[j].getY()))
                << "nodes " << i << " and " << j;
        }
    }
}

TEST(MazeConverterNodes, RequestBeyondThePassageCellsStopsAtTheCells)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 50);

    EXPECT_EQ(result.nodes.size(), static_cast<size_t>(9));
}

// Fallback nodes keep 2 * node_radius apart: with 0.6 every corridor cell is
// next to a dead end, so only the junction (2 cells away) can be added.
TEST(MazeConverterNodes, FallbackKeepsTwoRadiiFromExistingNodes)
{
    const Conversion result = convert(fixtures::makeImage(PLUS_MAZE), 9, NODE_RADIUS_WIDE);

    ASSERT_EQ(result.nodes.size(), static_cast<size_t>(5));
    EXPECT_NE(nodeAt(result.nodes, 3.0f, 3.0f), nullptr);
}
