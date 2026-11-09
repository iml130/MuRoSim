// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/grid/map.hpp"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::grid::Map;

constexpr float EPSILON = 1e-5f;

// The test world spans [-5, 5] x [-5, 5] m. With GRID_SCALE it becomes a
// SIZE x SIZE map; BUFFER_SCALE grows a BUFFER_CELLS wide band around
// obstacles. World x grows with map x, world y grows against map y:
//   cell x = (wx + 5) * 10,   cell y = 100 - (wy + 5) * 10
constexpr float WORLD_HALF_SIZE = 5.0f;
constexpr float GRID_SCALE = 0.1f;
constexpr float BUFFER_SCALE = 0.2f;
constexpr size_t SIZE = 101;
constexpr size_t BUFFER_CELLS = 2;

size_t cellX(const float wx)
{
    return static_cast<size_t>(std::lround((wx + WORLD_HALF_SIZE) / GRID_SCALE));
}

size_t cellY(const float wy)
{
    return static_cast<size_t>(std::lround((WORLD_HALF_SIZE - wy) / GRID_SCALE));
}

/// The test world: a square border, a wall from (0, -5) up to (0, 0) and on
/// to (5, 0) that closes off the lower right quadrant, a rectangular obstacle
/// [1, 3] x [1, 3], a diagonal cross around (-2.5, -2.5) and a circle of
/// radius 0.5 at (-2.5, 2.5).
murosim::geometry::World testWorld()
{
    murosim::geometry::Shape shape;

    // border
    shape.addLine(-5.0f, -5.0f, 5.0f, -5.0f);
    shape.addLine(5.0f, -5.0f, 5.0f, 5.0f);
    shape.addLine(5.0f, 5.0f, -5.0f, 5.0f);
    shape.addLine(-5.0f, 5.0f, -5.0f, -5.0f);

    // wall attached to the border
    shape.addLine(0.0f, -5.0f, 0.0f, 0.0f);
    shape.addLine(0.0f, 0.0f, 5.0f, 0.0f);

    // rectangular obstacle
    shape.addLine(1.0f, 1.0f, 3.0f, 1.0f);
    shape.addLine(3.0f, 1.0f, 3.0f, 3.0f);
    shape.addLine(3.0f, 3.0f, 1.0f, 3.0f);
    shape.addLine(1.0f, 3.0f, 1.0f, 1.0f);

    // diagonal cross
    shape.addLine(-3.0f, -3.0f, -2.0f, -2.0f);
    shape.addLine(-3.0f, -2.0f, -2.0f, -3.0f);

    // circle
    shape.addCircle(-2.5f, 2.5f, 0.5f);

    return murosim::geometry::World(shape);
}

/// Asserts that the cells strictly inside the outline [x0, x1] x [y0, y1]
/// are FREE except for the BUFFER band along the outline, and that every
/// cell on or outside the outline is an obstacle (OCCUPIED or UNREACHABLE).
void assertOnlyRegionIsFree(const Map &map, const size_t x0, const size_t x1, const size_t y0, const size_t y1)
{
    for (size_t y = 0; y < map.sizeY(); y++)
    {
        for (size_t x = 0; x < map.sizeX(); x++)
        {
            const bool inside = x0 < x && x < x1 && y0 < y && y < y1;
            const bool free = x0 + BUFFER_CELLS < x && x < x1 - BUFFER_CELLS &&
                              y0 + BUFFER_CELLS < y && y < y1 - BUFFER_CELLS;

            if (free)
            {
                ASSERT_TRUE(map.isFree(x, y)) << "cell " << x << ", " << y;
            }
            else if (inside)
            {
                ASSERT_TRUE(map.isBuffer(x, y)) << "cell " << x << ", " << y;
            }
            else
            {
                ASSERT_TRUE(map.isObstacle(x, y)) << "cell " << x << ", " << y;
            }
        }
    }
}

/// A 1 m x 1 m box; with a buffer wider than half of it no FREE cell remains.
murosim::geometry::World smallBox()
{
    murosim::geometry::Shape shape;
    shape.addLine(0.0f, 0.0f, 1.0f, 0.0f);
    shape.addLine(1.0f, 0.0f, 1.0f, 1.0f);
    shape.addLine(1.0f, 1.0f, 0.0f, 1.0f);
    shape.addLine(0.0f, 1.0f, 0.0f, 0.0f);
    return murosim::geometry::World(shape);
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class GridMapTest : public ::testing::Test
{
protected:
    // No seed points: the FREE region is the largest connected one
    Map map{testWorld(), {}, GRID_SCALE, BUFFER_SCALE};
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(GridMapTest, NoPointsEqualsPointInTheLargestRegion)
{
    const Map map_point_middle(testWorld(), {{0.0f, 0.0f}}, GRID_SCALE, BUFFER_SCALE);

    ASSERT_EQ(map_point_middle.size(), map.size());
    for (size_t i = 0; i < map.size(); i++)
    {
        ASSERT_EQ(map_point_middle[i], map[i]) << "index " << i;
    }
}

TEST_F(GridMapTest, PointInsideObstacleRectangleFreesOnlyItsInterior)
{
    // The rectangle [1, 3] x [1, 3] encloses the seed point (2, 2)
    const Map map_point_obstacle(testWorld(), {{2.0f, 2.0f}}, GRID_SCALE, BUFFER_SCALE);

    assertOnlyRegionIsFree(map_point_obstacle, cellX(1.0f), cellX(3.0f), cellY(3.0f), cellY(1.0f));
}

TEST_F(GridMapTest, PointInsideWalledQuadrantFreesOnlyThatQuadrant)
{
    // The wall and the border enclose the lower right quadrant [0, 5] x [-5, 0]
    const Map map_point_wall(testWorld(), {{2.5f, -2.5f}}, GRID_SCALE, BUFFER_SCALE);

    assertOnlyRegionIsFree(map_point_wall, cellX(0.0f), cellX(5.0f), cellY(0.0f), cellY(-5.0f));
}

TEST_F(GridMapTest, PointInsideCircleFallsBackToLargestRegion)
{
    // A seed point on an obstacle cell is ignored: the map is created as if no
    // points were given
    const Map map_point_circle(testWorld(), {{-2.5f, 2.5f}}, GRID_SCALE, BUFFER_SCALE);

    ASSERT_EQ(map_point_circle.size(), map.size());
    for (size_t i = 0; i < map.size(); i++)
    {
        ASSERT_EQ(map_point_circle[i], map[i]) << "index " << i;
    }
}

TEST_F(GridMapTest, PointWhoseRegionTheBufferSwallowsThrows)
{
    // The seed point sits on a FREE cell, but a 0.6 m buffer in a 1 m box
    // turns every cell into BUFFER, so no FREE region survives
    EXPECT_THROW(Map(smallBox(), {{0.5f, 0.5f}}, GRID_SCALE, 0.6f), std::runtime_error);
}

TEST_F(GridMapTest, PointSurvivesABufferThatLeavesFreeCells)
{
    const Map map_small(smallBox(), {{0.5f, 0.5f}}, GRID_SCALE, BUFFER_SCALE);

    EXPECT_TRUE(map_small.isFree(5, 5)); // the centre cell
}

// ============================================================================
// Getter Tests
// ============================================================================

TEST_F(GridMapTest, Getter)
{
    EXPECT_FLOAT_EQ(map.getGridScale(), GRID_SCALE);
    EXPECT_FLOAT_EQ(map.getGridScaleInv(), 1.0f / GRID_SCALE);
    EXPECT_FLOAT_EQ(map.getBufferScale(), BUFFER_SCALE);

    EXPECT_FLOAT_EQ(map.getOrigin().x(), -WORLD_HALF_SIZE);
    EXPECT_FLOAT_EQ(map.getOrigin().y(), -WORLD_HALF_SIZE);
    EXPECT_FLOAT_EQ(map.getLength().x(), 2.0f * WORLD_HALF_SIZE);
    EXPECT_FLOAT_EQ(map.getLength().y(), 2.0f * WORLD_HALF_SIZE);

    EXPECT_EQ(map.size(), SIZE * SIZE);
    EXPECT_EQ(map.sizeX(), SIZE);
    EXPECT_EQ(map.sizeY(), SIZE);
}

// ============================================================================
// Setter Tests
// ============================================================================

TEST_F(GridMapTest, Setter)
{
    for (size_t y = 0; y < map.sizeY(); y++)
    {
        for (size_t x = 0; x < map.sizeX(); x++)
        {
            const Eigen::Vector2i point(static_cast<int>(x), static_cast<int>(y));

            map.set(Map::FIELD::OCCUPIED, static_cast<int>(x), static_cast<int>(y));
            ASSERT_EQ(map(x, y), Map::FIELD::OCCUPIED) << "cell " << x << ", " << y;

            map.set(Map::FIELD::FREE, point);
            ASSERT_EQ(map(x, y), Map::FIELD::FREE) << "cell " << x << ", " << y;
        }
    }
}

// ============================================================================
// Index Conversion Tests
// ============================================================================

TEST_F(GridMapTest, IndexAndPointConvertBothWays)
{
    for (size_t i = 0; i < map.size(); i++)
    {
        const Eigen::Vector2i point = map.indexToPoint(i);
        ASSERT_EQ(map.pointToIndex(point), i) << "index " << i;
        ASSERT_EQ(static_cast<size_t>(point.x()) + static_cast<size_t>(point.y()) * map.sizeX(), i) << "index " << i;

        int x;
        int y;
        map.indexToPoint(i, x, y);
        ASSERT_EQ(x, point.x()) << "index " << i;
        ASSERT_EQ(y, point.y()) << "index " << i;
        ASSERT_EQ(map.pointToIndex(x, y), i) << "index " << i;
    }
}

// ============================================================================
// Boundary Check Tests
// ============================================================================

TEST_F(GridMapTest, IsInMap)
{
    const int size = static_cast<int>(SIZE);

    // Signed coordinates, a margin of 5 cells around the map
    for (int y = -5; y <= size + 5; y++)
    {
        for (int x = -5; x <= size + 5; x++)
        {
            const bool inside = 0 <= x && x < size && 0 <= y && y < size;
            ASSERT_EQ(map.isInMap(x, y), inside) << "cell " << x << ", " << y;
            ASSERT_EQ(map.isInMap(Eigen::Vector2i(x, y)), inside) << "cell " << x << ", " << y;
        }
    }

    // Unsigned coordinates
    for (size_t y = 0; y <= SIZE + 5; y++)
    {
        for (size_t x = 0; x <= SIZE + 5; x++)
        {
            const bool inside = x < SIZE && y < SIZE;
            ASSERT_EQ(map.isInMap(x, y), inside) << "cell " << x << ", " << y;
        }
    }
}

// ============================================================================
// Field State Tests
// ============================================================================

TEST_F(GridMapTest, IsFree)
{
    for (size_t i = 0; i < map.size(); i++)
    {
        ASSERT_EQ(map.isFree(i), map[i] == Map::FIELD::FREE) << "index " << i;
    }
}

TEST_F(GridMapTest, IsBuffer)
{
    for (size_t i = 0; i < map.size(); i++)
    {
        ASSERT_EQ(map.isBuffer(i), map[i] == Map::FIELD::BUFFER) << "index " << i;
    }
}

TEST_F(GridMapTest, IsObstacle)
{
    // Both drawn obstacles and unreachable free space count as obstacles
    for (size_t i = 0; i < map.size(); i++)
    {
        const bool obstacle = map[i] == Map::FIELD::OCCUPIED || map[i] == Map::FIELD::UNREACHABLE;
        ASSERT_EQ(map.isObstacle(i), obstacle) << "index " << i;
    }
}

// ============================================================================
// Operator Tests
// ============================================================================

TEST_F(GridMapTest, OperatorBracketsMatchesOperatorParentheses)
{
    size_t index = 0;
    for (size_t y = 0; y < map.sizeY(); y++)
    {
        for (size_t x = 0; x < map.sizeX(); x++)
        {
            ASSERT_EQ(map(x, y), map[index]) << "cell " << x << ", " << y;
            index++;
        }
    }
}

TEST_F(GridMapTest, OperatorBracketsWrites)
{
    for (size_t i = 0; i < map.size(); i++)
    {
        map[i] = Map::FIELD::BUFFER;
        ASSERT_EQ(map[i], Map::FIELD::BUFFER) << "index " << i;
    }
}

TEST_F(GridMapTest, OperatorParenthesesReadsCreatedMap)
{
    const size_t last = SIZE - 1;

    // The border is OCCUPIED
    for (size_t i = 0; i < SIZE; i++)
    {
        ASSERT_EQ(map(i, 0), Map::FIELD::OCCUPIED) << "x " << i;
        ASSERT_EQ(map(i, last), Map::FIELD::OCCUPIED) << "x " << i;
        ASSERT_EQ(map(0, i), Map::FIELD::OCCUPIED) << "y " << i;
        ASSERT_EQ(map(Eigen::Vector2i(static_cast<int>(last), static_cast<int>(i))), Map::FIELD::OCCUPIED) << "y " << i;
    }

    // The row and the column next to the border are BUFFER
    for (size_t i = 1; i < last; i++)
    {
        ASSERT_EQ(map(i, 1), Map::FIELD::BUFFER) << "x " << i;
        ASSERT_EQ(map(Eigen::Vector2i(1, static_cast<int>(i))), Map::FIELD::BUFFER) << "y " << i;
    }

    // The walled lower right quadrant is UNREACHABLE
    for (size_t y = cellY(0.0f) + 1; y < last; y++)
    {
        for (size_t x = cellX(0.0f) + 1; x < last; x++)
        {
            ASSERT_EQ(map(x, y), Map::FIELD::UNREACHABLE) << "cell " << x << ", " << y;
        }
    }
}

TEST_F(GridMapTest, OperatorParenthesesWrites)
{
    for (size_t y = 0; y < map.sizeY(); y++)
    {
        for (size_t x = 0; x < map.sizeX(); x++)
        {
            const Eigen::Vector2i point(static_cast<int>(x), static_cast<int>(y));

            map(x, y) = Map::FIELD::OCCUPIED;
            ASSERT_EQ(map(x, y), Map::FIELD::OCCUPIED) << "cell " << x << ", " << y;

            map(point) = Map::FIELD::UNREACHABLE;
            ASSERT_EQ(map(point), Map::FIELD::UNREACHABLE) << "cell " << x << ", " << y;
        }
    }
}

// ============================================================================
// Coordinate Conversion Tests
// ============================================================================

TEST_F(GridMapTest, FromMapIntoWorldIntoMapIsIdentity)
{
    for (size_t y = 0; y < map.sizeY(); y++)
    {
        for (size_t x = 0; x < map.sizeX(); x++)
        {
            const int cx = static_cast<int>(x);
            const int cy = static_cast<int>(y);
            const Eigen::Vector2i point(cx, cy);
            float wx;
            float wy;
            int mx;
            int my;
            Eigen::Vector2f world_point;
            Eigen::Vector2i map_point;

            // ints via floats
            map.fromMapIntoWorld(cx, cy, wx, wy);
            map.fromWorldIntoMap(wx, wy, mx, my);
            ASSERT_EQ(mx, cx) << "cell " << x << ", " << y;
            ASSERT_EQ(my, cy) << "cell " << x << ", " << y;

            // ints via Vector2f
            map.fromMapIntoWorld(cx, cy, world_point);
            map.fromWorldIntoMap(world_point, mx, my);
            ASSERT_EQ(mx, cx) << "cell " << x << ", " << y;
            ASSERT_EQ(my, cy) << "cell " << x << ", " << y;

            // Vector2i via floats
            map.fromMapIntoWorld(point, wx, wy);
            map.fromWorldIntoMap(wx, wy, map_point);
            ASSERT_EQ(map_point, point) << "cell " << x << ", " << y;

            // Vector2i via Vector2f
            map.fromMapIntoWorld(point, world_point);
            map.fromWorldIntoMap(world_point, map_point);
            ASSERT_EQ(map_point, point) << "cell " << x << ", " << y;
        }
    }
}

TEST_F(GridMapTest, FromWorldIntoMapIntoWorldIsIdentityOnCellCenters)
{
    // Whole metres are exact cell centres, so the round trip is lossless
    for (float y = -WORLD_HALF_SIZE; y <= WORLD_HALF_SIZE; y += 1.0f)
    {
        for (float x = -WORLD_HALF_SIZE; x <= WORLD_HALF_SIZE; x += 1.0f)
        {
            const Eigen::Vector2f point(x, y);
            float wx;
            float wy;
            int mx;
            int my;
            Eigen::Vector2f world_point;
            Eigen::Vector2i map_point;

            // floats via ints
            map.fromWorldIntoMap(x, y, mx, my);
            map.fromMapIntoWorld(mx, my, wx, wy);
            EXPECT_NEAR(wx, x, EPSILON);
            EXPECT_NEAR(wy, y, EPSILON);

            // floats via Vector2i
            map.fromWorldIntoMap(x, y, map_point);
            map.fromMapIntoWorld(map_point, wx, wy);
            EXPECT_NEAR(wx, x, EPSILON);
            EXPECT_NEAR(wy, y, EPSILON);

            // Vector2f via ints
            map.fromWorldIntoMap(point, mx, my);
            map.fromMapIntoWorld(mx, my, world_point);
            EXPECT_NEAR(world_point.x(), x, EPSILON);
            EXPECT_NEAR(world_point.y(), y, EPSILON);

            // Vector2f via Vector2i
            map.fromWorldIntoMap(point, map_point);
            map.fromMapIntoWorld(map_point, world_point);
            EXPECT_NEAR(world_point.x(), x, EPSILON);
            EXPECT_NEAR(world_point.y(), y, EPSILON);
        }
    }
}

TEST_F(GridMapTest, FromWorldIntoMapFloatOverloads)
{
    // The continuous map coordinate is the world coordinate shifted by the
    // origin and scaled, with y flipped; sampled every cell. The expectation
    // multiplies by the inverse scale like the implementation, so that float
    // rounding matches.
    for (float y = -WORLD_HALF_SIZE; y <= WORLD_HALF_SIZE; y += GRID_SCALE)
    {
        for (float x = -WORLD_HALF_SIZE; x <= WORLD_HALF_SIZE; x += GRID_SCALE)
        {
            const Eigen::Vector2f world_point(x, y);
            const float expected_x = (x + WORLD_HALF_SIZE) * map.getGridScaleInv();
            const float expected_y = static_cast<float>(SIZE - 1) - (y + WORLD_HALF_SIZE) * map.getGridScaleInv();
            float mx;
            float my;
            Eigen::Vector2f map_point;

            map.fromWorldIntoMap(x, y, mx, my);
            ASSERT_FLOAT_EQ(mx, expected_x) << "world " << x << ", " << y;
            ASSERT_FLOAT_EQ(my, expected_y) << "world " << x << ", " << y;

            map.fromWorldIntoMap(x, y, map_point);
            ASSERT_FLOAT_EQ(map_point.x(), expected_x) << "world " << x << ", " << y;
            ASSERT_FLOAT_EQ(map_point.y(), expected_y) << "world " << x << ", " << y;

            map.fromWorldIntoMap(world_point, mx, my);
            ASSERT_FLOAT_EQ(mx, expected_x) << "world " << x << ", " << y;
            ASSERT_FLOAT_EQ(my, expected_y) << "world " << x << ", " << y;

            map.fromWorldIntoMap(world_point, map_point);
            ASSERT_FLOAT_EQ(map_point.x(), expected_x) << "world " << x << ", " << y;
            ASSERT_FLOAT_EQ(map_point.y(), expected_y) << "world " << x << ", " << y;
        }
    }
}

// ============================================================================
// Map Creation Tests
// ============================================================================

TEST_F(GridMapTest, CreateMapDrawsTheBorder)
{
    const size_t last = SIZE - 1;

    for (size_t i = 0; i < SIZE; i++)
    {
        ASSERT_EQ(map(i, 0), Map::FIELD::OCCUPIED) << "x " << i;
        ASSERT_EQ(map(i, last), Map::FIELD::OCCUPIED) << "x " << i;
        ASSERT_EQ(map(0, i), Map::FIELD::OCCUPIED) << "y " << i;
        ASSERT_EQ(map(last, i), Map::FIELD::OCCUPIED) << "y " << i;
    }
}

TEST_F(GridMapTest, CreateMapDrawsTheCross)
{
    Eigen::Vector2i begin;
    Eigen::Vector2i end;

    // First diagonal, rising in the world: map y decreases
    map.fromWorldIntoMap(-3.0f, -3.0f, begin);
    map.fromWorldIntoMap(-2.0f, -2.0f, end);
    for (Eigen::Vector2i cell = begin; cell.x() <= end.x() && cell.y() >= end.y(); cell += Eigen::Vector2i(1, -1))
    {
        ASSERT_EQ(map(cell), Map::FIELD::OCCUPIED) << "cell " << cell.transpose();
    }

    // Second diagonal, falling in the world: map y increases
    map.fromWorldIntoMap(-3.0f, -2.0f, begin);
    map.fromWorldIntoMap(-2.0f, -3.0f, end);
    for (Eigen::Vector2i cell = begin; cell.x() <= end.x() && cell.y() <= end.y(); cell += Eigen::Vector2i(1, 1))
    {
        ASSERT_EQ(map(cell), Map::FIELD::OCCUPIED) << "cell " << cell.transpose();
    }
}

TEST_F(GridMapTest, CreateMapDrawsTheCircleWithBuffer)
{
    const float radius = 0.5f;
    const float radius_occupied = radius + GRID_SCALE / 2.0f;
    const float radius_buffer = radius + BUFFER_SCALE;

    Eigen::Vector2f center;
    Eigen::Vector2i begin;
    Eigen::Vector2i end;
    map.fromWorldIntoMap(-2.5f, 2.5f, center);
    map.fromWorldIntoMap(-2.5f - radius_buffer, 2.5f + radius_buffer, begin);
    map.fromWorldIntoMap(-2.5f + radius_buffer, 2.5f - radius_buffer, end);

    for (int y = begin.y(); y <= end.y(); y++)
    {
        for (int x = begin.x(); x <= end.x(); x++)
        {
            const float distance = (Eigen::Vector2f(static_cast<float>(x), static_cast<float>(y)) - center).norm()
                                   * GRID_SCALE;

            if (distance <= radius_occupied)
            {
                ASSERT_EQ(map(Eigen::Vector2i(x, y)), Map::FIELD::OCCUPIED) << "cell " << x << ", " << y;
            }
            else if (distance <= radius_buffer)
            {
                ASSERT_EQ(map(Eigen::Vector2i(x, y)), Map::FIELD::BUFFER) << "cell " << x << ", " << y;
            }
        }
    }
}
