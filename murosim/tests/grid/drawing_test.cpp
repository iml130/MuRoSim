// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/grid/drawing.hpp"
#include "murosim/grid/map.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
using murosim::grid::Map;

// drawing::circle fills the disk. The int overloads take cells whose centre
// is within the radius, the world-coordinate overloads every cell the circle
// physically overlaps (radius + half a cell), so the edge of the drawn disk
// lies within half a cell of the radius.
constexpr float RASTER_TOLERANCE = 0.5f;

/// A square world of 20 m around the origin; with GRID_SCALE it becomes a
/// 201 x 201 map whose cell (100, 100) is the world origin.
constexpr float WORLD_HALF_SIZE = 10.0f;
constexpr float GRID_SCALE = 0.1f;

murosim::geometry::World squareWorld()
{
    murosim::geometry::Shape shape;
    shape.addLine(-WORLD_HALF_SIZE, -WORLD_HALF_SIZE, WORLD_HALF_SIZE, -WORLD_HALF_SIZE);
    shape.addLine(WORLD_HALF_SIZE, -WORLD_HALF_SIZE, WORLD_HALF_SIZE, WORLD_HALF_SIZE);
    shape.addLine(WORLD_HALF_SIZE, WORLD_HALF_SIZE, -WORLD_HALF_SIZE, WORLD_HALF_SIZE);
    shape.addLine(-WORLD_HALF_SIZE, WORLD_HALF_SIZE, -WORLD_HALF_SIZE, -WORLD_HALF_SIZE);
    return murosim::geometry::World(shape);
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class DrawingTest : public ::testing::Test
{
protected:
    Map map{squareWorld(), {{0.0f, 0.0f}}, GRID_SCALE, GRID_SCALE};

    // Every test starts from an all-FREE map so only the drawn cells differ
    void SetUp() override
    {
        fillMap(Map::FIELD::FREE);
    }

    void fillMap(const uint8_t symbol)
    {
        for (size_t i = 0; i < map.size(); i++)
        {
            map[i] = symbol;
        }
    }

    size_t countCells(const uint8_t symbol) const
    {
        size_t count = 0;
        for (size_t i = 0; i < map.size(); i++)
        {
            if (map[i] == symbol)
            {
                count++;
            }
        }
        return count;
    }

    /// Draws the OCCUPIED outline of the square [low, high] x [low, high].
    void drawSquareOutline(const size_t low, const size_t high)
    {
        for (size_t i = low; i <= high; i++)
        {
            map(i, low) = Map::FIELD::OCCUPIED;
            map(i, high) = Map::FIELD::OCCUPIED;
            map(low, i) = Map::FIELD::OCCUPIED;
            map(high, i) = Map::FIELD::OCCUPIED;
        }
    }

    /// Asserts that every cell between the two end points of an axis-aligned
    /// or diagonal segment is OCCUPIED and that nothing else is.
    void assertSegmentDrawn(const Eigen::Vector2i &p1, const Eigen::Vector2i &p2)
    {
        const int steps = std::max(std::abs(p2.x() - p1.x()), std::abs(p2.y() - p1.y()));
        const int dx = (p2.x() > p1.x()) - (p2.x() < p1.x());
        const int dy = (p2.y() > p1.y()) - (p2.y() < p1.y());

        for (int i = 0; i <= steps; i++)
        {
            const Eigen::Vector2i cell(p1.x() + i * dx, p1.y() + i * dy);
            ASSERT_EQ(map(cell), Map::FIELD::OCCUPIED) << "cell " << cell.transpose();
        }
        EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), static_cast<size_t>(steps + 1));
    }

    /// Asserts that the OCCUPIED cells form the disk of the given radius
    /// around the centre: everything clearly inside is occupied, everything
    /// clearly outside is free, with half a cell of raster slack in between.
    void assertDiskDrawn(const Eigen::Vector2i &center, const float radius)
    {
        ASSERT_EQ(map(center), Map::FIELD::OCCUPIED);

        for (size_t i = 0; i < map.size(); i++)
        {
            const Eigen::Vector2i cell = map.indexToPoint(i);
            const float distance = (cell - center).cast<float>().norm();
            const bool occupied = map[i] == Map::FIELD::OCCUPIED;

            if (distance <= radius - RASTER_TOLERANCE)
            {
                ASSERT_TRUE(occupied) << "cell " << cell.transpose() << " inside the disk";
            }
            else if (distance > radius + RASTER_TOLERANCE)
            {
                ASSERT_FALSE(occupied) << "cell " << cell.transpose() << " outside the disk";
            }
        }
    }
};

// ============================================================================
// Line Drawing Tests
// ============================================================================

TEST_F(DrawingTest, LineInts)
{
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, 50, 100, 150, 100);

    assertSegmentDrawn({50, 100}, {150, 100});
}

TEST_F(DrawingTest, LineVector2i)
{
    const Eigen::Vector2i p1(50, 50);
    const Eigen::Vector2i p2(100, 100);
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, p1, p2);

    assertSegmentDrawn(p1, p2);
}

TEST_F(DrawingTest, LineFloats)
{
    // World coordinates are converted to cells first
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, -5.0f, 0.0f, 5.0f, 0.0f);

    Eigen::Vector2i p1;
    Eigen::Vector2i p2;
    map.fromWorldIntoMap(-5.0f, 0.0f, p1);
    map.fromWorldIntoMap(5.0f, 0.0f, p2);
    assertSegmentDrawn(p1, p2);
}

TEST_F(DrawingTest, LineVector2f)
{
    const Eigen::Vector2f w1(-5.0f, -5.0f);
    const Eigen::Vector2f w2(5.0f, 5.0f);
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, w1, w2);

    Eigen::Vector2i p1;
    Eigen::Vector2i p2;
    map.fromWorldIntoMap(w1, p1);
    map.fromWorldIntoMap(w2, p2);
    assertSegmentDrawn(p1, p2);
}

TEST_F(DrawingTest, LineShapeLine)
{
    const murosim::geometry::Line shape_line(-5.0f, 0.0f, 5.0f, 0.0f);
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, shape_line);

    Eigen::Vector2i p1;
    Eigen::Vector2i p2;
    map.fromWorldIntoMap(shape_line.getPoint1(), p1);
    map.fromWorldIntoMap(shape_line.getPoint2(), p2);
    assertSegmentDrawn(p1, p2);
}

// ============================================================================
// Circle Drawing Tests
// ============================================================================

TEST_F(DrawingTest, CircleInts)
{
    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, 100, 100, 10);

    assertDiskDrawn({100, 100}, 10.0f);
}

TEST_F(DrawingTest, CircleVector2i)
{
    const Eigen::Vector2i center(100, 100);
    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, center, 10);

    assertDiskDrawn(center, 10.0f);
}

TEST_F(DrawingTest, CircleFloats)
{
    // World coordinates: the origin is cell (100, 100), 1 m is 10 cells
    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, 0.0f, 0.0f, 1.0f);

    Eigen::Vector2i center;
    map.fromWorldIntoMap(0.0f, 0.0f, center);
    assertDiskDrawn(center, 1.0f * map.getGridScaleInv());
}

TEST_F(DrawingTest, CircleVector2f)
{
    const Eigen::Vector2f world_center(0.0f, 0.0f);
    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, world_center, 1.0f);

    Eigen::Vector2i center;
    map.fromWorldIntoMap(world_center, center);
    assertDiskDrawn(center, 1.0f * map.getGridScaleInv());
}

TEST_F(DrawingTest, CircleShapeCircle)
{
    const murosim::geometry::Circle shape_circle(0.0f, 0.0f, 1.0f);
    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, shape_circle);

    Eigen::Vector2i center;
    map.fromWorldIntoMap(shape_circle.getPoint(), center);
    assertDiskDrawn(center, shape_circle.getRadius() * map.getGridScaleInv());
}

// ============================================================================
// Fill Tests
// ============================================================================

// The fill tests draw the outline of the square [50, 60] x [50, 60] and flood
// its interior [51, 59] x [51, 59]: 81 cells

TEST_F(DrawingTest, FillInts)
{
    drawSquareOutline(50, 60);

    murosim::grid::drawing::fill(map, Map::FIELD::FREE, Map::FIELD::BUFFER, 55, 55);

    EXPECT_EQ(countCells(Map::FIELD::BUFFER), static_cast<size_t>(81));
    EXPECT_EQ(map(55, 55), Map::FIELD::BUFFER);
    EXPECT_EQ(map(51, 51), Map::FIELD::BUFFER);
    EXPECT_EQ(map(50, 50), Map::FIELD::OCCUPIED); // outline untouched
    EXPECT_EQ(map(49, 49), Map::FIELD::FREE);     // outside untouched
}

TEST_F(DrawingTest, FillVector2i)
{
    drawSquareOutline(50, 60);

    const Eigen::Vector2i start(55, 55);
    murosim::grid::drawing::fill(map, Map::FIELD::FREE, Map::FIELD::BUFFER, start);

    EXPECT_EQ(countCells(Map::FIELD::BUFFER), static_cast<size_t>(81));
    EXPECT_EQ(map(start), Map::FIELD::BUFFER);
    EXPECT_EQ(map(50, 50), Map::FIELD::OCCUPIED); // outline untouched
    EXPECT_EQ(map(49, 49), Map::FIELD::FREE);     // outside untouched
}

TEST_F(DrawingTest, FillWithIndicesInts)
{
    drawSquareOutline(50, 60);

    std::vector<size_t> filled_indices;
    murosim::grid::drawing::fill(map, Map::FIELD::FREE, Map::FIELD::BUFFER, 55, 55, filled_indices);

    // The reported indices are exactly the filled cells
    ASSERT_EQ(filled_indices.size(), static_cast<size_t>(81));
    EXPECT_EQ(countCells(Map::FIELD::BUFFER), static_cast<size_t>(81));
    for (const size_t index : filled_indices)
    {
        EXPECT_EQ(map[index], Map::FIELD::BUFFER) << "index " << index;
    }
}

TEST_F(DrawingTest, FillWithIndicesVector2i)
{
    drawSquareOutline(50, 60);

    std::vector<size_t> filled_indices;
    const Eigen::Vector2i start(55, 55);
    murosim::grid::drawing::fill(map, Map::FIELD::FREE, Map::FIELD::BUFFER, start, filled_indices);

    ASSERT_EQ(filled_indices.size(), static_cast<size_t>(81));
    EXPECT_EQ(countCells(Map::FIELD::BUFFER), static_cast<size_t>(81));
    for (const size_t index : filled_indices)
    {
        EXPECT_EQ(map[index], Map::FIELD::BUFFER) << "index " << index;
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(DrawingTest, LineSinglePoint)
{
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, 100, 100, 100, 100);

    EXPECT_EQ(map(100, 100), Map::FIELD::OCCUPIED);
    EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), static_cast<size_t>(1));
}

TEST_F(DrawingTest, CircleZeroRadiusIsTheCenter)
{
    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, 100, 100, 0);

    EXPECT_EQ(map(100, 100), Map::FIELD::OCCUPIED);
    EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), static_cast<size_t>(1));
}

TEST_F(DrawingTest, FillWithoutMatchingCellsChangesNothing)
{
    fillMap(Map::FIELD::OCCUPIED);

    // Nothing is FREE, so there is nothing to replace
    murosim::grid::drawing::fill(map, Map::FIELD::FREE, Map::FIELD::BUFFER, 55, 55);

    EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), map.size());
}

// ============================================================================
// Map Edge Tests
// ============================================================================

// Cells outside the map are skipped; nothing throws and nothing is written
// out of bounds.

TEST_F(DrawingTest, LineBeyondMapEdgeIsClipped)
{
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, 100, 100, 250, 100); // ends 50 cells past the edge

    for (size_t x = 100; x < map.sizeX(); x++)
    {
        ASSERT_EQ(map(x, 100), Map::FIELD::OCCUPIED) << "cell " << x;
    }
    EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), map.sizeX() - 100);
}

TEST_F(DrawingTest, LineStartingOutsideMapDrawsOnlyTheInsidePart)
{
    murosim::grid::drawing::line(map, Map::FIELD::OCCUPIED, -20, 50, 20, 50);

    EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), static_cast<size_t>(21)); // x = 0 .. 20
    EXPECT_EQ(map(0, 50), Map::FIELD::OCCUPIED);
    EXPECT_EQ(map(20, 50), Map::FIELD::OCCUPIED);
}

TEST_F(DrawingTest, CircleOnMapEdgeIsClipped)
{
    const Eigen::Vector2i center(static_cast<int>(map.sizeX()) - 1, 100); // right edge

    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, center, 5);

    assertDiskDrawn(center, 5.0f); // the in-map half of the disk
}

TEST_F(DrawingTest, WorldCircleOutsideMapDrawsNothing)
{
    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, 2.0f * WORLD_HALF_SIZE, 0.0f, 0.5f);

    EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), static_cast<size_t>(0));
}

// ============================================================================
// Circle Cells Tests
// ============================================================================

TEST_F(DrawingTest, CircleCellsVisitsEveryCellOnce)
{
    std::vector<Eigen::Vector2i> visited;

    murosim::grid::drawing::circleCells(map, Eigen::Vector2f(0.0f, 0.0f), 0.5f,
                                        [&visited](const Eigen::Vector2i cell) { visited.push_back(cell); });

    std::vector<Eigen::Vector2i> unique = visited;
    std::sort(unique.begin(), unique.end(), [](const Eigen::Vector2i &a, const Eigen::Vector2i &b)
    {
        return a.x() != b.x() ? a.x() < b.x() : a.y() < b.y();
    });
    unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
    EXPECT_EQ(unique.size(), visited.size());
    EXPECT_EQ(visited.front(), Eigen::Vector2i(100, 100)); // the centre cell comes first
}

TEST_F(DrawingTest, CircleCellsMatchesDrawnCircle)
{
    std::vector<Eigen::Vector2i> visited;
    murosim::grid::drawing::circleCells(map, Eigen::Vector2f(1.0f, -2.0f), 0.7f,
                                        [&visited](const Eigen::Vector2i cell) { visited.push_back(cell); });

    murosim::grid::drawing::circle(map, Map::FIELD::OCCUPIED, 1.0f, -2.0f, 0.7f);

    EXPECT_EQ(countCells(Map::FIELD::OCCUPIED), visited.size());
    for (size_t i = 0; i < visited.size(); i++)
    {
        ASSERT_EQ(map(visited[i]), Map::FIELD::OCCUPIED) << "cell " << visited[i].transpose();
    }
}

TEST_F(DrawingTest, CircleCellsOfTinyCircleIsTheCentreCell)
{
    std::vector<Eigen::Vector2i> visited;

    murosim::grid::drawing::circleCells(map, Eigen::Vector2f(0.0f, 0.0f), 0.001f,
                                        [&visited](const Eigen::Vector2i cell) { visited.push_back(cell); });

    ASSERT_EQ(visited.size(), static_cast<size_t>(1));
    EXPECT_EQ(visited.front(), Eigen::Vector2i(100, 100));
}

TEST_F(DrawingTest, CircleCellsOutsideMapVisitsNothing)
{
    size_t visits = 0;

    murosim::grid::drawing::circleCells(map, Eigen::Vector2f(3.0f * WORLD_HALF_SIZE, 0.0f), 0.5f,
                                        [&visits](const Eigen::Vector2i) { visits++; });

    EXPECT_EQ(visits, static_cast<size_t>(0));
}
