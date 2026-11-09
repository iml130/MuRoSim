// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/util/spatial_grid.hpp"
#include "murosim/geometry/circle.hpp"
#include <stdexcept>

namespace
{
using murosim::geometry::Circle;
using murosimproc::util::SpatialGrid;

constexpr float CELL_SIZE = 2.0f;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(SpatialGridConstructor, NonPositiveCellSizeThrows)
{
    EXPECT_THROW(SpatialGrid(0.0f), std::invalid_argument);
    EXPECT_THROW(SpatialGrid(-1.0f), std::invalid_argument);
}

TEST(SpatialGridConstructor, StartsEmpty)
{
    const SpatialGrid grid(CELL_SIZE);

    EXPECT_EQ(grid.size(), static_cast<size_t>(0));
    EXPECT_FALSE(grid.hasCollision(Circle(0.0f, 0.0f, 1.0f)));
}

// ============================================================================
// Test Fixture
// ============================================================================

class SpatialGridTest : public ::testing::Test
{
protected:
    SpatialGrid grid{CELL_SIZE};
};

// ============================================================================
// Insert Tests
// ============================================================================

TEST_F(SpatialGridTest, CircleInsideOneCellCountsOnce)
{
    grid.insert(Circle(1.0f, 1.0f, 0.5f)); // well inside cell (0, 0)

    EXPECT_EQ(grid.size(), static_cast<size_t>(1));
}

// size() counts one entry per cell a circle overlaps, not one per circle.
TEST_F(SpatialGridTest, CircleAcrossCellsCountsPerCell)
{
    grid.insert(Circle(2.0f, 2.0f, 0.5f)); // on the corner of four cells

    EXPECT_EQ(grid.size(), static_cast<size_t>(4));
}

TEST_F(SpatialGridTest, ClearRemovesEverything)
{
    grid.insert(Circle(1.0f, 1.0f, 0.5f));
    grid.insert(Circle(5.0f, 5.0f, 0.5f));

    grid.clear();

    EXPECT_EQ(grid.size(), static_cast<size_t>(0));
    EXPECT_FALSE(grid.hasCollision(Circle(1.0f, 1.0f, 0.5f)));
}

// ============================================================================
// Collision Tests
// ============================================================================

TEST_F(SpatialGridTest, OverlappingCircleCollides)
{
    grid.insert(Circle(1.0f, 1.0f, 0.5f));

    EXPECT_TRUE(grid.hasCollision(Circle(1.6f, 1.0f, 0.5f))); // centres 0.6 apart, radii sum to 1
}

TEST_F(SpatialGridTest, SeparatedCircleDoesNotCollide)
{
    grid.insert(Circle(1.0f, 1.0f, 0.5f));

    EXPECT_FALSE(grid.hasCollision(Circle(1.0f, 2.5f, 0.5f))); // centres 1.5 apart
}

TEST_F(SpatialGridTest, CollisionIsFoundAcrossCellBoundaries)
{
    grid.insert(Circle(1.9f, 1.0f, 0.5f)); // cell (0, 0), reaching into cell (1, 0)

    EXPECT_TRUE(grid.hasCollision(Circle(2.4f, 1.0f, 0.5f))); // cell (1, 0)
}

TEST_F(SpatialGridTest, NegativeCoordinatesWork)
{
    grid.insert(Circle(-3.0f, -3.0f, 0.5f));

    EXPECT_TRUE(grid.hasCollision(Circle(-3.5f, -3.0f, 0.5f)));
    EXPECT_FALSE(grid.hasCollision(Circle(3.0f, 3.0f, 0.5f)));
}

TEST_F(SpatialGridTest, LargeCircleCollidesWithDistantSmallOne)
{
    grid.insert(Circle(0.0f, 0.0f, 0.2f));

    EXPECT_TRUE(grid.hasCollision(Circle(6.0f, 0.0f, 6.0f))); // spans many cells and reaches the origin
}
