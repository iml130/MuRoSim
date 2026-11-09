// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/util/grid.hpp"
#include "murosim/grid/map.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <vector>

namespace
{
using murosim::grid::Map;
using murosimrl::util::grid::embedMapCentered;
using murosimrl::util::grid::extractMapPatch;
using murosimrl::util::grid::gridValueLut;

constexpr size_t MAP_SIZE = 128; // larger than the 101 x 101 fixture map
constexpr size_t PATCH_DIM = 5;
constexpr float FREE = 0.0f;
constexpr float OCCUPIED = 1.0f;
constexpr float BUFFER = 0.5f;
} // namespace

// ============================================================================
// Lookup Table Tests
// ============================================================================

TEST(UtilGridLut, WithBufferMapsBufferToHalf)
{
    const float *lut = gridValueLut(true);

    EXPECT_FLOAT_EQ(lut[Map::FIELD::FREE], FREE);
    EXPECT_FLOAT_EQ(lut[Map::FIELD::OCCUPIED], OCCUPIED);
    EXPECT_FLOAT_EQ(lut[Map::FIELD::BUFFER], BUFFER);
    EXPECT_FLOAT_EQ(lut[Map::FIELD::UNREACHABLE], OCCUPIED);
}

TEST(UtilGridLut, WithoutBufferMapsBufferToFree)
{
    const float *lut = gridValueLut(false);

    EXPECT_FLOAT_EQ(lut[Map::FIELD::FREE], FREE);
    EXPECT_FLOAT_EQ(lut[Map::FIELD::OCCUPIED], OCCUPIED);
    EXPECT_FLOAT_EQ(lut[Map::FIELD::BUFFER], FREE);
    EXPECT_FLOAT_EQ(lut[Map::FIELD::UNREACHABLE], OCCUPIED);
}

// ============================================================================
// Test Fixture
// ============================================================================

class UtilGridTest : public ::testing::Test
{
protected:
    /// Index of the cell (x, y) in a patch of PATCH_DIM columns.
    static size_t patchIndex(const size_t x, const size_t y)
    {
        return y * PATCH_DIM + x;
    }

    Map map{fixtures::squareWorld(), {}, fixtures::GRID_SCALE, fixtures::BUFFER_SCALE};
    const float *lut = gridValueLut(true);
};

// ============================================================================
// Extract Map Patch Tests
// ============================================================================

TEST_F(UtilGridTest, PatchInsideMapCopiesCellsThroughLut)
{
    const Eigen::Vector2i center(50, 50); // the world origin
    std::vector<float> patch(PATCH_DIM * PATCH_DIM, -1.0f);

    extractMapPatch(map, center, PATCH_DIM, lut, patch.data());

    for (size_t y = 0; y < PATCH_DIM; y++)
    {
        for (size_t x = 0; x < PATCH_DIM; x++)
        {
            const size_t map_x = static_cast<size_t>(center.x()) - PATCH_DIM / 2 + x;
            const size_t map_y = static_cast<size_t>(center.y()) - PATCH_DIM / 2 + y;
            ASSERT_FLOAT_EQ(patch[patchIndex(x, y)], lut[map(map_x, map_y)]) << "cell " << x << ", " << y;
        }
    }
}

TEST_F(UtilGridTest, PatchAtTheWallSeesOccupiedAndBuffer)
{
    const Eigen::Vector2i center(98, 50); // two cells left of the right wall
    std::vector<float> patch(PATCH_DIM * PATCH_DIM, -1.0f);

    extractMapPatch(map, center, PATCH_DIM, lut, patch.data());

    // The last column is the wall, the columns before it the buffer band
    EXPECT_FLOAT_EQ(patch[patchIndex(4, 2)], OCCUPIED);
    EXPECT_FLOAT_EQ(patch[patchIndex(3, 2)], lut[map(99, 50)]);
    EXPECT_GT(std::count(patch.begin(), patch.end(), BUFFER), 0);
    EXPECT_GT(std::count(patch.begin(), patch.end(), FREE), 0);
}

TEST_F(UtilGridTest, PatchBeyondTheMapReadsOccupied)
{
    const Eigen::Vector2i center(0, 0); // the map corner: three quarters of the patch lie outside
    std::vector<float> patch(PATCH_DIM * PATCH_DIM, -1.0f);

    extractMapPatch(map, center, PATCH_DIM, lut, patch.data());

    for (size_t y = 0; y < PATCH_DIM; y++)
    {
        for (size_t x = 0; x < PATCH_DIM; x++)
        {
            const bool outside = x < PATCH_DIM / 2 || y < PATCH_DIM / 2;
            const float expected = outside ? OCCUPIED : lut[map(x - PATCH_DIM / 2, y - PATCH_DIM / 2)];
            ASSERT_FLOAT_EQ(patch[patchIndex(x, y)], expected) << "cell " << x << ", " << y;
        }
    }
}

TEST_F(UtilGridTest, PatchEntirelyOutsideIsAllOccupied)
{
    const Eigen::Vector2i center(500, 500);
    std::vector<float> patch(PATCH_DIM * PATCH_DIM, -1.0f);

    extractMapPatch(map, center, PATCH_DIM, lut, patch.data());

    for (size_t i = 0; i < patch.size(); i++)
    {
        ASSERT_FLOAT_EQ(patch[i], OCCUPIED) << "index " << i;
    }
}

// ============================================================================
// Embed Map Centered Tests
// ============================================================================

TEST_F(UtilGridTest, EmbedReturnsTheCenteringOffset)
{
    std::vector<float> output(MAP_SIZE * MAP_SIZE, -1.0f);

    const Eigen::Vector2i offset = embedMapCentered(map, MAP_SIZE, lut, output.data());

    EXPECT_EQ(offset.x(), static_cast<int>((MAP_SIZE - map.sizeX()) / 2));
    EXPECT_EQ(offset.y(), static_cast<int>((MAP_SIZE - map.sizeY()) / 2));
}

TEST_F(UtilGridTest, EmbedCopiesTheMapAndPadsWithOccupied)
{
    std::vector<float> output(MAP_SIZE * MAP_SIZE, -1.0f);

    const Eigen::Vector2i offset = embedMapCentered(map, MAP_SIZE, lut, output.data());

    for (size_t y = 0; y < MAP_SIZE; y++)
    {
        for (size_t x = 0; x < MAP_SIZE; x++)
        {
            const int map_x = static_cast<int>(x) - offset.x();
            const int map_y = static_cast<int>(y) - offset.y();
            const float expected =
                map.isInMap(map_x, map_y) ? lut[map(static_cast<size_t>(map_x), static_cast<size_t>(map_y))] : OCCUPIED;
            ASSERT_FLOAT_EQ(output[y * MAP_SIZE + x], expected) << "cell " << x << ", " << y;
        }
    }
}

TEST_F(UtilGridTest, EmbedWithExactSizeHasNoPadding)
{
    const size_t size = map.sizeX();
    std::vector<float> output(size * size, -1.0f);

    const Eigen::Vector2i offset = embedMapCentered(map, size, lut, output.data());

    EXPECT_EQ(offset, Eigen::Vector2i(0, 0));
    for (size_t i = 0; i < output.size(); i++)
    {
        ASSERT_FLOAT_EQ(output[i], lut[map[i]]) << "index " << i;
    }
}
