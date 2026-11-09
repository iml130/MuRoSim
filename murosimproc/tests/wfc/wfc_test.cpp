// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/wfc/wfc.hpp"
#include "murosimproc/wfc/preset.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace
{
using murosimproc::util::Array2d;
using murosimproc::wfc::Tiles;
using murosimproc::wfc::WaveFunctionCollapse;

constexpr size_t PATTERN_SIZE = 2;
constexpr size_t CELLS_X = 5;
constexpr size_t CELLS_Y = 4;
constexpr size_t ATTEMPTS = 10;
constexpr uint32_t SEEDS = 10;

/// The four orientations of a half wall (see propagator_test.cpp).
Tiles halfWallTiles()
{
    return Tiles(fixtures::makeImage({"##", "  "}), PATTERN_SIZE);
}

bool sameImage(const Array2d<uint8_t> &a, const Array2d<uint8_t> &b)
{
    return a == b;
}

/// Every 2 x 2 block at a tile position is one of the four half walls: two
/// wall cells in one row or one column, the other two free.
void assertBlocksAreHalfWalls(const Array2d<uint8_t> &image)
{
    for (size_t by = 0; by < image.sizeY(); by += PATTERN_SIZE)
    {
        for (size_t bx = 0; bx < image.sizeX(); bx += PATTERN_SIZE)
        {
            const bool top_row = image(bx, by) == 1 && image(bx + 1, by) == 1;
            const bool bottom_row = image(bx, by + 1) == 1 && image(bx + 1, by + 1) == 1;
            const bool left_column = image(bx, by) == 1 && image(bx, by + 1) == 1;
            const bool right_column = image(bx + 1, by) == 1 && image(bx + 1, by + 1) == 1;
            size_t walls = 0;
            for (size_t y = by; y < by + PATTERN_SIZE; y++)
            {
                for (size_t x = bx; x < bx + PATTERN_SIZE; x++)
                {
                    if (image(x, y) == 1)
                    {
                        walls++;
                    }
                }
            }
            ASSERT_EQ(walls, static_cast<size_t>(2)) << "block " << bx << ", " << by;
            ASSERT_TRUE(top_row || bottom_row || left_column || right_column) << "block " << bx << ", " << by;
        }
    }
}

/// Adjacent tiles share their facing edge: the last column of one tile
/// equals the first column of the next, and the same for rows.
void assertTileEdgesMatch(const Array2d<uint8_t> &image, const size_t pattern_size)
{
    for (size_t x = pattern_size; x < image.sizeX(); x += pattern_size)
    {
        for (size_t y = 0; y < image.sizeY(); y++)
        {
            ASSERT_EQ(image(x - 1, y), image(x, y)) << "column boundary " << x << " row " << y;
        }
    }
    for (size_t y = pattern_size; y < image.sizeY(); y += pattern_size)
    {
        for (size_t x = 0; x < image.sizeX(); x++)
        {
            ASSERT_EQ(image(x, y - 1), image(x, y)) << "row boundary " << y << " column " << x;
        }
    }
}
} // namespace

// ============================================================================
// Run Tests
// ============================================================================

TEST(WaveFunctionCollapseRun, ImageSizeIsCellsTimesPatternSize)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    const Array2d<uint8_t> image = wfc.run(CELLS_X, CELLS_Y, fixtures::SEED, ATTEMPTS);

    ASSERT_FALSE(image.empty());
    EXPECT_EQ(image.sizeX(), CELLS_X * PATTERN_SIZE);
    EXPECT_EQ(image.sizeY(), CELLS_Y * PATTERN_SIZE);
}

TEST(WaveFunctionCollapseRun, SinglePatternFillsTheImage)
{
    WaveFunctionCollapse wfc(Tiles(fixtures::makeImage({"...", "...", "..."}), PATTERN_SIZE));

    const Array2d<uint8_t> image = wfc.run(CELLS_X, CELLS_Y, fixtures::SEED, ATTEMPTS);

    ASSERT_EQ(image.size(), CELLS_X * CELLS_Y * PATTERN_SIZE * PATTERN_SIZE);
    for (size_t i = 0; i < image.size(); i++)
    {
        ASSERT_EQ(image[i], static_cast<uint8_t>(2)) << "pixel " << i;
    }
}

TEST(WaveFunctionCollapseRun, EveryCellIsOneOfThePatterns)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    const Array2d<uint8_t> image = wfc.run(CELLS_X, CELLS_Y, fixtures::SEED, ATTEMPTS);

    ASSERT_FALSE(image.empty());
    assertBlocksAreHalfWalls(image);
}

TEST(WaveFunctionCollapseRun, AdjacentTilesShareTheirEdges)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    const Array2d<uint8_t> image = wfc.run(CELLS_X, CELLS_Y, fixtures::SEED, ATTEMPTS);

    ASSERT_FALSE(image.empty());
    assertTileEdgesMatch(image, PATTERN_SIZE);
}

TEST(WaveFunctionCollapseRun, PresetTilesGiveAValidImage)
{
    const Tiles tiles = murosimproc::wfc::preset::getPreset(murosimproc::wfc::preset::PresetType::LABYRINTH);
    WaveFunctionCollapse wfc(tiles);

    const Array2d<uint8_t> image = wfc.run(CELLS_X, CELLS_Y, fixtures::SEED, ATTEMPTS);

    ASSERT_FALSE(image.empty());
    EXPECT_EQ(image.sizeX(), CELLS_X * tiles.getPatternSize());
    EXPECT_EQ(image.sizeY(), CELLS_Y * tiles.getPatternSize());
    for (size_t i = 0; i < image.size(); i++)
    {
        ASSERT_LE(image[i], static_cast<uint8_t>(2)) << "pixel " << i;
    }
    assertTileEdgesMatch(image, tiles.getPatternSize());
}

TEST(WaveFunctionCollapseRun, OneCellImageIsASinglePattern)
{
    const Tiles tiles = halfWallTiles();
    WaveFunctionCollapse wfc(tiles);

    const Array2d<uint8_t> image = wfc.run(1, 1, fixtures::SEED, ATTEMPTS);

    ASSERT_EQ(image.sizeX(), PATTERN_SIZE);
    ASSERT_EQ(image.sizeY(), PATTERN_SIZE);
    bool is_pattern = false;
    for (const Array2d<uint8_t> &pattern : tiles.getPatterns())
    {
        is_pattern = is_pattern || pattern == image;
    }
    EXPECT_TRUE(is_pattern);
}

// ============================================================================
// Attempt Tests
// ============================================================================

TEST(WaveFunctionCollapseAttempts, ZeroAttemptsGiveAnEmptyImage)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    EXPECT_TRUE(wfc.run(CELLS_X, CELLS_Y, fixtures::SEED, 0).empty());
}

// Attempt i runs with seed + i, so a run that succeeds on its first attempt
// equals the same seed with more attempts allowed.
TEST(WaveFunctionCollapseAttempts, FirstSuccessfulAttemptIsTheResult)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    size_t first_try_successes = 0;
    for (uint32_t seed = 0; seed < SEEDS; seed++)
    {
        const Array2d<uint8_t> single = wfc.run(CELLS_X, CELLS_Y, seed, 1);
        if (!single.empty())
        {
            first_try_successes++;
            ASSERT_TRUE(sameImage(single, wfc.run(CELLS_X, CELLS_Y, seed, ATTEMPTS))) << "seed " << seed;
        }
    }
    EXPECT_GT(first_try_successes, static_cast<size_t>(0));
}

TEST(WaveFunctionCollapseAttempts, RetriesRecoverFromContradictions)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    for (uint32_t seed = 0; seed < SEEDS; seed++)
    {
        ASSERT_FALSE(wfc.run(CELLS_X, CELLS_Y, seed, ATTEMPTS).empty()) << "seed " << seed;
    }
}

// ============================================================================
// Determinism Tests
// ============================================================================

TEST(WaveFunctionCollapseDeterminism, SameSeedGivesTheSameImage)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    const Array2d<uint8_t> first = wfc.run(CELLS_X, CELLS_Y, 7, ATTEMPTS);
    const Array2d<uint8_t> second = wfc.run(CELLS_X, CELLS_Y, 7, ATTEMPTS);

    ASSERT_FALSE(first.empty());
    EXPECT_TRUE(sameImage(first, second));
}

TEST(WaveFunctionCollapseDeterminism, DifferentSeedsGiveDifferentImages)
{
    WaveFunctionCollapse wfc(halfWallTiles());

    bool differs = false;
    const Array2d<uint8_t> reference = wfc.run(CELLS_X, CELLS_Y, 0, ATTEMPTS);
    ASSERT_FALSE(reference.empty());
    for (uint32_t seed = 1; seed < SEEDS; seed++)
    {
        const Array2d<uint8_t> image = wfc.run(CELLS_X, CELLS_Y, seed, ATTEMPTS);
        ASSERT_FALSE(image.empty()) << "seed " << seed;
        differs = differs || !sameImage(reference, image);
    }
    EXPECT_TRUE(differs);
}
