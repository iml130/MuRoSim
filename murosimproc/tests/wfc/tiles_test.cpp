// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/wfc/tiles.hpp"
#include "murosimproc/wfc/preset.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace
{
using murosimproc::util::Array2d;
using murosimproc::wfc::DIRECTION;
using murosimproc::wfc::Tiles;

constexpr size_t PATTERN_SIZE = 2;
constexpr size_t SYMMETRIES = 8; // every window is added with its 7 mirror/rotation variants
constexpr double EPSILON = 1e-12; // weights are exact ratios of small integers

// A 2 x 2 window with one wall row: its 8 symmetries are the four
// orientations of that wall (top, bottom, left, right), two each.
const std::vector<std::string> HALF_WALL = {
    "##",
    "  ",
};

// The window at (0, 0) holds the only wall cell; the other three windows are empty.
const std::vector<std::string> ONE_CORNER = {
    "#  ",
    "   ",
    "   ",
};

/// Index of the pattern equal to `rows`, or the pattern count if there is none.
size_t findPattern(const Tiles &tiles, const std::vector<std::string> &rows)
{
    const Array2d<uint8_t> wanted = fixtures::makeImage(rows);
    for (size_t i = 0; i < tiles.getPatterns().size(); i++)
    {
        if (tiles.getPatterns()[i] == wanted)
        {
            return i;
        }
    }
    return tiles.getPatterns().size();
}

bool contains(const std::vector<uint16_t> &patterns, const size_t pattern)
{
    for (const uint16_t p : patterns)
    {
        if (p == pattern)
        {
            return true;
        }
    }
    return false;
}

double sum(const std::vector<double> &values)
{
    double total = 0.0;
    for (const double value : values)
    {
        total += value;
    }
    return total;
}
} // namespace

// ============================================================================
// Extraction Tests
// ============================================================================

TEST(TilesExtraction, UniformImageGivesOnePattern)
{
    const Tiles tiles(fixtures::makeImage({"   ", "   ", "   "}), PATTERN_SIZE);

    ASSERT_EQ(tiles.getPatterns().size(), static_cast<size_t>(1));
    EXPECT_EQ(tiles.getPatterns()[0], fixtures::makeImage({"  ", "  "}));
    ASSERT_EQ(tiles.getWeights().size(), static_cast<size_t>(1));
    EXPECT_NEAR(tiles.getWeights()[0], 1.0, EPSILON);
}

TEST(TilesExtraction, PatternsHaveThePatternSize)
{
    const Tiles tiles(fixtures::makeImage(ONE_CORNER), PATTERN_SIZE);

    ASSERT_FALSE(tiles.getPatterns().empty());
    for (size_t i = 0; i < tiles.getPatterns().size(); i++)
    {
        EXPECT_EQ(tiles.getPatterns()[i].sizeX(), PATTERN_SIZE) << "pattern " << i;
        EXPECT_EQ(tiles.getPatterns()[i].sizeY(), PATTERN_SIZE) << "pattern " << i;
    }
    EXPECT_EQ(tiles.getPatternSize(), PATTERN_SIZE);
    EXPECT_EQ(tiles.getStride(), static_cast<size_t>(1));
}

TEST(TilesExtraction, SymmetriesAreAddedAndDeduplicated)
{
    const Tiles tiles(fixtures::makeImage(HALF_WALL), PATTERN_SIZE);

    ASSERT_EQ(tiles.getPatterns().size(), static_cast<size_t>(4));
    EXPECT_LT(findPattern(tiles, {"##", "  "}), tiles.getPatterns().size());
    EXPECT_LT(findPattern(tiles, {"  ", "##"}), tiles.getPatterns().size());
    EXPECT_LT(findPattern(tiles, {"# ", "# "}), tiles.getPatterns().size());
    EXPECT_LT(findPattern(tiles, {" #", " #"}), tiles.getPatterns().size());
    EXPECT_FALSE(tiles.hasDuplicateTiles());
    EXPECT_TRUE(tiles.getDuplicateTiles().empty());
}

TEST(TilesExtraction, ObstacleCellsAreKeptApartFromWalls)
{
    const Tiles tiles(fixtures::makeImage({"..", "##"}), PATTERN_SIZE);

    ASSERT_EQ(tiles.getPatterns().size(), static_cast<size_t>(4));
    const size_t pattern = findPattern(tiles, {"..", "##"});
    ASSERT_LT(pattern, tiles.getPatterns().size());
    EXPECT_EQ(tiles.getPatterns()[pattern](0, 0), static_cast<uint8_t>(2));
    EXPECT_EQ(tiles.getPatterns()[pattern](0, 1), static_cast<uint8_t>(1));
}

// ============================================================================
// Weight Tests
// ============================================================================

TEST(TilesWeights, EqualOccurrencesGiveEqualWeights)
{
    const Tiles tiles(fixtures::makeImage(HALF_WALL), PATTERN_SIZE);

    ASSERT_EQ(tiles.getWeights().size(), static_cast<size_t>(4));
    for (size_t i = 0; i < tiles.getWeights().size(); i++)
    {
        EXPECT_NEAR(tiles.getWeights()[i], 0.25, EPSILON) << "pattern " << i;
    }
}

// 4 windows x 8 symmetries = 32 samples: 24 empty, 2 for each of the 4 corners
TEST(TilesWeights, WeightsCountOccurrencesIncludingSymmetries)
{
    const Tiles tiles(fixtures::makeImage(ONE_CORNER), PATTERN_SIZE);

    ASSERT_EQ(tiles.getPatterns().size(), static_cast<size_t>(5));
    const size_t empty = findPattern(tiles, {"  ", "  "});
    ASSERT_LT(empty, tiles.getPatterns().size());
    EXPECT_NEAR(tiles.getWeights()[empty], 24.0 / 32.0, EPSILON);
    const size_t corner = findPattern(tiles, {"# ", "  "});
    ASSERT_LT(corner, tiles.getPatterns().size());
    EXPECT_NEAR(tiles.getWeights()[corner], 2.0 / 32.0, EPSILON);
}

TEST(TilesWeights, WeightsSumToOne)
{
    const Tiles tiles = murosimproc::wfc::preset::getPreset(murosimproc::wfc::preset::PresetType::BLOCK);

    EXPECT_EQ(tiles.getWeights().size(), tiles.getPatterns().size());
    EXPECT_NEAR(sum(tiles.getWeights()), 1.0, 1e-9);
    for (size_t i = 0; i < tiles.getWeights().size(); i++)
    {
        EXPECT_GT(tiles.getWeights()[i], 0.0) << "pattern " << i;
    }
}

TEST(TilesWeights, StrideSkipsWindows)
{
    // Stride 2 on the 3 x 3 image leaves only the window at (0, 0)
    const Tiles tiles(fixtures::makeImage(ONE_CORNER), PATTERN_SIZE, 2);

    EXPECT_EQ(tiles.getStride(), static_cast<size_t>(2));
    ASSERT_EQ(tiles.getPatterns().size(), static_cast<size_t>(4));
    EXPECT_EQ(findPattern(tiles, {"  ", "  "}), tiles.getPatterns().size());
    for (size_t i = 0; i < tiles.getWeights().size(); i++)
    {
        EXPECT_NEAR(tiles.getWeights()[i], 2.0 / SYMMETRIES, EPSILON) << "pattern " << i;
    }
}

// ============================================================================
// Compatibility Tests
// ============================================================================

TEST(TilesCompatibility, UniformPatternIsCompatibleWithItselfEverywhere)
{
    const Tiles tiles(fixtures::makeImage({"   ", "   ", "   "}), PATTERN_SIZE);

    ASSERT_EQ(tiles.getCompatible().size(), static_cast<size_t>(1));
    for (size_t direction = 0; direction < DIRECTION::SIZE; direction++)
    {
        ASSERT_EQ(tiles.getCompatible()[0][direction].size(), static_cast<size_t>(1)) << "direction " << direction;
        EXPECT_EQ(tiles.getCompatible()[0][direction][0], static_cast<uint16_t>(0)) << "direction " << direction;
    }
}

// A neighbour in direction d must repeat the pattern's edge facing d on its
// opposite edge: the top wall continues sideways and meets the bottom wall
// above and below.
TEST(TilesCompatibility, NeighboursMustShareTheFacingEdge)
{
    const Tiles tiles(fixtures::makeImage(HALF_WALL), PATTERN_SIZE);
    const size_t top = findPattern(tiles, {"##", "  "});
    const size_t bottom = findPattern(tiles, {"  ", "##"});
    const size_t left = findPattern(tiles, {"# ", "# "});
    ASSERT_LT(top, tiles.getPatterns().size());
    ASSERT_LT(bottom, tiles.getPatterns().size());
    ASSERT_LT(left, tiles.getPatterns().size());

    const std::array<std::vector<uint16_t>, DIRECTION::SIZE> &compatible = tiles.getCompatible()[top];
    ASSERT_EQ(compatible[DIRECTION::NORTH].size(), static_cast<size_t>(1));
    EXPECT_EQ(compatible[DIRECTION::NORTH][0], static_cast<uint16_t>(bottom));
    ASSERT_EQ(compatible[DIRECTION::SOUTH].size(), static_cast<size_t>(1));
    EXPECT_EQ(compatible[DIRECTION::SOUTH][0], static_cast<uint16_t>(bottom));
    ASSERT_EQ(compatible[DIRECTION::EAST].size(), static_cast<size_t>(1));
    EXPECT_EQ(compatible[DIRECTION::EAST][0], static_cast<uint16_t>(top));
    ASSERT_EQ(compatible[DIRECTION::WEST].size(), static_cast<size_t>(1));
    EXPECT_EQ(compatible[DIRECTION::WEST][0], static_cast<uint16_t>(top));
    EXPECT_FALSE(contains(compatible[DIRECTION::EAST], left));
}

TEST(TilesCompatibility, RelationIsSymmetric)
{
    const Tiles tiles = murosimproc::wfc::preset::getPreset(murosimproc::wfc::preset::PresetType::BLOCK);
    const std::array<size_t, DIRECTION::SIZE> opposite = {DIRECTION::SOUTH, DIRECTION::WEST, DIRECTION::NORTH,
                                                          DIRECTION::EAST};

    ASSERT_EQ(tiles.getCompatible().size(), tiles.getPatterns().size());
    for (size_t p1 = 0; p1 < tiles.getCompatible().size(); p1++)
    {
        for (size_t direction = 0; direction < DIRECTION::SIZE; direction++)
        {
            for (const uint16_t p2 : tiles.getCompatible()[p1][direction])
            {
                ASSERT_TRUE(contains(tiles.getCompatible()[p2][opposite[direction]], p1))
                    << "pattern " << p1 << " direction " << direction << " neighbour " << p2;
            }
        }
    }
}

TEST(TilesCompatibility, EveryPatternHasANeighbourInEveryDirection)
{
    const Tiles tiles = murosimproc::wfc::preset::getPreset(murosimproc::wfc::preset::PresetType::BLOCK);

    for (size_t p = 0; p < tiles.getCompatible().size(); p++)
    {
        for (size_t direction = 0; direction < DIRECTION::SIZE; direction++)
        {
            ASSERT_FALSE(tiles.getCompatible()[p][direction].empty()) << "pattern " << p << " direction " << direction;
        }
    }
}
