// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/wfc/preset.hpp"
#include "murosimproc/wfc/wfc.hpp"
#include "fixtures.hpp"
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace
{
using murosimproc::util::Array2d;
using murosimproc::wfc::Tiles;
using murosimproc::wfc::preset::getAllPresetConfigs;
using murosimproc::wfc::preset::getPreset;
using murosimproc::wfc::preset::getPresetConfig;
using murosimproc::wfc::preset::getPresetName;
using murosimproc::wfc::preset::parseStringToTiles;
using murosimproc::wfc::preset::PRESET_COUNT;
using murosimproc::wfc::preset::PresetConfig;
using murosimproc::wfc::preset::PresetType;

constexpr size_t PATTERN_SIZE = 3; // every preset uses 3 x 3 tiles with stride 1
constexpr size_t CELLS = 4;
constexpr size_t ATTEMPTS = 10;
const std::array<std::string_view, PRESET_COUNT> NAMES = {"Block", "Obstacle", "Labyrinth", "Warehouse",
                                                          "Room",  "Sparse",   "Cavern",    "Lines"};

PresetType presetAt(const size_t index)
{
    return static_cast<PresetType>(index);
}
} // namespace

// ============================================================================
// Config Tests
// ============================================================================

TEST(PresetConfigs, EightPresets)
{
    EXPECT_EQ(PRESET_COUNT, static_cast<size_t>(8));
    EXPECT_EQ(getAllPresetConfigs().size(), PRESET_COUNT);
}

TEST(PresetConfigs, NamesMatchTheEnumOrder)
{
    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        EXPECT_EQ(getPresetName(presetAt(i)), NAMES[i]) << "preset " << i;
        EXPECT_EQ(getPresetConfig(presetAt(i)).name, NAMES[i]) << "preset " << i;
        EXPECT_EQ(getAllPresetConfigs()[i].name, NAMES[i]) << "preset " << i;
    }
}

TEST(PresetConfigs, EveryPresetUsesThreeByThreeTilesWithStrideOne)
{
    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        const PresetConfig &config = getPresetConfig(presetAt(i));
        EXPECT_EQ(config.pattern_size, PATTERN_SIZE) << "preset " << i;
        EXPECT_EQ(config.stride, static_cast<size_t>(1)) << "preset " << i;
        EXPECT_FALSE(config.pattern.empty()) << "preset " << i;
    }
}

TEST(PresetConfigs, CountIsNotAPreset)
{
    EXPECT_THROW(getPresetConfig(PresetType::COUNT), std::out_of_range);
    EXPECT_THROW(getPresetName(PresetType::COUNT), std::out_of_range);
    EXPECT_THROW(getPreset(PresetType::COUNT), std::out_of_range);
}

// ============================================================================
// Tile Tests
// ============================================================================

TEST(PresetTiles, EveryPresetParsesIntoWeightedTiles)
{
    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        const Tiles tiles = getPreset(presetAt(i));
        ASSERT_FALSE(tiles.getPatterns().empty()) << "preset " << i;
        EXPECT_EQ(tiles.getPatternSize(), PATTERN_SIZE) << "preset " << i;
        EXPECT_EQ(tiles.getWeights().size(), tiles.getPatterns().size()) << "preset " << i;
        double weight_sum = 0.0;
        for (const double weight : tiles.getWeights())
        {
            weight_sum += weight;
        }
        EXPECT_NEAR(weight_sum, 1.0, 1e-9) << "preset " << i;
    }
}

TEST(PresetTiles, ConfigToTilesMatchesGetPreset)
{
    const Tiles from_config = getPresetConfig(PresetType::ROOM).toTiles();
    const Tiles from_enum = getPreset(PresetType::ROOM);

    ASSERT_EQ(from_config.getPatterns().size(), from_enum.getPatterns().size());
    for (size_t i = 0; i < from_config.getPatterns().size(); i++)
    {
        ASSERT_EQ(from_config.getPatterns()[i], from_enum.getPatterns()[i]) << "pattern " << i;
    }
}

TEST(PresetTiles, EveryPresetGeneratesAnImage)
{
    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        murosimproc::wfc::WaveFunctionCollapse wfc(getPreset(presetAt(i)));
        const Array2d<uint8_t> image = wfc.run(CELLS, CELLS, fixtures::SEED, ATTEMPTS);
        ASSERT_FALSE(image.empty()) << "preset " << i;
        EXPECT_EQ(image.sizeX(), CELLS * PATTERN_SIZE) << "preset " << i;
    }
}

// ============================================================================
// Parse Tests
// ============================================================================

TEST(PresetParse, CharactersMapToCellValues)
{
    const Tiles tiles = parseStringToTiles("  \n  \n", 2);
    ASSERT_EQ(tiles.getPatterns().size(), static_cast<size_t>(1));
    EXPECT_EQ(tiles.getPatterns()[0](0, 0), static_cast<uint8_t>(0));

    const Tiles walls = parseStringToTiles("##\n##\n", 2);
    ASSERT_EQ(walls.getPatterns().size(), static_cast<size_t>(1));
    EXPECT_EQ(walls.getPatterns()[0](1, 1), static_cast<uint8_t>(1));

    const Tiles obstacles = parseStringToTiles("..\n..\n", 2);
    ASSERT_EQ(obstacles.getPatterns().size(), static_cast<size_t>(1));
    EXPECT_EQ(obstacles.getPatterns()[0](0, 1), static_cast<uint8_t>(2));
}

TEST(PresetParse, RowsAreImageRows)
{
    const Tiles tiles = parseStringToTiles("##\n  ", 2);

    ASSERT_EQ(tiles.getPatterns().size(), static_cast<size_t>(4));
    bool has_top_wall = false;
    for (const Array2d<uint8_t> &pattern : tiles.getPatterns())
    {
        has_top_wall = has_top_wall || pattern == fixtures::makeImage({"##", "  "});
    }
    EXPECT_TRUE(has_top_wall);
}

TEST(PresetParse, TrailingNewlineIsOptional)
{
    const Tiles with = parseStringToTiles("# \n  \n", 2);
    const Tiles without = parseStringToTiles("# \n  ", 2);

    EXPECT_EQ(with.getPatterns().size(), without.getPatterns().size());
}

TEST(PresetParse, StrideIsPassedThrough)
{
    const Tiles tiles = parseStringToTiles("#  \n   \n   \n", 2, 2);

    EXPECT_EQ(tiles.getStride(), static_cast<size_t>(2));
    EXPECT_EQ(tiles.getPatterns().size(), static_cast<size_t>(4)); // only the corner window
}

TEST(PresetParse, InvalidArgumentsThrow)
{
    EXPECT_THROW(parseStringToTiles("##\n##\n", 1), std::invalid_argument);   // pattern size below 2
    EXPECT_THROW(parseStringToTiles("", 2), std::invalid_argument);           // empty
    EXPECT_THROW(parseStringToTiles("##\n#\n", 2), std::invalid_argument);    // ragged rows
    EXPECT_THROW(parseStringToTiles("##\n##\n", 3), std::invalid_argument);   // smaller than the pattern
    EXPECT_THROW(parseStringToTiles("##\n##\n", 2, 0), std::invalid_argument); // stride 0
    EXPECT_THROW(parseStringToTiles("#x\n##\n", 2), std::invalid_argument);   // unknown character
}
