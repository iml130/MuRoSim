// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/wfc/wave.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <vector>

namespace
{
using murosimproc::util::Array2d;
using murosimproc::wfc::EntropyResult;
using murosimproc::wfc::Wave;
using murosim::random::RandomGenerator;

constexpr size_t SIZE_X = 3;
constexpr size_t SIZE_Y = 2;
const std::vector<double> EQUAL_WEIGHTS = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
// Removing the heavy pattern raises a cell's entropy above the untouched
// cell's, removing a light one lowers it (see LowestEntropyWins).
const std::vector<double> SKEWED_WEIGHTS = {0.8, 0.1, 0.1};

/// Leaves only `pattern` possible in the cell at `index`.
void collapse(Wave &wave, const size_t index, const size_t pattern)
{
    for (size_t p = 0; p < EQUAL_WEIGHTS.size(); p++)
    {
        wave.set(index, p, p == pattern);
    }
}
} // namespace

// ============================================================================
// Entropy Result Tests
// ============================================================================

TEST(EntropyResultFactories, CellNamesTheIndex)
{
    const EntropyResult result = EntropyResult::cell(5);

    EXPECT_TRUE(result.isCell());
    EXPECT_FALSE(result.isFinished());
    EXPECT_FALSE(result.isContradiction());
    EXPECT_EQ(result.status, EntropyResult::Status::Cell);
    EXPECT_EQ(result.cell_index, static_cast<size_t>(5));
}

TEST(EntropyResultFactories, FinishedAndContradictionAreExclusive)
{
    const EntropyResult finished = EntropyResult::finished();
    const EntropyResult contradiction = EntropyResult::contradiction();

    EXPECT_TRUE(finished.isFinished());
    EXPECT_FALSE(finished.isCell());
    EXPECT_FALSE(finished.isContradiction());
    EXPECT_TRUE(contradiction.isContradiction());
    EXPECT_FALSE(contradiction.isCell());
    EXPECT_FALSE(contradiction.isFinished());
}

// ============================================================================
// Test Fixture
// ============================================================================

class WaveTest : public ::testing::Test
{
protected:
    Wave wave{SIZE_X, SIZE_Y, EQUAL_WEIGHTS};
    RandomGenerator random_generator{fixtures::SEED};
};

// ============================================================================
// Pattern Set Tests
// ============================================================================

TEST_F(WaveTest, EveryPatternIsPossibleAtFirst)
{
    for (size_t index = 0; index < SIZE_X * SIZE_Y; index++)
    {
        for (size_t p = 0; p < EQUAL_WEIGHTS.size(); p++)
        {
            EXPECT_TRUE(wave.get(index, p)) << "cell " << index << " pattern " << p;
        }
    }
}

TEST_F(WaveTest, SetDisablesAndReenablesAPattern)
{
    wave.set(4, 1, false);
    EXPECT_FALSE(wave.get(4, 1));
    EXPECT_TRUE(wave.get(4, 0));
    EXPECT_TRUE(wave.get(3, 1));

    wave.set(4, 1, true);
    EXPECT_TRUE(wave.get(4, 1));
}

TEST_F(WaveTest, CoordinateOverloadsUseRowMajorIndices)
{
    wave.set(2, 1, 0, false); // index 2 + 1 * SIZE_X = 5

    EXPECT_FALSE(wave.get(5, 0));
    EXPECT_FALSE(wave.get(2, 1, 0));
    EXPECT_TRUE(wave.get(2, 0, 0));
}

TEST_F(WaveTest, SettingTheSameValueTwiceIsANoOp)
{
    wave.set(0, 0, false);
    wave.set(0, 0, false);
    wave.set(0, 0, true);

    EXPECT_TRUE(wave.get(0, 0));
    EXPECT_TRUE(wave.getMinimalEntropy(random_generator).isCell());
}

// ============================================================================
// Minimal Entropy Tests
// ============================================================================

TEST_F(WaveTest, FreshWaveNamesSomeCell)
{
    const EntropyResult result = wave.getMinimalEntropy(random_generator);

    ASSERT_TRUE(result.isCell());
    EXPECT_LT(result.cell_index, SIZE_X * SIZE_Y);
}

TEST_F(WaveTest, CellWithFewerPatternsIsChosen)
{
    wave.set(4, 2, false);

    const EntropyResult result = wave.getMinimalEntropy(random_generator);

    ASSERT_TRUE(result.isCell());
    EXPECT_EQ(result.cell_index, static_cast<size_t>(4));
}

TEST_F(WaveTest, CollapsedCellsAreSkipped)
{
    collapse(wave, 0, 1);
    wave.set(3, 2, false);

    const EntropyResult result = wave.getMinimalEntropy(random_generator);

    ASSERT_TRUE(result.isCell());
    EXPECT_EQ(result.cell_index, static_cast<size_t>(3));
}

TEST_F(WaveTest, AllCellsCollapsedIsFinished)
{
    for (size_t index = 0; index < SIZE_X * SIZE_Y; index++)
    {
        collapse(wave, index, index % EQUAL_WEIGHTS.size());
    }

    EXPECT_TRUE(wave.getMinimalEntropy(random_generator).isFinished());
}

TEST_F(WaveTest, EmptyingACellIsAContradiction)
{
    for (size_t p = 0; p < EQUAL_WEIGHTS.size(); p++)
    {
        wave.set(1, p, false);
    }

    EXPECT_TRUE(wave.getMinimalEntropy(random_generator).isContradiction());
}

TEST_F(WaveTest, ContradictionIsSticky)
{
    for (size_t p = 0; p < EQUAL_WEIGHTS.size(); p++)
    {
        wave.set(1, p, false);
    }
    wave.set(1, 0, true);

    EXPECT_TRUE(wave.getMinimalEntropy(random_generator).isContradiction());
}

// Shannon entropy, not pattern count, decides: cell 2 keeps 2 patterns
// {0.8, 0.1} (entropy 0.35), cell 1 keeps {0.1, 0.1} (0.69) and cell 0 all
// three (0.64). The tie-breaking noise is below 0.05, so cell 2 always wins.
TEST(WaveEntropy, LowestEntropyWins)
{
    Wave wave(SIZE_X, 1, SKEWED_WEIGHTS);
    RandomGenerator random_generator(fixtures::SEED);
    wave.set(1, 0, false);
    wave.set(2, 1, false);

    const EntropyResult result = wave.getMinimalEntropy(random_generator);

    ASSERT_TRUE(result.isCell());
    EXPECT_EQ(result.cell_index, static_cast<size_t>(2));
}

TEST(WaveEntropy, EntropyIsRestoredWhenAPatternComesBack)
{
    Wave wave(SIZE_X, 1, SKEWED_WEIGHTS);
    RandomGenerator random_generator(fixtures::SEED);
    wave.set(0, 1, false); // cell 0 would win now
    wave.set(0, 1, true);
    wave.set(2, 1, false);

    const EntropyResult result = wave.getMinimalEntropy(random_generator);

    ASSERT_TRUE(result.isCell());
    EXPECT_EQ(result.cell_index, static_cast<size_t>(2));
}

// ============================================================================
// Pattern Array Tests
// ============================================================================

TEST_F(WaveTest, PatternArrayHoldsTheSurvivingPatternPerCell)
{
    for (size_t index = 0; index < SIZE_X * SIZE_Y; index++)
    {
        collapse(wave, index, (index * 2) % EQUAL_WEIGHTS.size());
    }

    const Array2d<uint16_t> patterns = wave.createPatternArray2d();

    ASSERT_EQ(patterns.sizeX(), SIZE_X);
    ASSERT_EQ(patterns.sizeY(), SIZE_Y);
    for (size_t y = 0; y < SIZE_Y; y++)
    {
        for (size_t x = 0; x < SIZE_X; x++)
        {
            const size_t index = x + y * SIZE_X;
            EXPECT_EQ(patterns(x, y), static_cast<uint16_t>((index * 2) % EQUAL_WEIGHTS.size()))
                << "cell " << x << ", " << y;
        }
    }
}
