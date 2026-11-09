// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/wfc/propagator.hpp"
#include "murosimproc/wfc/tiles.hpp"
#include "murosimproc/wfc/wave.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace
{
using murosimproc::util::Array2d;
using murosimproc::wfc::Propagator;
using murosimproc::wfc::Tiles;
using murosimproc::wfc::Wave;
using murosim::random::RandomGenerator;

constexpr size_t PATTERN_SIZE = 2;
constexpr size_t PATTERNS = 4;

/// Index of the pattern equal to `rows` in `tiles` (the half-wall set below).
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

/// Removes every pattern but `pattern` from cell `(x, y)` through the propagator.
void collapse(Wave &wave, Propagator &propagator, const size_t x, const size_t y, const size_t pattern)
{
    for (size_t p = 0; p < PATTERNS; p++)
    {
        if (p != pattern)
        {
            propagator.add(x, y, p);
            wave.set(x, y, p, false);
        }
    }
}

size_t countPossible(const Wave &wave, const size_t x, const size_t y)
{
    size_t count = 0;
    for (size_t p = 0; p < PATTERNS; p++)
    {
        if (wave.get(x, y, p))
        {
            count++;
        }
    }
    return count;
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// The four orientations of a half wall: a top wall continues sideways as a
// top wall and meets a bottom wall above and below; a left wall continues
// up and down and meets a right wall on either side.
class PropagatorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        top = findPattern(tiles, {"##", "  "});
        bottom = findPattern(tiles, {"  ", "##"});
        left = findPattern(tiles, {"# ", "# "});
        right = findPattern(tiles, {" #", " #"});
        ASSERT_EQ(tiles.getPatterns().size(), PATTERNS);
    }

    Tiles tiles{fixtures::makeImage({"##", "  "}), PATTERN_SIZE};
    size_t top = 0;
    size_t bottom = 0;
    size_t left = 0;
    size_t right = 0;
};

// ============================================================================
// Propagation Tests
// ============================================================================

TEST_F(PropagatorTest, NothingQueuedChangesNothing)
{
    Wave wave(3, 3, tiles.getWeights());
    Propagator propagator(3, 3, tiles.getCompatible());

    propagator.propagate(wave);

    for (size_t index = 0; index < 9; index++)
    {
        EXPECT_EQ(countPossible(wave, index % 3, index / 3), PATTERNS) << "cell " << index;
    }
}

TEST_F(PropagatorTest, CollapsedCellRestrictsItsNeighbour)
{
    Wave wave(2, 1, tiles.getWeights());
    Propagator propagator(2, 1, tiles.getCompatible());

    collapse(wave, propagator, 0, 0, top);
    propagator.propagate(wave);

    EXPECT_TRUE(wave.get(1, 0, top));
    EXPECT_FALSE(wave.get(1, 0, bottom));
    EXPECT_FALSE(wave.get(1, 0, left));
    EXPECT_FALSE(wave.get(1, 0, right));
}

TEST_F(PropagatorTest, VerticalNeighbourGetsTheOppositeOrientation)
{
    Wave wave(1, 2, tiles.getWeights());
    Propagator propagator(1, 2, tiles.getCompatible());

    collapse(wave, propagator, 0, 0, top);
    propagator.propagate(wave);

    EXPECT_TRUE(wave.get(0, 1, bottom));
    EXPECT_EQ(countPossible(wave, 0, 1), static_cast<size_t>(1));
}

TEST_F(PropagatorTest, PropagationChainsAcrossTheGrid)
{
    Wave wave(4, 3, tiles.getWeights());
    Propagator propagator(4, 3, tiles.getCompatible());
    RandomGenerator random_generator(fixtures::SEED);

    collapse(wave, propagator, 0, 0, left);
    propagator.propagate(wave);

    for (size_t y = 0; y < 3; y++)
    {
        for (size_t x = 0; x < 4; x++)
        {
            ASSERT_EQ(countPossible(wave, x, y), static_cast<size_t>(1)) << "cell " << x << ", " << y;
            EXPECT_TRUE(wave.get(x, y, x % 2 == 0 ? left : right)) << "cell " << x << ", " << y;
        }
    }
    EXPECT_TRUE(wave.getMinimalEntropy(random_generator).isFinished());
}

TEST_F(PropagatorTest, IncompatibleCollapsesGiveAContradiction)
{
    Wave wave(2, 1, tiles.getWeights());
    Propagator propagator(2, 1, tiles.getCompatible());
    RandomGenerator random_generator(fixtures::SEED);

    collapse(wave, propagator, 0, 0, top);
    collapse(wave, propagator, 1, 0, left);
    propagator.propagate(wave);

    EXPECT_TRUE(wave.getMinimalEntropy(random_generator).isContradiction());
}

TEST_F(PropagatorTest, IndexOverloadAddressesTheSameCell)
{
    Wave wave(3, 2, tiles.getWeights());
    Propagator propagator(3, 2, tiles.getCompatible());

    for (size_t p = 0; p < PATTERNS; p++)
    {
        if (p != top)
        {
            propagator.add(static_cast<size_t>(4), p); // cell (1, 1)
            wave.set(4, p, false);
        }
    }
    propagator.propagate(wave);

    EXPECT_EQ(countPossible(wave, 0, 1), static_cast<size_t>(1));
    EXPECT_TRUE(wave.get(0, 1, top));
    EXPECT_TRUE(wave.get(2, 1, top));
    EXPECT_TRUE(wave.get(1, 0, bottom));
    EXPECT_EQ(countPossible(wave, 1, 0), static_cast<size_t>(1));
}

TEST_F(PropagatorTest, GridEdgeStopsPropagation)
{
    Wave wave(1, 1, tiles.getWeights());
    Propagator propagator(1, 1, tiles.getCompatible());
    RandomGenerator random_generator(fixtures::SEED);

    collapse(wave, propagator, 0, 0, right);
    propagator.propagate(wave);

    EXPECT_TRUE(wave.get(0, 0, right));
    EXPECT_TRUE(wave.getMinimalEntropy(random_generator).isFinished());
}
