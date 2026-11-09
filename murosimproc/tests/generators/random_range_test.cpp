// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/generators/random_range.hpp"
#include "fixtures.hpp"
#include <stdexcept>

namespace
{
using murosimproc::generators::RandomRange;
using murosim::random::RandomGenerator;

constexpr int DRAWS = 100;

float takeRange(const RandomRange range)
{
    return range.max() - range.min();
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(RandomRangeConstructor, SingleValueIsAFixedRange)
{
    const RandomRange range(0.5f);

    EXPECT_FLOAT_EQ(range.min(), 0.5f);
    EXPECT_FLOAT_EQ(range.max(), 0.5f);
}

TEST(RandomRangeConstructor, OrderedRangeIsKept)
{
    const RandomRange range(0.7f, 1.25f);

    EXPECT_FLOAT_EQ(range.min(), 0.7f);
    EXPECT_FLOAT_EQ(range.max(), 1.25f);
}

TEST(RandomRangeConstructor, EqualBoundsAreAFixedRange)
{
    const RandomRange range(2.0f, 2.0f);

    EXPECT_FLOAT_EQ(range.min(), range.max());
}

TEST(RandomRangeConstructor, ReversedBoundsThrow)
{
    EXPECT_THROW(RandomRange(1.25f, 0.7f), std::invalid_argument);
}

TEST(RandomRangeConstructor, FloatConvertsImplicitly)
{
    EXPECT_FLOAT_EQ(takeRange(0.5f), 0.0f);
    EXPECT_FLOAT_EQ(takeRange({0.5f, 1.5f}), 1.0f);
}

// ============================================================================
// Draw Tests
// ============================================================================

TEST(RandomRangeDraw, FixedRangeReturnsTheValueWithoutDrawing)
{
    const RandomRange range(0.5f);
    RandomGenerator random_generator(fixtures::SEED);
    RandomGenerator untouched(fixtures::SEED);

    EXPECT_FLOAT_EQ(range(random_generator), 0.5f);
    EXPECT_EQ(random_generator(), untouched());
}

TEST(RandomRangeDraw, DrawsStayInsideTheBounds)
{
    const RandomRange range(0.7f, 1.25f);
    RandomGenerator random_generator(fixtures::SEED);

    for (int i = 0; i < DRAWS; i++)
    {
        const float value = range(random_generator);
        ASSERT_GE(value, 0.7f) << "draw " << i;
        ASSERT_LE(value, 1.25f) << "draw " << i;
    }
}

TEST(RandomRangeDraw, DrawsAdvanceTheGeneratorAndVary)
{
    const RandomRange range(0.0f, 1.0f);
    RandomGenerator random_generator(fixtures::SEED);

    const float first = range(random_generator);
    const float second = range(random_generator);

    EXPECT_NE(first, second);
}

TEST(RandomRangeDraw, SameSeedGivesTheSameDraws)
{
    const RandomRange range(0.0f, 1.0f);
    RandomGenerator first(fixtures::SEED);
    RandomGenerator second(fixtures::SEED);

    for (int i = 0; i < DRAWS; i++)
    {
        ASSERT_FLOAT_EQ(range(first), range(second)) << "draw " << i;
    }
}
