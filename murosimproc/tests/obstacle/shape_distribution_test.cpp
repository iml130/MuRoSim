// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/obstacle/shape_distribution.hpp"
#include "fixtures.hpp"
#include <stdexcept>

namespace
{
using murosimproc::generators::RandomRange;
using murosimproc::obstacle::ShapeDistribution;
using murosim::random::RandomGenerator;

constexpr int DRAWS = 50;
} // namespace

// ============================================================================
// Default Tests
// ============================================================================

TEST(ShapeDistributionDefaults, WeightsAreFixedValues)
{
    const ShapeDistribution shape;

    EXPECT_FLOAT_EQ(shape.circle.min(), 0.5f);
    EXPECT_FLOAT_EQ(shape.circle.max(), 0.5f);
    EXPECT_FLOAT_EQ(shape.polygon.min(), 0.5f);
    EXPECT_FLOAT_EQ(shape.polygon.max(), 0.5f);
    EXPECT_FLOAT_EQ(shape.decompose.min(), 0.3f);
    EXPECT_FLOAT_EQ(shape.decompose.max(), 0.3f);
    EXPECT_FLOAT_EQ(shape.min_polygon_radius, 0.05f);
}

TEST(ShapeDistributionDefaults, FixedWeightsDrawWithoutTouchingTheGenerator)
{
    const ShapeDistribution shape;
    RandomGenerator random_generator(fixtures::SEED);
    RandomGenerator untouched(fixtures::SEED);

    EXPECT_FLOAT_EQ(shape.circle(random_generator), 0.5f);
    EXPECT_FLOAT_EQ(shape.polygon(random_generator), 0.5f);
    EXPECT_FLOAT_EQ(shape.decompose(random_generator), 0.3f);
    EXPECT_EQ(random_generator(), untouched());
}

// ============================================================================
// Range Tests
// ============================================================================

TEST(ShapeDistributionRanges, WeightsCanBeRanges)
{
    const ShapeDistribution shape = {RandomRange(0.2f, 0.8f), RandomRange(0.0f, 1.0f), RandomRange(0.1f, 0.4f), 0.1f};

    EXPECT_FLOAT_EQ(shape.circle.min(), 0.2f);
    EXPECT_FLOAT_EQ(shape.circle.max(), 0.8f);
    EXPECT_FLOAT_EQ(shape.polygon.max(), 1.0f);
    EXPECT_FLOAT_EQ(shape.decompose.min(), 0.1f);
    EXPECT_FLOAT_EQ(shape.min_polygon_radius, 0.1f);
}

TEST(ShapeDistributionRanges, DrawsStayInsideTheRanges)
{
    const ShapeDistribution shape = {RandomRange(0.2f, 0.8f), RandomRange(0.0f, 1.0f), RandomRange(0.1f, 0.4f), 0.1f};
    RandomGenerator random_generator(fixtures::SEED);

    for (int i = 0; i < DRAWS; i++)
    {
        const float circle = shape.circle(random_generator);
        ASSERT_GE(circle, 0.2f) << "draw " << i;
        ASSERT_LE(circle, 0.8f) << "draw " << i;
        const float decompose = shape.decompose(random_generator);
        ASSERT_GE(decompose, 0.1f) << "draw " << i;
        ASSERT_LE(decompose, 0.4f) << "draw " << i;
    }
}

TEST(ShapeDistributionRanges, ReversedRangeThrows)
{
    EXPECT_THROW(RandomRange(0.8f, 0.2f), std::invalid_argument);
}
