// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/random/distribution.hpp"
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
using murosim::random::NormalDistribution;
using murosim::random::RandomGenerator;
using murosim::random::uniformFloat;
using murosim::random::uniformUnit;

constexpr uint64_t SEED = 42;
constexpr int SAMPLES = 200000;
constexpr float STDDEV = 0.25f;
constexpr float MEAN = 1.5f;
// Sampling error of mean and standard deviation over SAMPLES draws is
// ~STDDEV / sqrt(SAMPLES) = 5.6e-4; ten times that is a safe bound and the
// seed is fixed, so this is deterministic anyway.
constexpr float STATISTICAL_TOLERANCE = 0.006f;
constexpr float UNIT_STEP = 1.0f / 16777216.0f; // 2^-24, the resolution of uniformUnit
} // namespace

// ============================================================================
// Uniform Tests
// ============================================================================

TEST(DistributionUniform, UnitDrawsStayInTheHalfOpenInterval)
{
    RandomGenerator random_generator(SEED);

    for (int i = 0; i < SAMPLES; i++)
    {
        const float value = uniformUnit(random_generator);
        ASSERT_GE(value, 0.0f) << "draw " << i;
        ASSERT_LT(value, 1.0f) << "draw " << i;
    }
}

TEST(DistributionUniform, UnitDrawsAreMultiplesOfTheResolution)
{
    RandomGenerator random_generator(SEED);

    for (int i = 0; i < 1000; i++)
    {
        const float value = uniformUnit(random_generator);
        const float steps = value / UNIT_STEP;
        ASSERT_FLOAT_EQ(steps, std::round(steps)) << "draw " << i;
    }
}

TEST(DistributionUniform, UnitMeanIsOneHalf)
{
    RandomGenerator random_generator(SEED);

    double sum = 0.0;
    for (int i = 0; i < SAMPLES; i++)
    {
        sum += static_cast<double>(uniformUnit(random_generator));
    }
    EXPECT_NEAR(sum / SAMPLES, 0.5, STATISTICAL_TOLERANCE);
}

TEST(DistributionUniform, RangeDrawsStayInsideAndFillTheRange)
{
    RandomGenerator random_generator(SEED);

    float lowest = 1.0f;
    float highest = -1.0f;
    for (int i = 0; i < SAMPLES; i++)
    {
        const float value = uniformFloat(random_generator, -0.5f, 0.5f);
        ASSERT_GE(value, -0.5f) << "draw " << i;
        ASSERT_LT(value, 0.5f) << "draw " << i;
        lowest = std::min(lowest, value);
        highest = std::max(highest, value);
    }
    EXPECT_LT(lowest, -0.49f);
    EXPECT_GT(highest, 0.49f);
}

TEST(DistributionUniform, SameSeedGivesTheSameDraws)
{
    RandomGenerator first(SEED);
    RandomGenerator second(SEED);

    for (int i = 0; i < 1000; i++)
    {
        ASSERT_FLOAT_EQ(uniformUnit(first), uniformUnit(second)) << "draw " << i;
    }
}

TEST(DistributionUniform, OneDrawConsumesOneGeneratorValue)
{
    RandomGenerator random_generator(SEED);
    RandomGenerator reference(SEED);

    uniformUnit(random_generator);
    reference();

    EXPECT_EQ(random_generator(), reference());
}

// ============================================================================
// Normal Tests
// ============================================================================

TEST(DistributionNormal, MeanAndStandardDeviationMatch)
{
    RandomGenerator random_generator(SEED);
    NormalDistribution distribution(MEAN, STDDEV);

    double sum = 0.0;
    double sum_squares = 0.0;
    for (int i = 0; i < SAMPLES; i++)
    {
        const double value = static_cast<double>(distribution(random_generator));
        sum += value;
        sum_squares += value * value;
    }
    const double mean = sum / SAMPLES;
    const double stddev = std::sqrt(sum_squares / SAMPLES - mean * mean);
    EXPECT_NEAR(mean, MEAN, STATISTICAL_TOLERANCE);
    EXPECT_NEAR(stddev, STDDEV, STATISTICAL_TOLERANCE);
}

TEST(DistributionNormal, DrawsStayWithinSixSigma)
{
    RandomGenerator random_generator(SEED);
    NormalDistribution distribution(0.0f, STDDEV);

    for (int i = 0; i < SAMPLES; i++)
    {
        ASSERT_LT(std::abs(distribution(random_generator)), 6.0f * STDDEV) << "draw " << i;
    }
}

TEST(DistributionNormal, ZeroStandardDeviationIsTheMean)
{
    RandomGenerator random_generator(SEED);
    NormalDistribution distribution(MEAN, 0.0f);

    for (int i = 0; i < 100; i++)
    {
        ASSERT_FLOAT_EQ(distribution(random_generator), MEAN) << "draw " << i;
    }
}

TEST(DistributionNormal, SameSeedGivesTheSameDraws)
{
    RandomGenerator first(SEED);
    RandomGenerator second(SEED);
    NormalDistribution distribution_first(0.0f, STDDEV);
    NormalDistribution distribution_second(0.0f, STDDEV);

    for (int i = 0; i < 1000; i++)
    {
        ASSERT_FLOAT_EQ(distribution_first(first), distribution_second(second)) << "draw " << i;
    }
}

// The polar method produces two values per accepted pair: the second draw
// comes from the spare and consumes no generator output.
TEST(DistributionNormal, SecondDrawComesFromTheSpare)
{
    RandomGenerator random_generator(SEED);
    RandomGenerator reference(SEED);
    NormalDistribution distribution(0.0f, STDDEV);

    distribution(random_generator);
    const uint64_t after_first = RandomGenerator(random_generator)();
    distribution(random_generator);
    const uint64_t after_second = RandomGenerator(random_generator)();

    EXPECT_EQ(after_first, after_second);
}
