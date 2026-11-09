// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/random/random_generator.hpp"
#include "murosim/random/splitmix64.hpp"
#include "murosim/random/xoshiro256.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <random>
#include <set>
#include <type_traits>
#include <vector>

namespace
{
using murosim::random::RandomGenerator;
using murosim::random::SplitMix64;
using murosim::random::Xoshiro256Plus;
using murosim::random::Xoshiro256PlusPlus;
using murosim::random::Xoshiro256StarStar;

constexpr uint64_t SEED = 42;
constexpr int SEQUENCE_LENGTH = 1000;
constexpr int SAMPLES = 10000;
constexpr int NUM_BUCKETS = 10;

// The distribution tests draw SAMPLES values from a fixed seed, so they are
// deterministic; the tolerances only have to cover the sampling error of that
// one run. For 10000 draws the standard error of the mean of N(0, 1) is 0.01
// and that of a bucket count of 1000 is about 30, so 0.1 and 20 % leave a
// wide margin.
constexpr float STATISTICAL_TOLERANCE = 0.1f;
constexpr float BUCKET_TOLERANCE = 0.2f;

/// Expects the first draws of `rng` to equal the reference sequence.
template <typename Generator, size_t N>
void assertReferenceSequence(Generator &rng, const std::array<uint64_t, N> &expected)
{
    for (size_t i = 0; i < N; i++)
    {
        ASSERT_EQ(rng(), expected[i]) << "draw " << i;
    }
}

/// Number of distinct values among `count` draws.
template <typename Generator>
size_t distinctValues(Generator &rng, const int count)
{
    std::set<uint64_t> values;
    for (int i = 0; i < count; ++i)
    {
        values.insert(rng());
    }
    return values.size();
}

/// True if the two generators disagree at least once within `count` draws.
template <typename Generator>
bool sequencesDiffer(Generator &rng1, Generator &rng2, const int count)
{
    for (int i = 0; i < count; ++i)
    {
        if (rng1() != rng2())
        {
            return true;
        }
    }
    return false;
}
} // namespace

// ============================================================================
// Reference Sequence Tests
// ============================================================================

// These freeze the exact output stream so that any future change to the
// generators (refactor, "optimization", constant typo) is caught immediately,
// which is what guarantees that experiments stay reproducible. The SplitMix64
// seed-0 sequence additionally matches the published canonical reference.

TEST(SplitMix64Reference, Seed0)
{
    SplitMix64 rng(0);
    assertReferenceSequence(rng, std::array<uint64_t, 4>{0xe220a8397b1dcdafULL, 0x6e789e6aa1b965f4ULL,
                                                         0x06c45d188009454fULL, 0xf88bb8a8724c81ecULL});
}

TEST(SplitMix64Reference, Seed42)
{
    SplitMix64 rng(SEED);
    assertReferenceSequence(rng, std::array<uint64_t, 4>{0xbdd732262feb6e95ULL, 0x28efe333b266f103ULL,
                                                         0x47526757130f9f52ULL, 0x581ce1ff0e4ae394ULL});
}

TEST(Xoshiro256Reference, PlusSeed42)
{
    Xoshiro256Plus rng(SEED);
    assertReferenceSequence(rng, std::array<uint64_t, 4>{0x15f414253e365229ULL, 0x4f771f08f4211387ULL,
                                                         0x100492bd8828891eULL, 0x4e743fce495374aeULL});
}

TEST(Xoshiro256Reference, PlusPlusSeed42)
{
    Xoshiro256PlusPlus rng(SEED);
    assertReferenceSequence(rng, std::array<uint64_t, 4>{0xd0764d4f4476689fULL, 0x519e4174576f3791ULL,
                                                         0xfbe07cfb0c24ed8cULL, 0xb37d9f600cd835b8ULL});
}

TEST(Xoshiro256Reference, StarStarSeed42)
{
    Xoshiro256StarStar rng(SEED);
    assertReferenceSequence(rng, std::array<uint64_t, 4>{0x15780b2e0c2ec716ULL, 0x6104d9866d113a7eULL,
                                                         0xae17533239e499a1ULL, 0xecb8ad4703b360a1ULL});
}

// ============================================================================
// Generator Contract Tests
// ============================================================================

// The same contract, instantiated for every engine via typed tests

template <typename Gen>
class GeneratorContract : public ::testing::Test
{
};

using AllGenerators = ::testing::Types<SplitMix64, Xoshiro256Plus, Xoshiro256PlusPlus, Xoshiro256StarStar>;
TYPED_TEST_SUITE(GeneratorContract, AllGenerators);

TYPED_TEST(GeneratorContract, ResultTypeIs64Bit)
{
    static_assert(std::is_same<typename TypeParam::result_type, uint64_t>::value,
                  "all generators must produce 64-bit output");
    EXPECT_EQ(TypeParam::min(), std::numeric_limits<uint64_t>::min());
    EXPECT_EQ(TypeParam::max(), std::numeric_limits<uint64_t>::max());
}

TYPED_TEST(GeneratorContract, SameSeedSameSequence)
{
    TypeParam rng1(12345);
    TypeParam rng2(12345);
    for (int i = 0; i < SEQUENCE_LENGTH; ++i)
    {
        ASSERT_EQ(rng1(), rng2()) << "draw " << i;
    }
}

TYPED_TEST(GeneratorContract, DifferentSeedDifferentSequence)
{
    TypeParam rng1(12345);
    TypeParam rng2(54321);

    EXPECT_TRUE(sequencesDiffer(rng1, rng2, 100));
}

TYPED_TEST(GeneratorContract, NextAndOperatorEquivalent)
{
    TypeParam rng1(SEED);
    TypeParam rng2(SEED);
    for (int i = 0; i < 100; ++i)
    {
        ASSERT_EQ(rng1.next(), rng2()) << "draw " << i;
    }
}

TYPED_TEST(GeneratorContract, ProducesDiverseValues)
{
    // With 64-bit output, collisions in 1000 draws are astronomically unlikely
    TypeParam rng(SEED);

    EXPECT_EQ(distinctValues(rng, SEQUENCE_LENGTH), static_cast<size_t>(SEQUENCE_LENGTH));
}

TYPED_TEST(GeneratorContract, ZeroSeedIsWellBehaved)
{
    TypeParam rng(0);

    EXPECT_EQ(distinctValues(rng, SEQUENCE_LENGTH), static_cast<size_t>(SEQUENCE_LENGTH));
}

TYPED_TEST(GeneratorContract, MaxSeedIsWellBehaved)
{
    TypeParam rng(std::numeric_limits<uint64_t>::max());

    EXPECT_EQ(distinctValues(rng, SEQUENCE_LENGTH), static_cast<size_t>(SEQUENCE_LENGTH));
}

// ============================================================================
// Xoshiro256 Jump Tests
// ============================================================================

TEST(Xoshiro256Jump, JumpIsDeterministic)
{
    Xoshiro256Plus jumped1(SEED);
    Xoshiro256Plus jumped2(SEED);
    jumped1.jump();
    jumped2.jump();

    for (int i = 0; i < 100; ++i)
    {
        ASSERT_EQ(jumped1(), jumped2()) << "draw " << i;
    }
}

TEST(Xoshiro256Jump, JumpedStreamDiffersFromBaseStream)
{
    Xoshiro256Plus base(SEED);
    Xoshiro256Plus jumped(SEED);
    jumped.jump();

    EXPECT_TRUE(sequencesDiffer(base, jumped, 100));
}

// ============================================================================
// Xoshiro256 Variant Tests
// ============================================================================

TEST(Xoshiro256Variants, DifferentScramblersDifferentOutput)
{
    Xoshiro256Plus plus(SEED);
    Xoshiro256PlusPlus plusplus(SEED);
    Xoshiro256StarStar starstar(SEED);

    bool all_equal = true;
    for (int i = 0; i < 100; ++i)
    {
        const uint64_t a = plus();
        const uint64_t b = plusplus();
        const uint64_t c = starstar();
        if (a != b || b != c)
        {
            all_equal = false;
            break;
        }
    }
    EXPECT_FALSE(all_equal);
}

// ============================================================================
// Random Generator Seed Tests
// ============================================================================

TEST(RandomGeneratorSeed, DefaultConstructorProducesDistinctValues)
{
    // Seeded from hardware, so only the diversity can be checked
    RandomGenerator rng;

    EXPECT_EQ(distinctValues(rng, 100), static_cast<size_t>(100));
}

TEST(RandomGeneratorSeed, SeedGetterReportsSeed)
{
    const RandomGenerator rng(0xABCDEF0123456789ULL);

    EXPECT_EQ(rng.seed(), 0xABCDEF0123456789ULL);
}

TEST(RandomGeneratorSeed, DefaultConstructedIsReproducibleFromReportedSeed)
{
    RandomGenerator a;
    RandomGenerator b(a.seed());

    for (int i = 0; i < 100; ++i)
    {
        ASSERT_EQ(a(), b()) << "draw " << i;
    }
}

TEST(RandomGeneratorSeed, SameSeedSameSequence)
{
    RandomGenerator rng1(12345);
    RandomGenerator rng2(12345);

    for (int i = 0; i < SEQUENCE_LENGTH; ++i)
    {
        ASSERT_EQ(rng1(), rng2()) << "draw " << i;
    }
}

TEST(RandomGeneratorSeed, MinMaxAre64Bit)
{
    EXPECT_EQ(RandomGenerator::min(), std::numeric_limits<uint64_t>::min());
    EXPECT_EQ(RandomGenerator::max(), std::numeric_limits<uint64_t>::max());
}

TEST(RandomGeneratorSeed, LongSequenceMatchesReference)
{
    // Pins the state transition over a long run: the value after one million
    // draws of RandomGenerator(42)
    RandomGenerator rng(SEED);
    for (int i = 0; i < 1000000; ++i)
    {
        rng();
    }

    EXPECT_EQ(rng(), 0xe36453b799f7c621ULL);
}

// ============================================================================
// Random Generator Child Tests
// ============================================================================

TEST(RandomGeneratorChild, CreateChildIsDeterministic)
{
    RandomGenerator parent1(SEED);
    RandomGenerator parent2(SEED);
    RandomGenerator child1 = parent1.createRandomGenerator();
    RandomGenerator child2 = parent2.createRandomGenerator();

    for (int i = 0; i < 100; ++i)
    {
        ASSERT_EQ(child1(), child2()) << "draw " << i;
    }
}

TEST(RandomGeneratorChild, ChildrenAreIndependent)
{
    RandomGenerator parent(SEED);
    RandomGenerator child1 = parent.createRandomGenerator();
    RandomGenerator child2 = parent.createRandomGenerator();

    EXPECT_TRUE(sequencesDiffer(child1, child2, 100));
}

// ============================================================================
// Test Fixture
// ============================================================================

// STL distribution compatibility (UniformRandomBitGenerator conformance)
class RandomGeneratorTest : public ::testing::Test
{
protected:
    RandomGenerator rng{SEED};
};

// ============================================================================
// Distribution Tests
// ============================================================================

TEST_F(RandomGeneratorTest, UniformIntDistribution)
{
    std::uniform_int_distribution<int> dist(0, 100);
    int min_val = 100;
    int max_val = 0;
    for (int i = 0; i < SEQUENCE_LENGTH; ++i)
    {
        const int value = dist(rng);
        ASSERT_GE(value, 0) << "draw " << i;
        ASSERT_LE(value, 100) << "draw " << i;
        min_val = std::min(min_val, value);
        max_val = std::max(max_val, value);
    }

    // Both ends of the range are reached
    EXPECT_LT(min_val, 10);
    EXPECT_GT(max_val, 90);
}

TEST_F(RandomGeneratorTest, UniformRealDistribution)
{
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    float min_val = 1.0f;
    float max_val = 0.0f;
    for (int i = 0; i < SEQUENCE_LENGTH; ++i)
    {
        const float value = dist(rng);
        ASSERT_GE(value, 0.0f) << "draw " << i;
        ASSERT_LT(value, 1.0f) << "draw " << i;
        min_val = std::min(min_val, value);
        max_val = std::max(max_val, value);
    }

    // Both ends of the range are reached
    EXPECT_LT(min_val, 0.1f);
    EXPECT_GT(max_val, 0.9f);
}

TEST_F(RandomGeneratorTest, NormalDistribution)
{
    std::normal_distribution<float> dist(0.0f, 1.0f);
    float sum = 0.0f;
    float sum_sq = 0.0f;
    for (int i = 0; i < SAMPLES; ++i)
    {
        const float value = dist(rng);
        sum += value;
        sum_sq += value * value;
    }
    const float mean = sum / static_cast<float>(SAMPLES);
    const float variance = (sum_sq / static_cast<float>(SAMPLES)) - (mean * mean);

    EXPECT_NEAR(mean, 0.0f, STATISTICAL_TOLERANCE);
    EXPECT_NEAR(variance, 1.0f, STATISTICAL_TOLERANCE);
}

TEST_F(RandomGeneratorTest, UniformityBuckets)
{
    std::vector<int> buckets(NUM_BUCKETS, 0);
    std::uniform_int_distribution<int> dist(0, NUM_BUCKETS - 1);
    for (int i = 0; i < SAMPLES; ++i)
    {
        buckets[static_cast<size_t>(dist(rng))]++;
    }

    const float expected = static_cast<float>(SAMPLES / NUM_BUCKETS);
    for (size_t i = 0; i < buckets.size(); ++i)
    {
        EXPECT_GT(buckets[i], static_cast<int>(expected * (1.0f - BUCKET_TOLERANCE))) << "bucket " << i;
        EXPECT_LT(buckets[i], static_cast<int>(expected * (1.0f + BUCKET_TOLERANCE))) << "bucket " << i;
    }
}
