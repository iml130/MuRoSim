// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/util/points.hpp"
#include "murosim/random/random_generator.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <limits>
#include <vector>

namespace
{
using murosim::random::RandomGenerator;
using fixtures::EPSILON;
using fixtures::SEED;

const Eigen::Vector2f CORNER_MIN(1.0f, 2.0f);
const Eigen::Vector2f CORNER_MAX(11.0f, 8.0f);
constexpr size_t COUNT = 200;
constexpr float SPACING = 1.0f;
constexpr float NO_NOISE = 0.0f;

void assertInsideRectangle(const std::vector<Eigen::Vector2f> &points)
{
    for (size_t i = 0; i < points.size(); i++)
    {
        ASSERT_GE(points[i].x(), CORNER_MIN.x()) << "point " << i;
        ASSERT_LE(points[i].x(), CORNER_MAX.x()) << "point " << i;
        ASSERT_GE(points[i].y(), CORNER_MIN.y()) << "point " << i;
        ASSERT_LE(points[i].y(), CORNER_MAX.y()) << "point " << i;
    }
}

/// Smallest distance between any two points.
float minimumDistance(const std::vector<Eigen::Vector2f> &points)
{
    float minimum = std::numeric_limits<float>::max();
    for (size_t i = 0; i < points.size(); i++)
    {
        for (size_t j = i + 1; j < points.size(); j++)
        {
            minimum = std::min(minimum, (points[i] - points[j]).norm());
        }
    }
    return minimum;
}

bool samePoints(const std::vector<Eigen::Vector2f> &a, const std::vector<Eigen::Vector2f> &b)
{
    if (a.size() != b.size())
    {
        return false;
    }
    for (size_t i = 0; i < a.size(); i++)
    {
        if (a[i].x() != b[i].x() || a[i].y() != b[i].y())
        {
            return false;
        }
    }
    return true;
}
} // namespace

// ============================================================================
// Uniform Points Tests
// ============================================================================

TEST(UtilPointsUniform, CreatesTheRequestedNumberInsideTheRectangle)
{
    std::vector<Eigen::Vector2f> points;

    murosimproc::util::points::createRandomPointsRectangleUniform(points, SEED, CORNER_MIN, CORNER_MAX, COUNT);

    ASSERT_EQ(points.size(), COUNT);
    assertInsideRectangle(points);
}

TEST(UtilPointsUniform, AppendsToExistingPoints)
{
    std::vector<Eigen::Vector2f> points = {Eigen::Vector2f(0.0f, 0.0f)};

    murosimproc::util::points::createRandomPointsRectangleUniform(points, SEED, CORNER_MIN, CORNER_MAX, 3);

    EXPECT_EQ(points.size(), static_cast<size_t>(4));
    EXPECT_FLOAT_EQ(points.front().x(), 0.0f);
}

TEST(UtilPointsUniform, CornersMayBeGivenInAnyOrder)
{
    std::vector<Eigen::Vector2f> points;

    murosimproc::util::points::createRandomPointsRectangleUniform(points, SEED, CORNER_MAX, CORNER_MIN, COUNT);

    assertInsideRectangle(points);
}

TEST(UtilPointsUniform, SeedOverloadEqualsAFreshGenerator)
{
    std::vector<Eigen::Vector2f> by_seed;
    std::vector<Eigen::Vector2f> by_generator;
    RandomGenerator rng(SEED);

    murosimproc::util::points::createRandomPointsRectangleUniform(by_seed, SEED, CORNER_MIN, CORNER_MAX, COUNT);
    murosimproc::util::points::createRandomPointsRectangleUniform(by_generator, rng, CORNER_MIN, CORNER_MAX, COUNT);

    EXPECT_TRUE(samePoints(by_seed, by_generator));
}

TEST(UtilPointsUniform, DifferentSeedsDiffer)
{
    std::vector<Eigen::Vector2f> a;
    std::vector<Eigen::Vector2f> b;

    murosimproc::util::points::createRandomPointsRectangleUniform(a, SEED, CORNER_MIN, CORNER_MAX, COUNT);
    murosimproc::util::points::createRandomPointsRectangleUniform(b, SEED + 1, CORNER_MIN, CORNER_MAX, COUNT);

    EXPECT_FALSE(samePoints(a, b));
}

// ============================================================================
// Hexagon Points Tests
// ============================================================================

TEST(UtilPointsHexagon, LatticeWithoutNoiseKeepsTheSpacing)
{
    std::vector<Eigen::Vector2f> points;

    murosimproc::util::points::createRandomPointsRectangleHexagon(points, SEED, CORNER_MIN, CORNER_MAX, SPACING,
                                                                  NO_NOISE);

    // Rows sqrt(3)/2 apart, shifted by half a spacing: every neighbour is exactly one spacing away
    ASSERT_GT(points.size(), static_cast<size_t>(50));
    assertInsideRectangle(points);
    EXPECT_NEAR(minimumDistance(points), SPACING, 1e-4f);
    EXPECT_FLOAT_EQ(points.front().x(), CORNER_MIN.x()); // the lattice starts in the corner
    EXPECT_FLOAT_EQ(points.front().y(), CORNER_MIN.y());
}

TEST(UtilPointsHexagon, NoiseJittersButKeepsPointsInside)
{
    std::vector<Eigen::Vector2f> exact;
    std::vector<Eigen::Vector2f> jittered;

    murosimproc::util::points::createRandomPointsRectangleHexagon(exact, SEED, CORNER_MIN, CORNER_MAX, SPACING,
                                                                  NO_NOISE);
    murosimproc::util::points::createRandomPointsRectangleHexagon(jittered, SEED, CORNER_MIN, CORNER_MAX, SPACING,
                                                                  0.3f);

    assertInsideRectangle(jittered);
    EXPECT_FALSE(samePoints(exact, jittered));
    EXPECT_GT(minimumDistance(jittered), SPACING - 0.3f - EPSILON); // jitter of at most 0.15 per point
}

TEST(UtilPointsHexagon, SeedOverloadEqualsAFreshGenerator)
{
    std::vector<Eigen::Vector2f> by_seed;
    std::vector<Eigen::Vector2f> by_generator;
    RandomGenerator rng(SEED);

    murosimproc::util::points::createRandomPointsRectangleHexagon(by_seed, SEED, CORNER_MIN, CORNER_MAX, SPACING, 0.3f);
    murosimproc::util::points::createRandomPointsRectangleHexagon(by_generator, rng, CORNER_MIN, CORNER_MAX, SPACING,
                                                                  0.3f);

    EXPECT_TRUE(samePoints(by_seed, by_generator));
}

// ============================================================================
// Poisson Disk Points Tests
// ============================================================================

TEST(UtilPointsPoissonDisk, PointsRespectTheMinimumDistance)
{
    std::vector<Eigen::Vector2f> points;

    murosimproc::util::points::createRandomPointsRectanglePoissonDisk(points, SEED, CORNER_MIN, CORNER_MAX, SPACING,
                                                                      2.0f * SPACING);

    ASSERT_GT(points.size(), static_cast<size_t>(10));
    assertInsideRectangle(points);
    EXPECT_GE(minimumDistance(points), SPACING - EPSILON);
}

TEST(UtilPointsPoissonDisk, FillsTheRectangleDensely)
{
    std::vector<Eigen::Vector2f> points;

    murosimproc::util::points::createRandomPointsRectanglePoissonDisk(points, SEED, CORNER_MIN, CORNER_MAX, SPACING,
                                                                      SPACING);

    // 60 m^2 with a 1 m exclusion disc holds at most about 60 / (pi / 4) points
    // and a Bridson sampling reaches a good fraction of that
    EXPECT_GT(points.size(), static_cast<size_t>(30));
    EXPECT_LT(points.size(), static_cast<size_t>(80));
}

TEST(UtilPointsPoissonDisk, SeedOverloadEqualsAFreshGenerator)
{
    std::vector<Eigen::Vector2f> by_seed;
    std::vector<Eigen::Vector2f> by_generator;
    RandomGenerator rng(SEED);

    murosimproc::util::points::createRandomPointsRectanglePoissonDisk(by_seed, SEED, CORNER_MIN, CORNER_MAX, SPACING,
                                                                      2.0f * SPACING);
    murosimproc::util::points::createRandomPointsRectanglePoissonDisk(by_generator, rng, CORNER_MIN, CORNER_MAX,
                                                                      SPACING, 2.0f * SPACING);

    EXPECT_TRUE(samePoints(by_seed, by_generator));
}

TEST(UtilPointsPoissonDisk, FewerCandidatesPerSampleStillRespectTheDistance)
{
    std::vector<Eigen::Vector2f> points;

    murosimproc::util::points::createRandomPointsRectanglePoissonDisk(points, SEED, CORNER_MIN, CORNER_MAX, SPACING,
                                                                      SPACING, 3);

    ASSERT_FALSE(points.empty());
    EXPECT_GE(minimumDistance(points), SPACING - EPSILON);
}

// ============================================================================
// Choice Tests
// ============================================================================

TEST(UtilPointsChoice, TruncatesToASizeWithinTheRange)
{
    std::vector<Eigen::Vector2f> points;
    murosimproc::util::points::createRandomPointsRectangleUniform(points, SEED, CORNER_MIN, CORNER_MAX, COUNT);
    const std::vector<Eigen::Vector2f> original = points;

    murosimproc::util::points::choiceRandomPointsMinMax(points, SEED, 10, 20);

    EXPECT_GE(points.size(), static_cast<size_t>(10));
    EXPECT_LE(points.size(), static_cast<size_t>(20));
    for (size_t i = 0; i < points.size(); i++)
    {
        const bool known = std::any_of(original.begin(), original.end(), [&](const Eigen::Vector2f &p)
        {
            return p.x() == points[i].x() && p.y() == points[i].y();
        });
        EXPECT_TRUE(known) << "point " << i << " was not in the input";
    }
}

TEST(UtilPointsChoice, FixedRangeGivesExactlyThatMany)
{
    std::vector<Eigen::Vector2f> points;
    murosimproc::util::points::createRandomPointsRectangleUniform(points, SEED, CORNER_MIN, CORNER_MAX, COUNT);

    murosimproc::util::points::choiceRandomPointsMinMax(points, SEED, 7, 7);

    EXPECT_EQ(points.size(), static_cast<size_t>(7));
}

TEST(UtilPointsChoice, LeavesASmallEnoughInputUnchanged)
{
    std::vector<Eigen::Vector2f> points = {Eigen::Vector2f(1.0f, 1.0f), Eigen::Vector2f(2.0f, 2.0f)};
    const std::vector<Eigen::Vector2f> original = points;

    murosimproc::util::points::choiceRandomPointsMinMax(points, SEED, 5, 10);

    EXPECT_TRUE(samePoints(points, original)); // not even shuffled
}

TEST(UtilPointsChoice, SeedOverloadEqualsAFreshGenerator)
{
    std::vector<Eigen::Vector2f> by_seed;
    murosimproc::util::points::createRandomPointsRectangleUniform(by_seed, SEED, CORNER_MIN, CORNER_MAX, COUNT);
    std::vector<Eigen::Vector2f> by_generator = by_seed;
    RandomGenerator rng(SEED);

    murosimproc::util::points::choiceRandomPointsMinMax(by_seed, SEED, 10, 20);
    murosimproc::util::points::choiceRandomPointsMinMax(by_generator, rng, 10, 20);

    EXPECT_TRUE(samePoints(by_seed, by_generator));
}
