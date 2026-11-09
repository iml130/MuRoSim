// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/sensor/lidar.hpp"
#include "murosim/random/random_generator.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::random::RandomGenerator;
using murosim::sensor::Lidar;

constexpr float EPSILON = 1e-5f;

constexpr float FOV_FULL = 2.0f * PI;
constexpr float FOV_HALF = PI;
constexpr float FOV_QUARTER = PI_2;
constexpr size_t RESOLUTION = 360;
constexpr float RANGE_MIN = 0.1f;
constexpr float RANGE_MAX = 10.0f;
constexpr float FREQUENCY = 10.0f; // Hz, scan period 0.1 s
constexpr float SCAN_PERIOD = 1.0f / FREQUENCY;
constexpr float NOISE_UNIFORM = 0.5f;
constexpr float NOISE_NORMAL = 0.1f;
constexpr uint64_t SEED = 42;
constexpr size_t SENTINEL = 99; // index window outputs start here so an untouched output is visible
constexpr float INF = std::numeric_limits<float>::infinity();
constexpr float HIT = 5.0f; // a nearest hit well inside [RANGE_MIN, RANGE_MAX]

const Eigen::Affine2f IDENTITY = Eigen::Affine2f::Identity();

/// Write `value` into every ray of the work buffer, as the laser kernels would
/// leave it after finding a hit at that distance on every ray.
void fillBuffer(Lidar &lidar, const float value)
{
    murosim::simd::ArrayFloat &buffer = lidar.getArrayLaserDistanceBuffer();
    std::fill(buffer.data(), buffer.data() + buffer.size(), value);
}

/// Angle between neighbouring rays of a partial FOV (both edges measured),
/// computed in double exactly as the lidar lays out its rays.
float angleStep(const float fov, const size_t resolution)
{
    return static_cast<float>(static_cast<double>(fov) / static_cast<double>(resolution - 1));
}

/// Angle between neighbouring rays of a full circle (no ray measured twice),
/// computed in double exactly as the lidar lays out its rays.
float fullCircleStep(const size_t resolution)
{
    return static_cast<float>(2.0 * static_cast<double>(PI) / static_cast<double>(resolution));
}

/// Unit vector of `angle`, computed in double exactly as the lidar does.
Eigen::Vector2f unitVector(const double angle)
{
    return (Eigen::Rotation2Dd(angle) * Eigen::Vector2d(1.0, 0.0)).cast<float>();
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

// A default-constructed Lidar (resolution 0) used to throw at runtime; the
// default constructor is deleted now, so the misuse fails at compile time.

TEST(LidarConstructor, BasicConstructor)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX);

    EXPECT_EQ(lidar.getLaserSize(), static_cast<size_t>(180));
    EXPECT_FLOAT_EQ(lidar.getRangeMin(), RANGE_MIN);
    EXPECT_FLOAT_EQ(lidar.getRangeMax(), RANGE_MAX);
    EXPECT_FLOAT_EQ(lidar.getNoise(), 0.0f);
    EXPECT_EQ(lidar.getNoiseType(), Lidar::NOISE_TYPE::NONE);
    EXPECT_FALSE(lidar.getRotationClockwise());
    EXPECT_LE(lidar.getFrequency(), 0.0f); // no frequency: partial sweeps disabled
    EXPECT_FLOAT_EQ(lidar.getAngleStart(), PI);
}

TEST(LidarConstructor, ConstructorWithFrequency)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY);

    EXPECT_EQ(lidar.getLaserSize(), static_cast<size_t>(180));
    EXPECT_FLOAT_EQ(lidar.getFrequency(), FREQUENCY);
    EXPECT_EQ(lidar.getNoiseType(), Lidar::NOISE_TYPE::NONE);
}

TEST(LidarConstructor, ConstructorWithUniformNoise)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::UNIFORM,
                      NOISE_UNIFORM);

    EXPECT_EQ(lidar.getNoiseType(), Lidar::NOISE_TYPE::UNIFORM);
    EXPECT_FLOAT_EQ(lidar.getNoise(), NOISE_UNIFORM);
}

TEST(LidarConstructor, ConstructorWithNormalNoise)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NORMAL,
                      NOISE_NORMAL);

    EXPECT_EQ(lidar.getNoiseType(), Lidar::NOISE_TYPE::NORMAL);
    EXPECT_FLOAT_EQ(lidar.getNoise(), NOISE_NORMAL);
}

TEST(LidarConstructor, ConstructorWithAngleStartAndRotationClockwise)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NONE, 0.0f, 0.0f,
                      true);

    EXPECT_FLOAT_EQ(lidar.getAngleStart(), 0.0f);
    EXPECT_TRUE(lidar.getRotationClockwise());
}

TEST(LidarConstructor, ConstructorStoresPoseTranslation)
{
    Eigen::Affine2f pose = Eigen::Affine2f::Identity();
    pose.translation() = Eigen::Vector2f(3.0f, 4.0f);

    const Lidar lidar(pose, FOV_HALF, 180, RANGE_MIN, RANGE_MAX);

    EXPECT_FLOAT_EQ(lidar.getPose().translation().x(), 3.0f);
    EXPECT_FLOAT_EQ(lidar.getPose().translation().y(), 4.0f);
}

TEST(LidarConstructor, ConstructorStoresPoseRotation)
{
    Eigen::Affine2f pose = Eigen::Affine2f::Identity();
    pose.rotate(Eigen::Rotation2Df(PI_2));

    const Lidar lidar(pose, FOV_HALF, 180, RANGE_MIN, RANGE_MAX);

    EXPECT_NEAR(Eigen::Rotation2Df(lidar.getPose().rotation()).angle(), PI_2, EPSILON);
}

TEST(LidarConstructor, SingleRayPointsForward)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 1, RANGE_MIN, RANGE_MAX);

    EXPECT_EQ(lidar.getLaserSize(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(lidar.getAngleMin(), 0.0f);
    EXPECT_FLOAT_EQ(lidar.getAngleMax(), 0.0f);
    EXPECT_FLOAT_EQ(lidar.getAngleStep(), 0.0f);
    EXPECT_FLOAT_EQ(lidar.getLaserX(0), 1.0f);
    EXPECT_FLOAT_EQ(lidar.getLaserY(0), 0.0f);
}

TEST(LidarConstructor, ZeroResolutionThrows)
{
    EXPECT_THROW(Lidar(IDENTITY, FOV_HALF, 0, RANGE_MIN, RANGE_MAX), std::invalid_argument);
}

TEST(LidarConstructor, RangeMustBeAProperInterval)
{
    // clampDistance() and the normalizing consumers need 0 <= range_min < range_max
    EXPECT_THROW(Lidar(IDENTITY, FOV_HALF, 180, -0.1f, RANGE_MAX), std::invalid_argument);
    EXPECT_THROW(Lidar(IDENTITY, FOV_HALF, 180, RANGE_MAX, RANGE_MAX), std::invalid_argument);
    EXPECT_THROW(Lidar(IDENTITY, FOV_HALF, 180, RANGE_MAX, RANGE_MIN), std::invalid_argument);
    EXPECT_THROW(Lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, INF), std::invalid_argument);
    EXPECT_THROW(Lidar(IDENTITY, FOV_HALF, 180, std::numeric_limits<float>::quiet_NaN(), RANGE_MAX), std::invalid_argument);
    EXPECT_NO_THROW(Lidar(IDENTITY, FOV_HALF, 180, 0.0f, RANGE_MAX));
}

TEST(LidarConstructor, FullFovSpreadsRaysEvenlyAroundTheCircle)
{
    const Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX);

    EXPECT_NEAR(lidar.getAngleMin(), -PI, EPSILON);
    EXPECT_NEAR(lidar.getAngleMax(), PI - fullCircleStep(RESOLUTION), EPSILON);
    EXPECT_FLOAT_EQ(lidar.getAngleStep(), fullCircleStep(RESOLUTION));
}

// Regression: a full circle used to place RESOLUTION rays fov / (RESOLUTION - 1)
// apart, so the first and the last ray both pointed at -pi/+pi and one
// direction was measured twice. The last ray now sits one step before the
// first, like every other neighbour.
TEST(LidarConstructor, FullFovMeasuresNoDirectionTwice)
{
    const Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX);

    const float cos_last_to_first = lidar.getLaserDirection(0).dot(lidar.getLaserDirection(RESOLUTION - 1));
    EXPECT_NEAR(std::acos(cos_last_to_first), fullCircleStep(RESOLUTION), 1e-3f);
}

// A FOV within half a full-circle step of 2*pi is laid out as a full circle
TEST(LidarConstructor, NearlyFullFovIsAFullCircle)
{
    const float nearly_full = FOV_FULL - 0.4f * fullCircleStep(RESOLUTION);
    const Lidar lidar(IDENTITY, nearly_full, RESOLUTION, RANGE_MIN, RANGE_MAX);

    EXPECT_FLOAT_EQ(lidar.getAngleStep(), fullCircleStep(RESOLUTION));
    EXPECT_NEAR(lidar.getAngleMin(), -PI, EPSILON);
}

TEST(LidarConstructor, HalfFovIsCentredOnForward)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX);

    EXPECT_NEAR(lidar.getAngleMin(), -PI_2, EPSILON);
    EXPECT_NEAR(lidar.getAngleMax(), PI_2, EPSILON);
}

TEST(LidarConstructor, QuarterFovIsCentredOnForward)
{
    const Lidar lidar(IDENTITY, FOV_QUARTER, 90, RANGE_MIN, RANGE_MAX);

    EXPECT_NEAR(lidar.getAngleMin(), -PI_4, EPSILON);
    EXPECT_NEAR(lidar.getAngleMax(), PI_4, EPSILON);
}

TEST(LidarConstructor, DistancesStartAtInf)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX);

    // Both the published scan and the work buffer report "no return"
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_EQ(lidar.getLaserDistance(i), INF) << "ray " << i;
        ASSERT_EQ(lidar.getArrayLaserDistanceBuffer()[i], INF) << "ray " << i;
    }
}

// ============================================================================
// Scan Order Tests
// ============================================================================

// Rays are stored in the order the beam measures them, so index i is the i-th
// ray of a rotation and its angle is angle_min + i * angle_step.

// Regression: rotation_clockwise_ used to be stored but ignored, the rays
// were always generated counter-clockwise.
TEST(LidarScanOrder, ClockwiseStoresRaysInReversedOrder)
{
    const Lidar ccw(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NONE, 0.0f,
                    PI, false);
    const Lidar cw(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NONE, 0.0f,
                   PI, true);

    // Both scans start at the seam and run around the circle in opposite
    // directions: clockwise ray i points where counter-clockwise ray -i does
    for (size_t i = 0; i < RESOLUTION; i++)
    {
        const size_t mirrored = (RESOLUTION - i) % RESOLUTION;
        ASSERT_NEAR(cw.getLaserX(i), ccw.getLaserX(mirrored), EPSILON) << "ray " << i;
        ASSERT_NEAR(cw.getLaserY(i), ccw.getLaserY(mirrored), EPSILON) << "ray " << i;
    }
}

TEST(LidarScanOrder, ClockwiseAngleMetadataIsReversed)
{
    const float fov = 1.5f * PI; // 270 degrees
    const Lidar lidar(IDENTITY, fov, 270, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NONE, 0.0f, PI,
                      true);

    // angle_min is the FIRST ray's angle, angle_step is negative
    EXPECT_NEAR(lidar.getAngleMin(), fov / 2.0f, EPSILON);
    EXPECT_NEAR(lidar.getAngleMax(), -fov / 2.0f, EPSILON);
    EXPECT_FLOAT_EQ(lidar.getAngleStep(), -angleStep(fov, 270));

    // angle_min + i * angle_step reconstructs the ray directions
    for (size_t i = 0; i < 270; i += 37)
    {
        const float angle = lidar.getAngleMin() + static_cast<float>(i) * lidar.getAngleStep();
        EXPECT_NEAR(lidar.getLaserX(i), std::cos(angle), EPSILON) << "ray " << i;
        EXPECT_NEAR(lidar.getLaserY(i), std::sin(angle), EPSILON) << "ray " << i;
    }
}

TEST(LidarScanOrder, AngleStartPlacesSeamOfFullCircleScan)
{
    // The first measured ray of a full-circle scan points at angle_start
    const Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NONE, 0.0f,
                      PI_2, false);

    EXPECT_NEAR(lidar.getLaserX(0), 0.0f, EPSILON); // +90 degrees
    EXPECT_NEAR(lidar.getLaserY(0), 1.0f, EPSILON);
    // angle_min is the default -pi shifted by the seam offset
    // (angle_start - pi), not normalised: -pi + (pi/2 - pi)
    EXPECT_NEAR(lidar.getAngleMin(), -PI + (PI_2 - PI), EPSILON);
}

TEST(LidarScanOrder, AngleStartIgnoredForPartialFov)
{
    // A partial FOV always starts the scan at a FOV edge
    const Lidar lidar(IDENTITY, FOV_QUARTER, 90, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NONE, 0.0f, 1.0f,
                      false);

    EXPECT_NEAR(lidar.getAngleMin(), -PI_4, EPSILON);
    EXPECT_NEAR(lidar.getAngleMax(), PI_4, EPSILON);
    EXPECT_NEAR(lidar.getLaserX(0), std::cos(-PI_4), EPSILON);
    EXPECT_NEAR(lidar.getLaserY(0), std::sin(-PI_4), EPSILON);
}

// ============================================================================
// Test Fixture
// ============================================================================

class LidarTest : public ::testing::Test
{
protected:
    /// Full-circle lidar: RESOLUTION rays spread evenly from -pi, counter-clockwise.
    Lidar lidar{IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX};
    RandomGenerator rng{SEED};
};

// ============================================================================
// Getter Tests
// ============================================================================

TEST_F(LidarTest, GetAngleAccessors)
{
    EXPECT_FLOAT_EQ(lidar.getAngleMin(), -PI);
    EXPECT_NEAR(lidar.getAngleMax(), PI - fullCircleStep(RESOLUTION), EPSILON);
    EXPECT_FLOAT_EQ(lidar.getAngleStart(), PI);
    EXPECT_FLOAT_EQ(lidar.getAngleStep(), fullCircleStep(RESOLUTION));
}

TEST_F(LidarTest, GetRangeAndNoiseAccessors)
{
    EXPECT_FLOAT_EQ(lidar.getRangeMin(), RANGE_MIN);
    EXPECT_FLOAT_EQ(lidar.getRangeMax(), RANGE_MAX);
    EXPECT_FLOAT_EQ(lidar.getNoise(), 0.0f);
    EXPECT_EQ(lidar.getNoiseType(), Lidar::NOISE_TYPE::NONE);
    EXPECT_FALSE(lidar.getRotationClockwise());
}

TEST_F(LidarTest, GetPoseIsIdentity)
{
    EXPECT_TRUE(lidar.getPose().isApprox(Eigen::Affine2f::Identity()));
}

TEST_F(LidarTest, GetLaserArraySizes)
{
    EXPECT_EQ(lidar.getLaserSize(), RESOLUTION);
    EXPECT_EQ(lidar.getArrayLaserDistance().size(), RESOLUTION);
    EXPECT_EQ(lidar.getArrayLaserDistanceBuffer().size(), RESOLUTION);
    EXPECT_EQ(lidar.getArrayLaserX().size(), RESOLUTION);
    EXPECT_EQ(lidar.getArrayLaserY().size(), RESOLUTION);
}

TEST_F(LidarTest, RayAccessorsAgreeWithLayout)
{
    const double angle_step = 2.0 * static_cast<double>(PI) / static_cast<double>(RESOLUTION);
    const double angle_first = -static_cast<double>(PI);

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        const Eigen::Vector2f direction = unitVector(angle_first + angle_step * static_cast<double>(i));

        ASSERT_FLOAT_EQ(lidar.getArrayLaserX()[i], direction.x()) << "ray " << i;
        ASSERT_FLOAT_EQ(lidar.getArrayLaserY()[i], direction.y()) << "ray " << i;
        ASSERT_FLOAT_EQ(lidar.getLaserX(i), direction.x()) << "ray " << i;
        ASSERT_FLOAT_EQ(lidar.getLaserY(i), direction.y()) << "ray " << i;
        ASSERT_FLOAT_EQ(lidar.getLaserDirection(i).x(), direction.x()) << "ray " << i;
        ASSERT_FLOAT_EQ(lidar.getLaserDirection(i).y(), direction.y()) << "ray " << i;
        // a fresh lidar has no return (+inf); its points end at range_max
        ASSERT_FLOAT_EQ(lidar.getLaserPoint(i).x(), direction.x() * RANGE_MAX) << "ray " << i;
        ASSERT_FLOAT_EQ(lidar.getLaserPoint(i).y(), direction.y() * RANGE_MAX) << "ray " << i;
        ASSERT_EQ(lidar.getLaserDistance(i), INF) << "ray " << i;
    }
}

TEST_F(LidarTest, GetLaserPointScalesDirectionByDistance)
{
    lidar.getArrayLaserDistanceBuffer()[0] = 2.5f;
    lidar.publishScan();

    EXPECT_FLOAT_EQ(lidar.getLaserPoint(0).x(), 2.5f * lidar.getLaserX(0));
    EXPECT_FLOAT_EQ(lidar.getLaserPoint(0).y(), 2.5f * lidar.getLaserY(0));
}

TEST_F(LidarTest, GetLaserPointClampsInfiniteReadings)
{
    lidar.getArrayLaserDistanceBuffer()[0] = INF;  // no return
    lidar.getArrayLaserDistanceBuffer()[1] = -INF; // too close
    lidar.publishScan();

    EXPECT_FLOAT_EQ(lidar.getLaserPoint(0).x(), RANGE_MAX * lidar.getLaserX(0));
    EXPECT_FLOAT_EQ(lidar.getLaserPoint(0).y(), RANGE_MAX * lidar.getLaserY(0));
    EXPECT_FLOAT_EQ(lidar.getLaserPoint(1).x(), RANGE_MIN * lidar.getLaserX(1));
    EXPECT_FLOAT_EQ(lidar.getLaserPoint(1).y(), RANGE_MIN * lidar.getLaserY(1));
}

TEST_F(LidarTest, ClampDistanceMapsReadingsOntoRange)
{
    EXPECT_FLOAT_EQ(lidar.clampDistance(INF), RANGE_MAX);
    EXPECT_FLOAT_EQ(lidar.clampDistance(-INF), RANGE_MIN);
    EXPECT_FLOAT_EQ(lidar.clampDistance(HIT), HIT);
    EXPECT_FLOAT_EQ(lidar.clampDistance(2.0f * RANGE_MAX), RANGE_MAX);
    EXPECT_FLOAT_EQ(lidar.clampDistance(0.0f), RANGE_MIN);
}

// ============================================================================
// Scan Buffer Tests
// ============================================================================

// The collision sweep writes into the work buffer; consumers read the
// published scan, which only changes on publishScan().

TEST_F(LidarTest, BufferWritesAreInvisibleUntilPublished)
{
    lidar.getArrayLaserDistanceBuffer()[0] = 2.5f;
    lidar.getArrayLaserDistanceBuffer()[50] = 3.7f;

    EXPECT_EQ(lidar.getLaserDistance(0), INF);
    EXPECT_EQ(lidar.getLaserDistance(50), INF);
}

TEST_F(LidarTest, PublishScanCopiesBufferIntoScan)
{
    lidar.getArrayLaserDistanceBuffer()[0] = 2.5f;
    lidar.getArrayLaserDistanceBuffer()[50] = 3.7f;

    lidar.publishScan();

    EXPECT_FLOAT_EQ(lidar.getLaserDistance(0), 2.5f);
    EXPECT_FLOAT_EQ(lidar.getLaserDistance(50), 3.7f);
    EXPECT_EQ(lidar.getLaserDistance(1), INF); // untouched ray
}

TEST_F(LidarTest, FillLaserDistanceWithInfResetsWholeBuffer)
{
    fillBuffer(lidar, 0.0f);

    lidar.fillLaserDistanceWithInf();

    const murosim::simd::ArrayFloat &buffer = lidar.getArrayLaserDistanceBuffer();
    for (size_t i = 0; i < buffer.size(); i++)
    {
        ASSERT_EQ(buffer[i], INF) << "ray " << i;
    }
}

TEST_F(LidarTest, FillLaserDistanceWithInfWindowLeavesOtherRays)
{
    fillBuffer(lidar, 0.0f);

    lidar.fillLaserDistanceWithInf(10, 20); // half-open window

    const murosim::simd::ArrayFloat &buffer = lidar.getArrayLaserDistanceBuffer();
    for (size_t i = 0; i < buffer.size(); i++)
    {
        const float expected = (10 <= i && i < 20) ? INF : 0.0f;
        ASSERT_EQ(buffer[i], expected) << "ray " << i;
    }
}

// ============================================================================
// Noise Tests
// ============================================================================

// applyNoise turns the nearest hits the kernels left in the work buffer into
// readings (REP 117): out-of-range hits become +inf/-inf, the rest get noise.

TEST_F(LidarTest, ApplyNoiseNoneKeepsHitsInRange)
{
    fillBuffer(lidar, HIT);

    lidar.applyNoise(rng);
    lidar.publishScan();

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_EQ(lidar.getLaserDistance(i), HIT) << "ray " << i;
    }
}

TEST_F(LidarTest, ApplyNoiseMapsOutOfRangeHitsToInf)
{
    // nearest hit -> reading, without noise (NOISE_TYPE::NONE)
    const std::vector<std::pair<float, float>> cases = {
        {INF, INF},                  // no hit at all: no return
        {2.0f * RANGE_MAX, INF},     // hit beyond range_max: no return
        {RANGE_MAX, RANGE_MAX},      // range_max itself is still measured
        {HIT, HIT},
        {RANGE_MIN, RANGE_MIN},      // range_min itself is still measured
        {0.5f * RANGE_MIN, -INF},    // closer than range_min: too close
        {0.0f, -INF},                // contact (lidar inside a circle)
    };

    for (size_t i = 0; i < cases.size(); i++)
    {
        lidar.getArrayLaserDistanceBuffer()[i] = cases[i].first;
    }

    lidar.applyNoise(rng, 0, cases.size());
    lidar.publishScan();

    for (size_t i = 0; i < cases.size(); i++)
    {
        EXPECT_EQ(lidar.getLaserDistance(i), cases[i].second) << "nearest hit " << cases[i].first;
    }
}

TEST(LidarNoise, UniformNoiseStaysWithinBounds)
{
    Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::UNIFORM, NOISE_UNIFORM);
    RandomGenerator rng(SEED);
    fillBuffer(lidar, HIT);

    lidar.applyNoise(rng);
    lidar.publishScan();

    // Every ray is disturbed (the chance of drawing exactly 0 is negligible
    // and the seed is fixed) and stays within [-noise, noise] of the hit
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        const float offset = lidar.getLaserDistance(i) - HIT;
        ASSERT_NE(offset, 0.0f) << "ray " << i;
        ASSERT_LE(std::abs(offset), NOISE_UNIFORM) << "ray " << i;
    }
}

TEST(LidarNoise, NormalNoiseDisturbsEveryRay)
{
    Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NORMAL,
                NOISE_NORMAL);
    RandomGenerator rng(SEED);
    fillBuffer(lidar, HIT);

    lidar.applyNoise(rng);
    lidar.publishScan();

    // Gaussian noise is unbounded, but with a fixed seed no draw of 360 is
    // beyond 5 standard deviations
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        const float offset = lidar.getLaserDistance(i) - HIT;
        ASSERT_NE(offset, 0.0f) << "ray " << i;
        ASSERT_LT(std::abs(offset), 5.0f * NOISE_NORMAL) << "ray " << i;
    }
}

TEST(LidarNoise, NoiseKeepsRaysWithoutReturnAtInf)
{
    // A fresh lidar has no hit on any ray: noise must not turn +inf into a reading
    Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NORMAL, NOISE_NORMAL);
    RandomGenerator rng(SEED);

    lidar.applyNoise(rng);
    lidar.publishScan();

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_EQ(lidar.getLaserDistance(i), INF) << "ray " << i;
    }
}

TEST(LidarNoise, NoiseAcrossTheRangeLimitsGivesInf)
{
    // Hits just inside the limits: noise pushing them out of the range turns
    // them into +inf (beyond range_max) or -inf (below range_min)
    for (const auto &[hit, outside] : {std::pair{RANGE_MAX - 0.01f, INF}, std::pair{RANGE_MIN + 0.01f, -INF}})
    {
        Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::UNIFORM, NOISE_UNIFORM);
        RandomGenerator rng(SEED);
        fillBuffer(lidar, hit);

        lidar.applyNoise(rng);
        lidar.publishScan();

        size_t pushed_out = 0;
        for (size_t i = 0; i < lidar.getLaserSize(); i++)
        {
            const float reading = lidar.getLaserDistance(i);
            if (reading == outside)
            {
                pushed_out++;
            }
            else
            {
                ASSERT_GE(reading, RANGE_MIN) << "ray " << i;
                ASSERT_LE(reading, RANGE_MAX) << "ray " << i;
                ASSERT_LE(std::abs(reading - hit), NOISE_UNIFORM) << "ray " << i;
            }
        }
        // about half the uniform draws exceed the 0.01 m margin
        EXPECT_GT(pushed_out, lidar.getLaserSize() / 4) << "hit " << hit;
        EXPECT_LT(pushed_out, lidar.getLaserSize()) << "hit " << hit;
    }
}

TEST(LidarNoise, NoiseDrawsDoNotDependOnTheScene)
{
    // Noise is drawn for every ray, hit or not, so a shared generator advances
    // the same way whatever the lidar sees
    Lidar empty(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NORMAL, NOISE_NORMAL);
    Lidar walled(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NORMAL, NOISE_NORMAL);
    fillBuffer(walled, HIT);
    RandomGenerator rng_empty(SEED);
    RandomGenerator rng_walled(SEED);

    empty.applyNoise(rng_empty);
    walled.applyNoise(rng_walled);

    EXPECT_EQ(rng_empty(), rng_walled());
}

TEST(LidarNoise, ApplyNoiseWritesWorkBufferOnly)
{
    Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::UNIFORM, NOISE_UNIFORM);
    RandomGenerator rng(SEED);
    fillBuffer(lidar, HIT);

    lidar.applyNoise(rng);

    EXPECT_NE(lidar.getArrayLaserDistanceBuffer()[0], HIT);
    EXPECT_EQ(lidar.getLaserDistance(0), INF); // not published yet
}

TEST(LidarNoise, ApplyNoiseWindowLeavesOtherRays)
{
    Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::UNIFORM, NOISE_UNIFORM);
    RandomGenerator rng(SEED);
    fillBuffer(lidar, HIT);

    lidar.applyNoise(rng, 10, 20); // half-open window
    lidar.publishScan();

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        const bool inside = 10 <= i && i < 20;
        ASSERT_EQ(lidar.getLaserDistance(i) != HIT, inside) << "ray " << i;
    }
}

TEST(LidarNoise, SameSeedGivesSameNoise)
{
    Lidar first(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NORMAL, NOISE_NORMAL);
    Lidar second(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY, Lidar::NOISE_TYPE::NORMAL, NOISE_NORMAL);
    RandomGenerator rng_first(SEED);
    RandomGenerator rng_second(SEED);
    fillBuffer(first, HIT);
    fillBuffer(second, HIT);

    first.applyNoise(rng_first);
    second.applyNoise(rng_second);
    first.publishScan();
    second.publishScan();

    for (size_t i = 0; i < first.getLaserSize(); i++)
    {
        ASSERT_FLOAT_EQ(first.getLaserDistance(i), second.getLaserDistance(i)) << "ray " << i;
    }
}

// ============================================================================
// Scan Time Tests
// ============================================================================

TEST(LidarScanTime, StartsAtZero)
{
    const Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY);

    EXPECT_FLOAT_EQ(lidar.getScanTime(), 0.0f);
}

TEST(LidarScanTime, AdvanceWrapsWithinOnePeriod)
{
    Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY);

    lidar.advanceScanTime(0.04f);
    EXPECT_NEAR(lidar.getScanTime(), 0.04f, EPSILON);

    lidar.advanceScanTime(0.08f); // 0.12 s wraps past the 0.1 s period
    EXPECT_NEAR(lidar.getScanTime(), 0.12f - SCAN_PERIOD, EPSILON);
}

TEST(LidarScanTime, AdvanceIsNoOpWithoutFrequency)
{
    Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX);

    lidar.advanceScanTime(0.04f);

    EXPECT_FLOAT_EQ(lidar.getScanTime(), 0.0f);
}

TEST(LidarScanTime, ResetRestartsTheScan)
{
    Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY);
    lidar.advanceScanTime(0.03f);
    fillBuffer(lidar, HIT);
    lidar.publishScan();

    lidar.reset();

    // back to the state of a fresh lidar: phase 0, no return anywhere
    EXPECT_FLOAT_EQ(lidar.getScanTime(), 0.0f);
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_EQ(lidar.getLaserDistance(i), INF) << "ray " << i;
        ASSERT_EQ(lidar.getArrayLaserDistanceBuffer()[i], INF) << "ray " << i;
    }
}

// ============================================================================
// Index Window Tests
// ============================================================================

// indexWindowForTimeInterval maps a time interval onto the half-open ray
// window [begin, end) the beam measures during it. These pin the exact
// arithmetic the partial sweep relies on: 360 rays at 10 Hz is a 0.1 s
// period, 3600 rays/s for a full circle.

TEST(LidarIndexWindow, FullCircleMapsTimeLinearly)
{
    const Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY);
    size_t begin = SENTINEL;
    size_t end = SENTINEL;

    const bool crossed_seam = lidar.indexWindowForTimeInterval(0.0f, 0.025f, begin, end);

    EXPECT_FALSE(crossed_seam);
    EXPECT_EQ(begin, static_cast<size_t>(0));
    EXPECT_EQ(end, static_cast<size_t>(90)); // quarter rotation
}

TEST(LidarIndexWindow, IntervalOfAtLeastOnePeriodSweepsEveryRay)
{
    const Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX, FREQUENCY);
    size_t begin = SENTINEL;
    size_t end = SENTINEL;

    const bool crossed_seam = lidar.indexWindowForTimeInterval(0.0f, SCAN_PERIOD, begin, end);

    EXPECT_FALSE(crossed_seam);
    EXPECT_EQ(begin, static_cast<size_t>(0));
    EXPECT_EQ(end, RESOLUTION);
}

TEST(LidarIndexWindow, DeadZoneIntervalMapsToEmptyWindow)
{
    // Half circle: only the first 0.05 s of each 0.1 s period measures
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY);
    size_t begin = SENTINEL;
    size_t end = SENTINEL;

    const bool crossed_seam = lidar.indexWindowForTimeInterval(0.06f, 0.09f, begin, end); // dead time only

    EXPECT_FALSE(crossed_seam);
    EXPECT_EQ(begin, end); // empty window
}

TEST(LidarIndexWindow, IntervalAcrossDeadZoneWrapsIntoNextScan)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY);
    size_t begin = SENTINEL;
    size_t end = SENTINEL;

    // Tail of this FOV [144, 180), the dead zone, then the next scan's start.
    // The wrapped end is chosen away from an integer ray boundary (~50.4 rays)
    // so the float quantisation is unambiguous.
    const bool crossed_seam = lidar.indexWindowForTimeInterval(0.04f, 0.114f, begin, end);

    EXPECT_FALSE(crossed_seam);
    EXPECT_EQ(begin, static_cast<size_t>(144));
    EXPECT_EQ(end, static_cast<size_t>(50)); // begin > end: wrapped window
}

TEST(LidarIndexWindow, IntervalStartingInDeadZoneReportsSeamCrossing)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 180, RANGE_MIN, RANGE_MAX, FREQUENCY);
    size_t begin = SENTINEL;
    size_t end = SENTINEL;

    // Begins in the dead zone (0.06 s), ends 0.014 s into the next rotation
    const bool crossed_seam = lidar.indexWindowForTimeInterval(0.06f, 0.114f, begin, end);

    EXPECT_TRUE(crossed_seam); // caller must publish before sweeping
    EXPECT_EQ(begin, static_cast<size_t>(0));
    EXPECT_EQ(end, static_cast<size_t>(50));
}

TEST(LidarIndexWindow, ThrowsWithoutFrequency)
{
    const Lidar lidar(IDENTITY, FOV_FULL, RESOLUTION, RANGE_MIN, RANGE_MAX);
    size_t begin = SENTINEL;
    size_t end = SENTINEL;

    EXPECT_THROW(lidar.indexWindowForTimeInterval(0.0f, 0.025f, begin, end), std::runtime_error);
}

// ============================================================================
// Stream Operator Tests
// ============================================================================

TEST(LidarStream, PrintsOneRayPerLine)
{
    const Lidar lidar(IDENTITY, FOV_HALF, 1, RANGE_MIN, RANGE_MAX); // single ray along +x

    std::ostringstream stream;
    stream << lidar;

    // a fresh lidar has no return yet
    EXPECT_EQ(stream.str(), "(1,\t0):\tinf\n");
}
