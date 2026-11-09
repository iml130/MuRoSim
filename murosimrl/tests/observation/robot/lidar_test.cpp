// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/lidar.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::robot::Robot;
using murosimrl::observation::robot::Lidar;
using fixtures::EPSILON;
using fixtures::LIDAR_RANGE_MAX;
using fixtures::LIDAR_RANGE_MIN;
using fixtures::LIDAR_RAYS;

constexpr float INF = std::numeric_limits<float>::infinity();

constexpr size_t LIDAR_INDEX = 0;
constexpr float NO_CLAMP = 0.0f;
constexpr float CLAMP = 2.0f;
constexpr bool RAW = false;
constexpr bool NORMALIZED = true;
constexpr size_t NO_STACK = 0;
constexpr size_t ONE_THREAD = 1;
constexpr size_t FOUR_THREADS = 4;

// Scan positions: 1 m from the +x wall so the distances vary per ray, and a
// second one further away for the stack test
const Eigen::Vector2f NEAR_WALL(4.0f, 0.0f);
const Eigen::Vector2f CENTRE(0.0f, 0.0f);
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class LidarObservationTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(NEAR_WALL.x(), NEAR_WALL.y(), 0.0f);
        step();
    }
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(LidarObservationTest, ShapeIsBatchByRayCount)
{
    const Lidar observation(LIDAR_INDEX, NO_CLAMP, RAW, NO_STACK, ONE_THREAD);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, LIDAR_RAYS}));
    EXPECT_EQ(Lidar(LIDAR_INDEX, NO_CLAMP, RAW, 3, ONE_THREAD).shape(robots()),
              (std::vector<size_t>{1, 3, LIDAR_RAYS}));
    EXPECT_TRUE(observation.shape(fixtures::RobotList()).empty());
}

TEST_F(LidarObservationTest, GettersReturnConfiguration)
{
    const Lidar observation(LIDAR_INDEX, CLAMP, NORMALIZED, NO_STACK, FOUR_THREADS);

    EXPECT_EQ(observation.name(), "lidar_0");
    EXPECT_EQ(observation.indexLidar(), LIDAR_INDEX);
    EXPECT_FLOAT_EQ(observation.rangeMax(), CLAMP);
    EXPECT_TRUE(observation.normalize());
    EXPECT_EQ(observation.numThreads(), FOUR_THREADS);
    EXPECT_FLOAT_EQ(Lidar(LIDAR_INDEX, -1.0f, RAW, NO_STACK).rangeMax(), 0.0f); // clamped to >= 0
}

// ============================================================================
// Value Tests
// ============================================================================

// Without clamping or normalization the observation is the published scan
// in ray order, which for the square world is the closed-form wall distance.
TEST_F(LidarObservationTest, RawValuesAreTheWallDistances)
{
    const Lidar observation(LIDAR_INDEX, NO_CLAMP, RAW, NO_STACK, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), LIDAR_RAYS);
    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        ASSERT_NEAR(output[i], fixtures::wallDistance(NEAR_WALL, fixtures::rayAngle(i)), EPSILON) << "ray " << i;
    }
}

// With no explicit range_max the observation normalizes by the robot's own
// sensor range (range_min -> 0, range_max -> 1), so different sensors
// normalize correctly per robot.
TEST_F(LidarObservationTest, NormalizesBySensorRange)
{
    const Lidar observation(LIDAR_INDEX, NO_CLAMP, NORMALIZED, NO_STACK, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        const float distance = std::clamp(fixtures::wallDistance(NEAR_WALL, fixtures::rayAngle(i)), LIDAR_RANGE_MIN, LIDAR_RANGE_MAX);
        const float expected = (distance - LIDAR_RANGE_MIN) / (LIDAR_RANGE_MAX - LIDAR_RANGE_MIN);
        ASSERT_NEAR(output[i], expected, EPSILON) << "ray " << i;
    }
}

// An explicit range_max clamps distances before (optional) normalization, so
// the observation is bounded independently of the sensor's real range.
TEST_F(LidarObservationTest, ExplicitRangeMaxClamps)
{
    const Lidar observation(LIDAR_INDEX, CLAMP, RAW, NO_STACK, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    // the rays away from the near wall exceed 2 m, so the clamp is exercised
    ASSERT_GT(fixtures::wallDistance(NEAR_WALL, PI), CLAMP);
    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        const float expected = std::min(fixtures::wallDistance(NEAR_WALL, fixtures::rayAngle(i)), CLAMP);
        ASSERT_NEAR(output[i], expected, EPSILON) << "ray " << i;
    }
}

TEST_F(LidarObservationTest, ExplicitRangeMaxClampsAndNormalizes)
{
    const Lidar observation(LIDAR_INDEX, CLAMP, NORMALIZED, NO_STACK, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        const float distance = std::clamp(fixtures::wallDistance(NEAR_WALL, fixtures::rayAngle(i)), LIDAR_RANGE_MIN, CLAMP);
        const float expected = (distance - LIDAR_RANGE_MIN) / (CLAMP - LIDAR_RANGE_MIN);
        ASSERT_NEAR(output[i], expected, EPSILON) << "ray " << i;
        ASSERT_LE(output[i], 1.0f) << "ray " << i;
    }
}

// Readings outside the sensor range are +inf (no return) and -inf (too
// close); every mode clamps them onto its limits, so the output stays finite
TEST_F(LidarObservationTest, InfiniteReadingsMapOntoTheLimits)
{
    murosim::sensor::Lidar &lidar = robot->getLidarAt(LIDAR_INDEX);
    lidar.getArrayLaserDistanceBuffer()[0] = INF;
    lidar.getArrayLaserDistanceBuffer()[1] = -INF;
    lidar.publishScan();
    robot->makeState();

    const std::vector<float> raw = Lidar(LIDAR_INDEX, NO_CLAMP, RAW, NO_STACK, ONE_THREAD)(robots());
    EXPECT_FLOAT_EQ(raw[0], LIDAR_RANGE_MAX);
    EXPECT_FLOAT_EQ(raw[1], LIDAR_RANGE_MIN);

    const std::vector<float> clamped = Lidar(LIDAR_INDEX, CLAMP, RAW, NO_STACK, ONE_THREAD)(robots());
    EXPECT_FLOAT_EQ(clamped[0], CLAMP);
    EXPECT_FLOAT_EQ(clamped[1], LIDAR_RANGE_MIN);

    const std::vector<float> normalized = Lidar(LIDAR_INDEX, NO_CLAMP, NORMALIZED, NO_STACK, ONE_THREAD)(robots());
    EXPECT_FLOAT_EQ(normalized[0], 1.0f);
    EXPECT_FLOAT_EQ(normalized[1], 0.0f);
}

// The clamp range [range_min, range_max] must be an interval
TEST_F(LidarObservationTest, RangeMaxNotAboveSensorRangeMinThrows)
{
    const Lidar observation(LIDAR_INDEX, 0.5f * LIDAR_RANGE_MIN, RAW, NO_STACK, ONE_THREAD);

    EXPECT_THROW(observation(robots()), std::invalid_argument);
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(LidarObservationTest, StackIsNewestFirst)
{
    robot->setPose(CENTRE.x(), CENTRE.y(), 0.0f);
    step();
    const Lidar observation(LIDAR_INDEX, NO_CLAMP, RAW, 2, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), 2 * LIDAR_RAYS);
    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        ASSERT_NEAR(output[i], fixtures::wallDistance(CENTRE, fixtures::rayAngle(i)), EPSILON) << "ray " << i;
        ASSERT_NEAR(output[LIDAR_RAYS + i], fixtures::wallDistance(NEAR_WALL, fixtures::rayAngle(i)), EPSILON)
            << "ray " << i;
    }
}

// ============================================================================
// Threading Tests
// ============================================================================

TEST_F(LidarObservationTest, MultipleThreadsMatchSingleThreadBitExact)
{
    fixtures::RobotList fleet;
    for (int i = 0; i < 5; i++)
    {
        const std::shared_ptr<Robot> member = fixtures::holonomicRobot("member", true);
        sim->addRobot(member);
        fleet.push_back(member);
    }
    sim->reset();
    for (size_t i = 0; i < fleet.size(); i++)
    {
        fleet[i]->setPose(-3.0f + static_cast<float>(i), 1.0f, 0.0f);
    }
    step();
    const Lidar single(LIDAR_INDEX, NO_CLAMP, NORMALIZED, 2, ONE_THREAD);
    const Lidar threaded(LIDAR_INDEX, NO_CLAMP, NORMALIZED, 2, FOUR_THREADS);

    const std::vector<float> expected = single(fleet);
    const std::vector<float> output = threaded(fleet);

    ASSERT_EQ(output.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
    {
        ASSERT_EQ(output[i], expected[i]) << "element " << i;
    }
}

// ============================================================================
// Contract Tests
// ============================================================================

TEST_F(LidarObservationTest, MissingLidarThrows)
{
    const std::shared_ptr<Robot> blind = fixtures::holonomicRobot("blind", false);
    blind->makeState();
    const Lidar observation(LIDAR_INDEX, NO_CLAMP, RAW, NO_STACK, ONE_THREAD);

    EXPECT_THROW(observation(fixtures::robotList(blind)), std::runtime_error);
    EXPECT_THROW(observation.shape(fixtures::robotList(blind)), std::runtime_error);
}

TEST_F(LidarObservationTest, LidarIndexOutOfRangeThrows)
{
    const Lidar observation(1, NO_CLAMP, RAW, NO_STACK, ONE_THREAD); // the robot has one lidar

    EXPECT_THROW(observation(robots()), std::runtime_error);
}
