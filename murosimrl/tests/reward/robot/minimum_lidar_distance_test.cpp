// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/robot/minimum_lidar_distance.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace
{
using murosim::robot::Robot;
using murosimrl::reward::robot::MinimumLidarDistance;
using fixtures::EPSILON;
using fixtures::LIDAR_RANGE_MIN;

constexpr float THRESHOLD = 2.0f;
constexpr float NEGATIVE = -1.0f;
constexpr float INF = std::numeric_limits<float>::infinity();

const Eigen::Vector2f NEAR_WALL(4.0f, 0.0f); // 1 m from the +x wall

/// The shortest fixture ray from NEAR_WALL: the ray closest to +x.
float minimumWallDistance()
{
    float minimum = fixtures::LIDAR_RANGE_MAX;
    for (size_t i = 0; i < fixtures::LIDAR_RAYS; i++)
    {
        minimum = std::min(minimum, fixtures::wallDistance(NEAR_WALL, fixtures::rayAngle(i)));
    }
    return minimum;
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// reward = clamp((threshold - min_distance) / (threshold - range_min), 0, 1)
// * negative, read from the lidars' current scan with min_distance clamped to
// the sensor range.
class MinimumLidarDistanceTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(NEAR_WALL.x(), NEAR_WALL.y(), 0.0f);
        step();
    }

    /// Publish `reading` on every ray of the robot's lidar.
    void publishReading(const float reading)
    {
        murosim::sensor::Lidar &lidar = robot->getLidarAt(0);
        murosim::simd::ArrayFloat &buffer = lidar.getArrayLaserDistanceBuffer();
        std::fill(buffer.data(), buffer.data() + buffer.size(), reading);
        lidar.publishScan();
    }

    MinimumLidarDistance reward{THRESHOLD, NEGATIVE};
};

// ============================================================================
// Penalty Tests
// ============================================================================

TEST_F(MinimumLidarDistanceTest, ShapeAndConfiguration)
{
    EXPECT_EQ(reward.shape(robots()), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(reward.name(), "minimum_lidar_distance");
    EXPECT_FLOAT_EQ(reward.thresholdDistance(), THRESHOLD);
    EXPECT_FLOAT_EQ(reward.negative(), NEGATIVE);
}

TEST_F(MinimumLidarDistanceTest, PenalisesProximityLinearly)
{
    const float minimum = minimumWallDistance();
    ASSERT_LT(minimum, THRESHOLD);

    EXPECT_NEAR(reward(robot)[0], NEGATIVE * (THRESHOLD - minimum) / (THRESHOLD - LIDAR_RANGE_MIN), EPSILON);
}

// range_min is the closest the lidar can measure, so it earns the full penalty
TEST_F(MinimumLidarDistanceTest, RangeMinEarnsTheFullPenalty)
{
    publishReading(LIDAR_RANGE_MIN);

    EXPECT_FLOAT_EQ(reward(robot)[0], NEGATIVE);
}

// -inf (too close, below range_min) clamps to range_min: the full penalty
TEST_F(MinimumLidarDistanceTest, TooCloseEarnsTheFullPenalty)
{
    publishReading(-INF);

    EXPECT_FLOAT_EQ(reward(robot)[0], NEGATIVE);
}

// +inf (no return) clamps to range_max, beyond any threshold below it
TEST_F(MinimumLidarDistanceTest, NoReturnEarnsNothing)
{
    publishReading(INF);

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

// No reading lies below a threshold at or under range_min (it would also
// divide by zero), so such a lidar does not contribute
TEST_F(MinimumLidarDistanceTest, ThresholdAtRangeMinEarnsNothing)
{
    publishReading(-INF);
    const MinimumLidarDistance at_range_min(LIDAR_RANGE_MIN, NEGATIVE);

    EXPECT_FLOAT_EQ(at_range_min(robot)[0], 0.0f);
}

TEST_F(MinimumLidarDistanceTest, BeyondTheThresholdEarnsNothing)
{
    const MinimumLidarDistance lenient(0.5f, NEGATIVE); // the nearest wall is 1 m away

    EXPECT_FLOAT_EQ(lenient(robot)[0], 0.0f);
}

TEST_F(MinimumLidarDistanceTest, FarFromEveryWallEarnsNothing)
{
    robot->setPose(0.0f, 0.0f, 0.0f); // 5 m from every wall
    step();

    EXPECT_FLOAT_EQ(reward(robot)[0], 0.0f);
}

// ============================================================================
// Disabled Tests
// ============================================================================

// A non-positive threshold would divide 0/0 into NaN: it reads as disabled.
TEST_F(MinimumLidarDistanceTest, NonPositiveThresholdDisablesReward)
{
    const MinimumLidarDistance disabled(0.0f, NEGATIVE);

    EXPECT_FLOAT_EQ(disabled(robot)[0], 0.0f);
}

TEST_F(MinimumLidarDistanceTest, RobotWithoutLidarsEarnsNothing)
{
    const std::shared_ptr<Robot> blind = fixtures::holonomicRobot("blind", false);

    EXPECT_FLOAT_EQ(reward(blind)[0], 0.0f);
}
