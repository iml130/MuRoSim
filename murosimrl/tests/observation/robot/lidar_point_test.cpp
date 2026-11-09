// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/lidar_point.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::math::PI_2;
using murosim::robot::Robot;
using murosimrl::observation::robot::LidarPoint;
using fixtures::EPSILON;
using fixtures::LIDAR_RAYS;

constexpr size_t LIDAR_INDEX = 0;
constexpr bool LOCAL = true;
constexpr bool WORLD = false;
constexpr size_t NO_STACK = 0;
constexpr size_t ONE_THREAD = 1;
constexpr size_t FOUR_THREADS = 4;

const Eigen::Vector2f NEAR_WALL(4.0f, 0.0f);
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class LidarPointTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(NEAR_WALL.x(), NEAR_WALL.y(), PI_2); // facing +y
        step();
    }

    /// Distance of ray `i` for the fixture pose: the rays turn with the robot.
    static float rayDistance(const size_t i)
    {
        return fixtures::wallDistance(NEAR_WALL, PI_2 + fixtures::rayAngle(i));
    }
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(LidarPointTest, ShapeIsBatchByTwoByRayCount)
{
    const LidarPoint observation(LIDAR_INDEX, LOCAL, NO_STACK, ONE_THREAD);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 2, LIDAR_RAYS}));
    EXPECT_EQ(LidarPoint(LIDAR_INDEX, LOCAL, 2, ONE_THREAD).shape(robots()),
              (std::vector<size_t>{1, 2, 2, LIDAR_RAYS}));
    EXPECT_TRUE(observation.shape(fixtures::RobotList()).empty());
}

TEST_F(LidarPointTest, GettersReturnConfiguration)
{
    const LidarPoint observation(LIDAR_INDEX, LOCAL, NO_STACK, FOUR_THREADS);

    EXPECT_EQ(observation.name(), "lidar_point_0");
    EXPECT_EQ(observation.indexLidar(), LIDAR_INDEX);
    EXPECT_TRUE(observation.local());
    EXPECT_EQ(observation.numThreads(), FOUR_THREADS);
}

// ============================================================================
// Value Tests
// ============================================================================

// Local points are distance times the unit beam direction in the lidar's own
// frame: an x row followed by a y row.
TEST_F(LidarPointTest, LocalPointsAreDistanceTimesBeamDirection)
{
    const LidarPoint observation(LIDAR_INDEX, LOCAL, NO_STACK, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), 2 * LIDAR_RAYS);
    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        const float angle = fixtures::rayAngle(i);
        ASSERT_NEAR(output[i], rayDistance(i) * std::cos(angle), EPSILON) << "ray " << i;
        ASSERT_NEAR(output[LIDAR_RAYS + i], rayDistance(i) * std::sin(angle), EPSILON) << "ray " << i;
    }
}

// +inf (no return) and -inf (too close) are clamped onto the sensor range
// first, so the points stay finite: at range_max and range_min on the beam
TEST_F(LidarPointTest, InfiniteReadingsGivePointsAtTheRangeLimits)
{
    murosim::sensor::Lidar &lidar = robot->getLidarAt(LIDAR_INDEX);
    lidar.getArrayLaserDistanceBuffer()[0] = std::numeric_limits<float>::infinity();
    lidar.getArrayLaserDistanceBuffer()[1] = -std::numeric_limits<float>::infinity();
    lidar.publishScan();
    robot->makeState();

    const std::vector<float> output = LidarPoint(LIDAR_INDEX, LOCAL, NO_STACK, ONE_THREAD)(robots());

    EXPECT_NEAR(output[0], fixtures::LIDAR_RANGE_MAX * std::cos(fixtures::rayAngle(0)), EPSILON);
    EXPECT_NEAR(output[LIDAR_RAYS], fixtures::LIDAR_RANGE_MAX * std::sin(fixtures::rayAngle(0)), EPSILON);
    EXPECT_NEAR(output[1], fixtures::LIDAR_RANGE_MIN * std::cos(fixtures::rayAngle(1)), EPSILON);
    EXPECT_NEAR(output[LIDAR_RAYS + 1], fixtures::LIDAR_RANGE_MIN * std::sin(fixtures::rayAngle(1)), EPSILON);
}

// World points go through the robot pose: with the robot facing +y a beam
// along local x points to world +y.
TEST_F(LidarPointTest, WorldPointsAreTransformedByTheRobotPose)
{
    const LidarPoint observation(LIDAR_INDEX, WORLD, NO_STACK, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), 2 * LIDAR_RAYS);
    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        const float angle = PI_2 + fixtures::rayAngle(i);
        ASSERT_NEAR(output[i], NEAR_WALL.x() + rayDistance(i) * std::cos(angle), EPSILON) << "ray " << i;
        ASSERT_NEAR(output[LIDAR_RAYS + i], NEAR_WALL.y() + rayDistance(i) * std::sin(angle), EPSILON) << "ray " << i;
    }
}

TEST_F(LidarPointTest, WorldPointsLieOnTheWalls)
{
    const LidarPoint observation(LIDAR_INDEX, WORLD, NO_STACK, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    for (size_t i = 0; i < LIDAR_RAYS; i++)
    {
        const float x = std::abs(output[i]);
        const float y = std::abs(output[LIDAR_RAYS + i]);
        ASSERT_NEAR(std::max(x, y), fixtures::WORLD_HALF_SIZE, EPSILON) << "ray " << i;
    }
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(LidarPointTest, StackIsNewestFirst)
{
    robot->setPose(0.0f, 0.0f, 0.0f);
    step();
    const LidarPoint observation(LIDAR_INDEX, LOCAL, 2, ONE_THREAD);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), 4 * LIDAR_RAYS);
    const float forward = fixtures::rayAngle(LIDAR_RAYS / 2); // a ray close to the robot's x axis
    const size_t ray = LIDAR_RAYS / 2;
    const float from_centre = fixtures::wallDistance(Eigen::Vector2f(0.0f, 0.0f), forward);
    EXPECT_NEAR(output[ray], from_centre * std::cos(forward), EPSILON);                    // newest, x row
    EXPECT_NEAR(output[2 * LIDAR_RAYS + ray], rayDistance(ray) * std::cos(forward), EPSILON); // one step back
}

// ============================================================================
// Threading Tests
// ============================================================================

TEST_F(LidarPointTest, MultipleThreadsMatchSingleThreadBitExact)
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
        fleet[i]->setPose(-3.0f + static_cast<float>(i), 1.0f, 0.3f * static_cast<float>(i));
    }
    step();
    const LidarPoint single(LIDAR_INDEX, WORLD, 2, ONE_THREAD);
    const LidarPoint threaded(LIDAR_INDEX, WORLD, 2, FOUR_THREADS);

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

TEST_F(LidarPointTest, MissingLidarThrows)
{
    const std::shared_ptr<Robot> blind = fixtures::holonomicRobot("blind", false);
    blind->makeState();
    const LidarPoint observation(LIDAR_INDEX, LOCAL, NO_STACK, ONE_THREAD);

    EXPECT_THROW(observation(fixtures::robotList(blind)), std::runtime_error);
    EXPECT_THROW(observation.shape(fixtures::robotList(blind)), std::runtime_error);
}
