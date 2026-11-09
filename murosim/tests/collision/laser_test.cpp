// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/collision/laser.hpp"
#include "murosim/collision/lidar.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::geometry::Shape;
using murosim::geometry::World;
using murosim::geometry::primitives::pose;
using murosim::sensor::Lidar;

constexpr float EPSILON = 1e-5f;
constexpr float RANGE_MAX = 10.0f;
constexpr float INF = std::numeric_limits<float>::infinity();

// The ray closest to the perpendicular of an obstacle at distance d is at most
// half an angular step off it and reads at most d / cos(step / 2). For the
// lidars here (1 deg steps, d <= 5 m) that is at most 2e-4 m more than d.
constexpr float EPSILON_RAY = 1e-3f;

/// Half-circle lidar with 180 rays used by the lidarWith* tests.
Lidar halfCircleLidar()
{
    return Lidar(Eigen::Affine2f::Identity(), PI, 180, 0.0f, RANGE_MAX);
}

/// True if at least one ray hit something. The kernels alone (no
/// Lidar::applyNoise) leave a ray without a hit at its +inf start value.
bool anyRayHit(const Lidar &lidar)
{
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        if (std::isfinite(lidar.getLaserDistance(i)))
        {
            return true;
        }
    }
    return false;
}

/// Shortest reading over all rays.
float minRayDistance(const Lidar &lidar)
{
    float minimum = INF;
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        minimum = std::min(minimum, lidar.getLaserDistance(i));
    }
    return minimum;
}

void assertNoRayHit(const Lidar &lidar)
{
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_EQ(lidar.getLaserDistance(i), INF) << "ray " << i;
    }
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class CollisionLaserTest : public ::testing::Test
{
protected:
    // Quarter-circle lidar with 100 rays at the origin, looking along +x
    Lidar lidar{Eigen::Affine2f::Identity(), PI_2, 100, 0.0f, RANGE_MAX};
};

// ============================================================================
// Laser With Line Tests
// ============================================================================

TEST_F(CollisionLaserTest, LaserWithLine)
{
    const Line line_front(1.0f, 0.5f, 1.0f, -0.5f);
    const Line line_back(-0.1f, 0.5f, -0.1f, -0.5f);

    // A fresh lidar reads +inf (no return) everywhere
    assertNoRayHit(lidar);

    // A line behind the lidar is never hit
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                      lidar.getArrayLaserDistanceBuffer(), line_back);
    lidar.publishScan();
    assertNoRayHit(lidar);

    // A line in front: the rays that hit end on the line, all others keep
    // +inf
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                      lidar.getArrayLaserDistanceBuffer(), line_front);
    lidar.publishScan();
    EXPECT_TRUE(anyRayHit(lidar));

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        if (std::isfinite(lidar.getLaserDistance(i)))
        {
            const Eigen::Vector2f point = lidar.getLaserPoint(i);
            ASSERT_NEAR(point.x(), 1.0f, EPSILON) << "ray " << i;
            ASSERT_LE(std::abs(point.y()), 0.5f + EPSILON) << "ray " << i;
        }
        else
        {
            ASSERT_EQ(lidar.getLaserDistance(i), INF) << "ray " << i;
        }
    }
}

TEST_F(CollisionLaserTest, LaserWithLineJustBehindIsNotHit)
{
    // A line 0.5 mm behind the lidar, crossing the backward extension of every
    // ray: the intersections lie at t < 0, so no ray hits it. (A tolerance on
    // t used to accept such hits as obstacles a fraction of a millimetre away.)
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                      lidar.getArrayLaserDistanceBuffer(), -0.0005f, 5.0f, -0.0005f, -5.0f);
    lidar.publishScan();

    assertNoRayHit(lidar);
}

TEST_F(CollisionLaserTest, LaserRaysAreUnbounded)
{
    // The kernels store the nearest hit at any distance; the range limit is
    // the sensor model's job (Lidar::applyNoise), not the kernels'
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                      lidar.getArrayLaserDistanceBuffer(), 2.0f * RANGE_MAX, 50.0f, 2.0f * RANGE_MAX, -50.0f);
    lidar.publishScan();
    EXPECT_NEAR(minRayDistance(lidar), 2.0f * RANGE_MAX, EPSILON_RAY * 2.0f * RANGE_MAX);

    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                        lidar.getArrayLaserDistanceBuffer(), 3.0f * RANGE_MAX, 0.0f, 1.0f);
    lidar.publishScan();
    EXPECT_NEAR(minRayDistance(lidar), 3.0f * RANGE_MAX - 1.0f, EPSILON_RAY * 3.0f * RANGE_MAX);
}

TEST_F(CollisionLaserTest, LaserKeepsNearerStoredValue)
{
    // A value already stored is a cap: a farther hit never replaces it
    std::fill(lidar.getArrayLaserDistanceBuffer().data(),
              lidar.getArrayLaserDistanceBuffer().data() + lidar.getLaserSize(), 1.0f);
    murosim::collision::laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                      lidar.getArrayLaserDistanceBuffer(), 2.0f, 5.0f, 2.0f, -5.0f);
    murosim::collision::laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                        lidar.getArrayLaserDistanceBuffer(), 3.0f, 0.0f, 0.5f);
    lidar.publishScan();

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_FLOAT_EQ(lidar.getLaserDistance(i), 1.0f) << "ray " << i;
    }
}

TEST_F(CollisionLaserTest, LaserWithLineFloatOverload)
{
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                      lidar.getArrayLaserDistanceBuffer(), 2.0f, 1.0f, 2.0f, -1.0f);
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        if (std::isfinite(lidar.getLaserDistance(i)))
        {
            ASSERT_NEAR(lidar.getLaserPoint(i).x(), 2.0f, EPSILON) << "ray " << i;
        }
    }
}

TEST_F(CollisionLaserTest, LaserWithLineVectorOverload)
{
    const Eigen::Vector2f p1(2.0f, 1.0f);
    const Eigen::Vector2f p2(2.0f, -1.0f);

    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                      lidar.getArrayLaserDistanceBuffer(), p1, p2);
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        if (std::isfinite(lidar.getLaserDistance(i)))
        {
            ASSERT_NEAR(lidar.getLaserPoint(i).x(), 2.0f, EPSILON) << "ray " << i;
        }
    }
}

// ============================================================================
// Laser With Circle Tests
// ============================================================================

TEST_F(CollisionLaserTest, LaserWithCircle)
{
    const Circle circle_back(-1.0f, 0.0f, 0.5f);
    const Circle circle_front(1.0f, 0.0f, 0.5f);

    // A circle behind the lidar is never hit
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                        lidar.getArrayLaserDistanceBuffer(), circle_back);
    lidar.publishScan();
    assertNoRayHit(lidar);

    // A circle in front: the rays that hit end on its boundary, all others keep
    // +inf
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                        lidar.getArrayLaserDistanceBuffer(), circle_front);
    lidar.publishScan();
    EXPECT_TRUE(anyRayHit(lidar));

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        if (std::isfinite(lidar.getLaserDistance(i)))
        {
            const Eigen::Vector2f point = lidar.getLaserPoint(i) - circle_front.getPoint();
            ASSERT_NEAR(point.norm(), circle_front.getRadius(), EPSILON) << "ray " << i;
        }
        else
        {
            ASSERT_EQ(lidar.getLaserDistance(i), INF) << "ray " << i;
        }
    }
}

TEST_F(CollisionLaserTest, LaserWithCircleOriginInside)
{
    // The lidar origin inside the circle: solid-disk semantics, every ray reads
    // a contact (distance 0) instead of seeing through the obstacle
    const Circle circle(0.1f, -0.1f, 0.5f);

    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                        lidar.getArrayLaserDistanceBuffer(), circle);
    lidar.publishScan();

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_FLOAT_EQ(lidar.getLaserDistance(i), 0.0f) << "ray " << i;
    }
}

TEST_F(CollisionLaserTest, LaserWithCircleFloatOverload)
{
    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                        lidar.getArrayLaserDistanceBuffer(), 2.0f, 0.0f, 0.5f);
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 1.5f, EPSILON_RAY); // nearest point of the circle
}

TEST_F(CollisionLaserTest, LaserWithCircleVectorOverload)
{
    const Eigen::Vector2f center(2.0f, 0.0f);

    lidar.fillLaserDistanceWithInf();
    murosim::collision::laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(),
                                        lidar.getArrayLaserDistanceBuffer(), center, 0.5f);
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 1.5f, EPSILON_RAY); // nearest point of the circle
}

// ============================================================================
// Lidar With World Tests
// ============================================================================

TEST(CollisionLidarWithWorld, Line)
{
    Lidar lidar = halfCircleLidar();
    const World world(Shape({{2.0f, 1.0f, 2.0f, -1.0f}}, {}));

    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithWorld(lidar, Eigen::Affine2f::Identity(), world);
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 2.0f, EPSILON_RAY);
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        if (std::isfinite(lidar.getLaserDistance(i)))
        {
            ASSERT_NEAR(lidar.getLaserPoint(i).x(), 2.0f, EPSILON) << "ray " << i;
        }
    }
}

TEST(CollisionLidarWithWorld, Circle)
{
    Lidar lidar = halfCircleLidar();
    const World world(Shape({}, {{3.0f, 0.0f, 0.5f}}));

    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithWorld(lidar, Eigen::Affine2f::Identity(), world);
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 2.5f, EPSILON_RAY); // nearest point of the circle
}

TEST(CollisionLidarWithWorld, RobotPose)
{
    Lidar lidar = halfCircleLidar();
    const World world(Shape({{2.0f, 1.0f, 2.0f, -1.0f}}, {}));

    // Robot at x = 1, line at x = 2: the closest ray reads 1 m
    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithWorld(lidar, pose(1.0f, 0.0f, 0.0f), world);
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 1.0f, EPSILON_RAY);
}

TEST(CollisionLidarWithWorld, Empty)
{
    Lidar lidar = halfCircleLidar();
    const World world{Shape()};

    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithWorld(lidar, Eigen::Affine2f::Identity(), world);
    lidar.publishScan();

    assertNoRayHit(lidar);
}

// ============================================================================
// Lidar With Shape Tests
// ============================================================================

TEST(CollisionLidarWithShape, Line)
{
    Lidar lidar = halfCircleLidar();
    const Shape shape({{0.0f, 1.0f, 0.0f, -1.0f}}, {});

    // The shape pose puts the line at x = 2
    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithShape(lidar, Eigen::Affine2f::Identity(), shape, pose(2.0f, 0.0f, 0.0f));
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 2.0f, EPSILON_RAY);
    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        if (std::isfinite(lidar.getLaserDistance(i)))
        {
            ASSERT_NEAR(lidar.getLaserPoint(i).x(), 2.0f, EPSILON) << "ray " << i;
        }
    }
}

TEST(CollisionLidarWithShape, Circle)
{
    Lidar lidar = halfCircleLidar();
    const Shape shape({}, {{0.0f, 0.0f, 0.5f}});

    // The shape pose puts the circle at x = 3
    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithShape(lidar, Eigen::Affine2f::Identity(), shape, pose(3.0f, 0.0f, 0.0f));
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 2.5f, EPSILON_RAY); // nearest point of the circle
}

TEST(CollisionLidarWithShape, Square)
{
    Lidar lidar(Eigen::Affine2f::Identity(), 2.0f * PI, 360, 0.0f, RANGE_MAX);
    const Shape square = murosim::geometry::primitives::square(2.0f);

    // Lidar at the centre of the square: every ray hits an edge between the
    // edge distance (1.0) and the corner distance (sqrt 2)
    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithShape(lidar, Eigen::Affine2f::Identity(), square, Eigen::Affine2f::Identity());
    lidar.publishScan();

    for (size_t i = 0; i < lidar.getLaserSize(); i++)
    {
        ASSERT_GE(lidar.getLaserDistance(i), 1.0f - EPSILON) << "ray " << i;
        ASSERT_LE(lidar.getLaserDistance(i), M_SQRT2f + EPSILON) << "ray " << i;
    }
    EXPECT_NEAR(minRayDistance(lidar), 1.0f, EPSILON_RAY);
}

TEST(CollisionLidarWithShape, BothPosesMoved)
{
    Lidar lidar = halfCircleLidar();
    const Shape shape({{0.0f, 1.0f, 0.0f, -1.0f}}, {});

    // Robot at x = 1, line at x = 3: the closest ray reads 2 m
    lidar.fillLaserDistanceWithInf();
    murosim::collision::lidarWithShape(lidar, pose(1.0f, 0.0f, 0.0f), shape, pose(3.0f, 0.0f, 0.0f));
    lidar.publishScan();

    EXPECT_TRUE(anyRayHit(lidar));
    EXPECT_NEAR(minRayDistance(lidar), 2.0f, EPSILON_RAY);
}
