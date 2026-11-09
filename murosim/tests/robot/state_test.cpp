// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/robot/state.hpp"
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <limits>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::actuator::HolonomicSimple;
using murosim::navigation::Node;
using murosim::robot::State;
using murosim::sensor::Lidar;

constexpr float EPSILON = 1e-5f;
constexpr float TIME_STEP = 0.1f;

// Limits of the fixture actuator; the acceleration limit makes acceleration
// commands storable and gives the velocity a known ramp
constexpr float VELOCITY_X_MIN = -0.5f;
constexpr float VELOCITY_X_MAX = 2.0f;
constexpr float VELOCITY_Y_MAX = 1.0f;
constexpr float VELOCITY_ANGULAR_MAX = PI_2;
constexpr float ACCELERATION_LINEAR = 0.5f;
constexpr float ACCELERATION_ANGULAR = PI_4;

constexpr float POSE_X = 1.0f;
constexpr float POSE_Y = 2.0f;
constexpr float POSE_ANGLE = 0.5f;
constexpr float GOAL_X = 5.0f;
constexpr float GOAL_Y = 6.0f;
constexpr float GOAL_RADIUS = 1.0f;
constexpr size_t LIDAR_RAYS = 100;
constexpr float LIDAR_RANGE_MAX = 10.0f;

Lidar frontLidar(const size_t rays, const float range_max)
{
    return Lidar(Eigen::Affine2f::Identity(), PI, rays, 0.0f, range_max);
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(StateConstructor, DefaultConstructorIsIdentityWithoutLidars)
{
    const State state;

    EXPECT_TRUE(state.getPose().isApprox(Eigen::Affine2f::Identity()));
    EXPECT_EQ(state.getLidarDistanceSize(), static_cast<size_t>(0));
    EXPECT_TRUE(state.getActuatorCommandVelocity().isZero());
    EXPECT_TRUE(state.getActuatorCommandAcceleration().isZero());
    EXPECT_TRUE(state.getActuatorVelocity().isZero());
}

// ============================================================================
// Test Fixture
// ============================================================================

class StateTest : public ::testing::Test
{
protected:
    HolonomicSimple actuator{VELOCITY_X_MIN,        VELOCITY_X_MAX,      VELOCITY_Y_MAX,
                             VELOCITY_ANGULAR_MAX,  ACCELERATION_LINEAR, ACCELERATION_ANGULAR};
    std::vector<Lidar> lidars = {frontLidar(LIDAR_RAYS, LIDAR_RANGE_MAX)};
    Eigen::Affine2f pose = murosim::geometry::primitives::pose(POSE_X, POSE_Y, POSE_ANGLE);
    Node goal{GOAL_X, GOAL_Y, GOAL_RADIUS};
};

// ============================================================================
// Snapshot Tests
// ============================================================================

TEST_F(StateTest, ConstructorStoresPose)
{
    const State state(actuator, lidars, pose, goal);

    EXPECT_FLOAT_EQ(state.getPose().translation().x(), POSE_X);
    EXPECT_FLOAT_EQ(state.getPose().translation().y(), POSE_Y);
    EXPECT_NEAR(Eigen::Rotation2Df(state.getPose().rotation()).angle(), POSE_ANGLE, EPSILON);
}

TEST_F(StateTest, ConstructorStoresGoal)
{
    const State state(actuator, lidars, pose, goal);

    EXPECT_FLOAT_EQ(state.getGoal().getX(), GOAL_X);
    EXPECT_FLOAT_EQ(state.getGoal().getY(), GOAL_Y);
    EXPECT_FLOAT_EQ(state.getGoal().getRadius(), GOAL_RADIUS);
}

TEST_F(StateTest, ConstructorCopiesActuatorCommandVelocity)
{
    actuator.setCommandVelocity(0.5f, 0.3f, 0.1f);

    const State state(actuator, lidars, pose, goal);

    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().x(), 0.5f);
    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().y(), 0.3f);
    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().z(), 0.1f);
}

TEST_F(StateTest, ConstructorCopiesActuatorCommandAcceleration)
{
    actuator.setCommandAcceleration(0.1f, 0.2f, 0.3f);

    const State state(actuator, lidars, pose, goal);

    EXPECT_FLOAT_EQ(state.getActuatorCommandAcceleration().x(), 0.1f);
    EXPECT_FLOAT_EQ(state.getActuatorCommandAcceleration().y(), 0.2f);
    EXPECT_FLOAT_EQ(state.getActuatorCommandAcceleration().z(), 0.3f);
}

TEST_F(StateTest, ConstructorCopiesActuatorVelocity)
{
    actuator.setCommandVelocity(1.0f, 0.0f, 0.0f);
    actuator.step(TIME_STEP); // ramps from rest at ACCELERATION_LINEAR

    const State state(actuator, lidars, pose, goal);

    EXPECT_NEAR(state.getActuatorVelocity().x(), ACCELERATION_LINEAR * TIME_STEP, EPSILON);
    EXPECT_FLOAT_EQ(state.getActuatorVelocity().y(), 0.0f);
    EXPECT_FLOAT_EQ(state.getActuatorVelocity().z(), 0.0f);
}

TEST_F(StateTest, ConstructorIsASnapshot)
{
    const State state(actuator, lidars, pose, goal);

    actuator.setCommandVelocity(1.0f, 0.0f, 0.0f); // changed after the snapshot

    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().x(), 0.0f);
}

// ============================================================================
// Lidar Distance Tests
// ============================================================================

TEST_F(StateTest, ConstructorCopiesLidarDistances)
{
    // the state copies the published scan: one hit, the other rays no return
    lidars[0].getArrayLaserDistanceBuffer()[0] = 2.5f;
    lidars[0].publishScan();

    const State state(actuator, lidars, pose, goal);

    ASSERT_EQ(state.getLidarDistanceSize(), static_cast<size_t>(1));
    const murosim::simd::ArrayFloat &distances = state.getLidarDistanceAt(0);
    ASSERT_EQ(distances.size(), LIDAR_RAYS);
    EXPECT_FLOAT_EQ(distances[0], 2.5f);
    EXPECT_EQ(distances[LIDAR_RAYS - 1], std::numeric_limits<float>::infinity());
}

TEST_F(StateTest, EmptyLidars)
{
    const std::vector<Lidar> no_lidars;

    const State state(actuator, no_lidars, pose, goal);

    EXPECT_EQ(state.getLidarDistanceSize(), static_cast<size_t>(0));
}

TEST_F(StateTest, MultipleLidarsKeepOrder)
{
    const std::vector<Lidar> three_lidars = {frontLidar(100, 10.0f), frontLidar(50, 5.0f), frontLidar(360, 30.0f)};

    const State state(actuator, three_lidars, pose, goal);

    ASSERT_EQ(state.getLidarDistanceSize(), static_cast<size_t>(3));
    EXPECT_EQ(state.getLidarDistanceAt(0).size(), static_cast<size_t>(100));
    EXPECT_EQ(state.getLidarDistanceAt(1).size(), static_cast<size_t>(50));
    EXPECT_EQ(state.getLidarDistanceAt(2).size(), static_cast<size_t>(360));
}

// ============================================================================
// Assign Tests
// ============================================================================

TEST_F(StateTest, AssignReplacesEveryField)
{
    State state(actuator, lidars, pose, goal);
    const std::vector<Lidar> two_lidars = {frontLidar(50, 5.0f), frontLidar(25, 3.0f)};
    const Eigen::Affine2f new_pose = murosim::geometry::primitives::pose(3.0f, 4.0f, PI_2);
    const Node new_goal(7.0f, 8.0f, 2.0f);
    actuator.setCommandVelocity(0.5f, 0.0f, 0.0f);

    state.assign(actuator, two_lidars, new_pose, new_goal);

    EXPECT_FLOAT_EQ(state.getPose().translation().x(), 3.0f);
    EXPECT_FLOAT_EQ(state.getPose().translation().y(), 4.0f);
    EXPECT_FLOAT_EQ(state.getGoal().getX(), 7.0f);
    EXPECT_FLOAT_EQ(state.getActuatorCommandVelocity().x(), 0.5f);
    ASSERT_EQ(state.getLidarDistanceSize(), static_cast<size_t>(2));
    EXPECT_EQ(state.getLidarDistanceAt(0).size(), static_cast<size_t>(50));
    EXPECT_EQ(state.getLidarDistanceAt(1).size(), static_cast<size_t>(25));
}

TEST_F(StateTest, AssignOnDefaultStateAllocatesLidarBuffers)
{
    State state;

    state.assign(actuator, lidars, pose, goal);

    ASSERT_EQ(state.getLidarDistanceSize(), static_cast<size_t>(1));
    EXPECT_EQ(state.getLidarDistanceAt(0).size(), LIDAR_RAYS);
    EXPECT_FLOAT_EQ(state.getPose().translation().x(), POSE_X);
}
