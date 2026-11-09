// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/distance_to_goal.hpp"
#include "fixtures.hpp"
#include <vector>

namespace
{
using murosimrl::observation::robot::DistanceToGoal;
using fixtures::EPSILON;

constexpr float NO_NORMALIZATION = 0.0f;
constexpr float NORMALIZE_DISTANCE = 10.0f;
constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class DistanceToGoalTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(1.5f, -2.0f, 0.0f);
        robot->setGoal(4.5f, 2.0f, 0.5f); // offset (3, 4): distance 5
        step();
    }
};

// ============================================================================
// Value Tests
// ============================================================================

TEST_F(DistanceToGoalTest, ShapeIsBatchByOne)
{
    const DistanceToGoal observation(NO_NORMALIZATION, NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 1}));
    EXPECT_EQ(observation.name(), "distance_to_goal");
}

TEST_F(DistanceToGoalTest, RawDistanceInMetres)
{
    const DistanceToGoal observation(NO_NORMALIZATION, NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(1));
    EXPECT_NEAR(output[0], 5.0f, EPSILON);
    EXPECT_FLOAT_EQ(observation.normalizeDistance(), 0.0f);
}

TEST_F(DistanceToGoalTest, NormalizedByGivenDistance)
{
    const DistanceToGoal observation(NORMALIZE_DISTANCE, NO_STACK);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 0.5f, EPSILON);
    EXPECT_FLOAT_EQ(observation.normalizeDistance(), NORMALIZE_DISTANCE);
}

TEST_F(DistanceToGoalTest, NegativeNormalizationIsClampedToRaw)
{
    const DistanceToGoal observation(-3.0f, NO_STACK);

    EXPECT_FLOAT_EQ(observation.normalizeDistance(), 0.0f);
    EXPECT_NEAR(observation(robots())[0], 5.0f, EPSILON);
}

TEST_F(DistanceToGoalTest, OnGoalIsZero)
{
    robot->setPose(4.5f, 2.0f, 0.0f);
    step();
    const DistanceToGoal observation(NORMALIZE_DISTANCE, NO_STACK);

    EXPECT_FLOAT_EQ(observation(robots())[0], 0.0f);
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(DistanceToGoalTest, StackIsNewestFirst)
{
    robot->setPose(4.5f, -1.0f, 0.0f); // 3 m below the goal
    step();
    const DistanceToGoal observation(NO_NORMALIZATION, 2);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 2, 1}));
    ASSERT_EQ(output.size(), static_cast<size_t>(2));
    EXPECT_NEAR(output[0], 3.0f, EPSILON);
    EXPECT_NEAR(output[1], 5.0f, EPSILON);
}
