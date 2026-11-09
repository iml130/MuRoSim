// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/goal.hpp"
#include "fixtures.hpp"
#include <vector>

namespace
{
using murosimrl::observation::robot::Goal;

constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class GoalTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setGoal(4.5f, 2.0f, 0.5f);
        step();
    }
};

// ============================================================================
// Value Tests
// ============================================================================

TEST_F(GoalTest, ShapeIsBatchByThree)
{
    const Goal observation(NO_STACK);

    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 3}));
    EXPECT_EQ(observation.name(), "goal");
}

TEST_F(GoalTest, ValuesAreCentreAndRadius)
{
    const Goal observation(NO_STACK);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(output[0], 4.5f);
    EXPECT_FLOAT_EQ(output[1], 2.0f);
    EXPECT_FLOAT_EQ(output[2], 0.5f);
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(GoalTest, StackIsNewestFirst)
{
    robot->setGoal(-1.0f, -1.0f, 0.25f);
    step();
    const Goal observation(2);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 2, 3}));
    ASSERT_EQ(output.size(), static_cast<size_t>(6));
    EXPECT_FLOAT_EQ(output[0], -1.0f); // newest goal
    EXPECT_FLOAT_EQ(output[2], 0.25f);
    EXPECT_FLOAT_EQ(output[3], 4.5f); // the goal one step back
    EXPECT_FLOAT_EQ(output[5], 0.5f);
}
