// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot/subgoal_direction.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <vector>

namespace
{
using murosim::math::PI_2;
using murosimrl::observation::robot::SubgoalDirection;
using fixtures::EPSILON;

constexpr float LOOKAHEAD = 2.0f;
constexpr size_t TWO_SUBGOALS = 2;
constexpr bool RAW = false;
constexpr bool NORMALIZED = true;

/// Straight path along +x from the origin, 3 m long, with points every metre.
std::vector<Eigen::Vector2f> straightPath()
{
    return {Eigen::Vector2f(1.0f, 0.0f), Eigen::Vector2f(2.0f, 0.0f), Eigen::Vector2f(3.0f, 0.0f)};
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class SubgoalDirectionTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPathPoints(straightPath());
        step();
    }
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(SubgoalDirectionTest, ShapeIsBatchByTwoPerSubgoal)
{
    const SubgoalDirection observation(LOOKAHEAD, TWO_SUBGOALS, RAW);

    // the stack size is fixed to 1, so the shape carries a stack dimension
    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 1, 4}));
    EXPECT_EQ(observation.name(), "subgoal_direction");
    EXPECT_EQ(observation.numSubgoals(), TWO_SUBGOALS);
    EXPECT_FLOAT_EQ(observation.lookaheadDistance(), LOOKAHEAD);
    EXPECT_FALSE(observation.normalize());
}

TEST_F(SubgoalDirectionTest, ArgumentsAreClampedToUsableValues)
{
    const SubgoalDirection observation(0.0f, 0, NORMALIZED);

    EXPECT_FLOAT_EQ(observation.lookaheadDistance(), 0.1f);
    EXPECT_EQ(observation.numSubgoals(), static_cast<size_t>(1));
    EXPECT_EQ(observation.shape(robots()), (std::vector<size_t>{1, 1, 2}));
}

// ============================================================================
// Direction Tests
// ============================================================================

// Subgoal s sits at (s + 1) / n of the lookahead along the path; direction s
// runs from the previous subgoal (the robot for s == 0) to subgoal s.
TEST_F(SubgoalDirectionTest, RawDirectionsChainAlongThePath)
{
    const SubgoalDirection observation(LOOKAHEAD, TWO_SUBGOALS, RAW);

    const std::vector<float> output = observation(robots());

    ASSERT_EQ(output.size(), static_cast<size_t>(4));
    EXPECT_NEAR(output[0], 1.0f, EPSILON); // robot (0, 0) to subgoal (1, 0)
    EXPECT_NEAR(output[1], 0.0f, EPSILON);
    EXPECT_NEAR(output[2], 1.0f, EPSILON); // subgoal (1, 0) to subgoal (2, 0)
    EXPECT_NEAR(output[3], 0.0f, EPSILON);
}

TEST_F(SubgoalDirectionTest, NormalizedDirectionsAreUnitLength)
{
    const SubgoalDirection observation(3.0f * LOOKAHEAD, TWO_SUBGOALS, NORMALIZED); // subgoals 3 m apart

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 1.0f, EPSILON);
    EXPECT_NEAR(output[1], 0.0f, EPSILON);
}

TEST_F(SubgoalDirectionTest, DirectionsAreInTheRobotFrame)
{
    robot->setPose(0.0f, 0.0f, PI_2); // facing +y: the +x path lies to the robot's right
    step();
    const SubgoalDirection observation(LOOKAHEAD, TWO_SUBGOALS, RAW);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 0.0f, EPSILON);
    EXPECT_NEAR(output[1], -1.0f, EPSILON);
    EXPECT_NEAR(output[2], 0.0f, EPSILON);
    EXPECT_NEAR(output[3], -1.0f, EPSILON);
}

TEST_F(SubgoalDirectionTest, SubgoalsBeyondThePathClampToTheGoal)
{
    const SubgoalDirection observation(10.0f, TWO_SUBGOALS, RAW); // both subgoals past the 3 m path

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 3.0f, EPSILON); // robot to the goal (3, 0)
    EXPECT_NEAR(output[1], 0.0f, EPSILON);
    EXPECT_NEAR(output[2], 0.0f, EPSILON); // goal to goal: no offset
    EXPECT_NEAR(output[3], 0.0f, EPSILON);
}

TEST_F(SubgoalDirectionTest, NormalizedDegenerateDirectionIsZero)
{
    const SubgoalDirection observation(10.0f, TWO_SUBGOALS, NORMALIZED);

    const std::vector<float> output = observation(robots());

    EXPECT_NEAR(output[0], 1.0f, EPSILON);
    EXPECT_FLOAT_EQ(output[2], 0.0f); // zero length is not normalized into NaN
    EXPECT_FLOAT_EQ(output[3], 0.0f);
}

TEST_F(SubgoalDirectionTest, EmptyPathYieldsZeros)
{
    robot->setPathPoints({});
    const SubgoalDirection observation(LOOKAHEAD, TWO_SUBGOALS, RAW);

    const std::vector<float> output = observation(robots());

    EXPECT_EQ(output, (std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f}));
}
