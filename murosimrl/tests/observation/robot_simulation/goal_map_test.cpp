// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot_simulation/goal_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <memory>
#include <vector>

namespace
{
using murosimrl::observation::robot_simulation::GoalMap;

constexpr size_t PATCH_DIM = 33; // 3.3 m at the fixture grid scale
constexpr bool WITHOUT_BUFFER = false;
constexpr bool WITH_BUFFER = true;
constexpr float FREE = 0.0f;
constexpr float OCCUPIED = 1.0f;
constexpr float BUFFER = 0.5f;

const Eigen::Vector2f OPEN_GOAL(3.0f, 3.0f);   // patch entirely in free space
const Eigen::Vector2f WALL_GOAL(4.5f, 0.0f);   // patch reaches past the +x wall
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class GoalMapTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setGoal(OPEN_GOAL.x(), OPEN_GOAL.y(), 0.5f);
        step();
    }

    /// The patch the util layer extracts around `goal`, the single owner of the geometry.
    std::vector<float> referencePatch(const Eigen::Vector2f &goal, const bool with_buffer) const
    {
        Eigen::Vector2i center;
        sim->getGridMap().fromWorldIntoMap(goal, center);
        std::vector<float> patch(PATCH_DIM * PATCH_DIM);
        murosimrl::util::grid::extractMapPatch(sim->getGridMap(), center, PATCH_DIM,
                                               murosimrl::util::grid::gridValueLut(with_buffer), patch.data());
        return patch;
    }
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(GoalMapTest, ShapeIsRobotsByChannelByPatch)
{
    const GoalMap observation(PATCH_DIM, WITHOUT_BUFFER);

    // stack_size is fixed to 1: the size-1 dimension is the channel axis
    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 1, PATCH_DIM, PATCH_DIM}));
    EXPECT_EQ(observation.name(), "goal_map");
    EXPECT_EQ(observation.patchDim(), PATCH_DIM);
    EXPECT_FALSE(observation.withBuffer());
}

// ============================================================================
// Patch Tests
// ============================================================================

TEST_F(GoalMapTest, PatchInOpenSpaceIsAllFree)
{
    const GoalMap observation(PATCH_DIM, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(output.size(), PATCH_DIM * PATCH_DIM);
    EXPECT_EQ(static_cast<size_t>(std::count(output.begin(), output.end(), FREE)), output.size());
}

TEST_F(GoalMapTest, PatchMatchesTheUtilExtraction)
{
    robot->setGoal(WALL_GOAL.x(), WALL_GOAL.y(), 0.5f);
    step();
    const GoalMap observation(PATCH_DIM, WITH_BUFFER);

    const std::vector<float> output = observation(sims());

    const std::vector<float> expected = referencePatch(WALL_GOAL, WITH_BUFFER);
    ASSERT_EQ(output.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
    {
        ASSERT_FLOAT_EQ(output[i], expected[i]) << "cell " << i;
    }
    EXPECT_GT(std::count(output.begin(), output.end(), OCCUPIED), 0); // the wall and the outside
    EXPECT_GT(std::count(output.begin(), output.end(), BUFFER), 0);   // the band along it
}

TEST_F(GoalMapTest, PatchIsCentredOnTheGoal)
{
    robot->setGoal(WALL_GOAL.x(), WALL_GOAL.y(), 0.5f);
    step();
    const GoalMap observation(PATCH_DIM, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());

    // The goal cell itself is free, the last column of the patch (0.5 m further
    // in +x than the wall) lies outside the map
    const size_t middle = PATCH_DIM / 2;
    EXPECT_FLOAT_EQ(output[middle * PATCH_DIM + middle], FREE);
    EXPECT_FLOAT_EQ(output[middle * PATCH_DIM + PATCH_DIM - 1], OCCUPIED);
    EXPECT_FLOAT_EQ(output[middle * PATCH_DIM], FREE);
}

TEST_F(GoalMapTest, WithoutBufferThePatchIsBinary)
{
    robot->setGoal(WALL_GOAL.x(), WALL_GOAL.y(), 0.5f);
    step();
    const GoalMap observation(PATCH_DIM, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());

    for (size_t i = 0; i < output.size(); i++)
    {
        ASSERT_TRUE(output[i] == FREE || output[i] == OCCUPIED) << "cell " << i << " reads " << output[i];
    }
}

// ============================================================================
// Batch Tests
// ============================================================================

TEST_F(GoalMapTest, BatchHoldsOnePatchPerRobot)
{
    const std::shared_ptr<murosim::robot::Robot> other = fixtures::holonomicRobot("other");
    sim->addRobot(other);
    sim->reset();
    robot->setGoal(OPEN_GOAL.x(), OPEN_GOAL.y(), 0.5f);
    other->setGoal(WALL_GOAL.x(), WALL_GOAL.y(), 0.5f);
    step();
    const GoalMap observation(PATCH_DIM, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{2, 1, PATCH_DIM, PATCH_DIM}));
    ASSERT_EQ(output.size(), 2 * PATCH_DIM * PATCH_DIM);
    const size_t step = PATCH_DIM * PATCH_DIM;
    EXPECT_EQ(static_cast<size_t>(std::count(output.begin(), output.begin() + step, FREE)), step);
    EXPECT_GT(std::count(output.begin() + step, output.end(), OCCUPIED), 0);
}
