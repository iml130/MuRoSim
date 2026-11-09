// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot_simulation/pose_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <vector>

namespace
{
using murosimrl::observation::robot_simulation::PoseMap;

constexpr size_t PATCH_DIM = 33;
constexpr bool WITHOUT_BUFFER = false;
constexpr bool WITH_BUFFER = true;
constexpr size_t NO_STACK = 0;
constexpr float FREE = 0.0f;
constexpr float OCCUPIED = 1.0f;

const Eigen::Vector2f NEAR_WALL(4.5f, 0.0f);
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class PoseMapTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(NEAR_WALL.x(), NEAR_WALL.y(), 0.0f);
        step();
    }

    std::vector<float> referencePatch(const Eigen::Vector2f &position, const bool with_buffer) const
    {
        Eigen::Vector2i center;
        sim->getGridMap().fromWorldIntoMap(position, center);
        std::vector<float> patch(PATCH_DIM * PATCH_DIM);
        murosimrl::util::grid::extractMapPatch(sim->getGridMap(), center, PATCH_DIM,
                                               murosimrl::util::grid::gridValueLut(with_buffer), patch.data());
        return patch;
    }
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(PoseMapTest, ShapeIsRobotsByPatch)
{
    const PoseMap observation(PATCH_DIM, WITHOUT_BUFFER, NO_STACK);

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, PATCH_DIM, PATCH_DIM}));
    EXPECT_EQ(PoseMap(PATCH_DIM, WITHOUT_BUFFER, 2).shape(sims()), (std::vector<size_t>{1, 2, PATCH_DIM, PATCH_DIM}));
    EXPECT_EQ(observation.name(), "pose_map");
    EXPECT_EQ(observation.patchDim(), PATCH_DIM);
    EXPECT_FALSE(observation.withBuffer());
}

// ============================================================================
// Patch Tests
// ============================================================================

TEST_F(PoseMapTest, PatchMatchesTheUtilExtraction)
{
    const PoseMap observation(PATCH_DIM, WITH_BUFFER, NO_STACK);

    const std::vector<float> output = observation(sims());

    const std::vector<float> expected = referencePatch(NEAR_WALL, WITH_BUFFER);
    ASSERT_EQ(output.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
    {
        ASSERT_FLOAT_EQ(output[i], expected[i]) << "cell " << i;
    }
}

TEST_F(PoseMapTest, PatchIsCentredOnTheRobot)
{
    const PoseMap observation(PATCH_DIM, WITHOUT_BUFFER, NO_STACK);

    const std::vector<float> output = observation(sims());

    const size_t middle = PATCH_DIM / 2;
    EXPECT_FLOAT_EQ(output[middle * PATCH_DIM + middle], FREE);        // the robot stands in free space
    EXPECT_FLOAT_EQ(output[middle * PATCH_DIM + PATCH_DIM - 1], OCCUPIED); // beyond the +x wall
    EXPECT_GT(std::count(output.begin(), output.end(), OCCUPIED), 0);
}

// ============================================================================
// Stack Tests
// ============================================================================

TEST_F(PoseMapTest, StackIsNewestFirst)
{
    robot->setPose(0.0f, 0.0f, 0.0f); // open space: an all-free patch
    step();
    const PoseMap observation(PATCH_DIM, WITHOUT_BUFFER, 2);

    const std::vector<float> output = observation(sims());

    const size_t step = PATCH_DIM * PATCH_DIM;
    ASSERT_EQ(output.size(), 2 * step);
    EXPECT_EQ(static_cast<size_t>(std::count(output.begin(), output.begin() + step, FREE)), step); // newest
    EXPECT_GT(std::count(output.begin() + step, output.end(), OCCUPIED), 0); // the wall patch one step back
}
