// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/simulation/velocity_map.hpp"
#include "murosim/grid/drawing.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <vector>

namespace
{
using murosimrl::observation::simulation::VelocityMap;

constexpr size_t MAP_SIZE = 128;
constexpr size_t NO_HISTORY = 0;
constexpr size_t TWO_HISTORY = 2;
constexpr bool CENTRE_CELL = false;
constexpr bool FOOTPRINT = true;
constexpr size_t NO_STACK = 0;
constexpr float VELOCITY_Y = 0.5f; // body +y command; heading 0 makes it world +y
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// The robot drives in world +y for five steps, so its cells carry a pure vy.
class VelocityMapTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(0.0f, 0.0f, 0.0f);
        robot->setActuatorCommandVelocity(0.0f, VELOCITY_Y, 0.0f);
        step(5);
    }

    /// Cells of `channel` in the first stacked frame that are not zero.
    static std::set<size_t> nonZeroCells(const std::vector<float> &output, const size_t channel)
    {
        std::set<size_t> cells;
        for (size_t i = 0; i < MAP_SIZE * MAP_SIZE; i++)
        {
            if (output[channel * MAP_SIZE * MAP_SIZE + i] != 0.0f)
            {
                cells.insert(i);
            }
        }
        return cells;
    }

    /// The cells the grid map's own footprint rasterization covers for the robot now.
    std::set<size_t> footprintCells() const
    {
        const murosim::grid::Map &map = sim->getGridMap();
        const size_t start_x = (MAP_SIZE - map.sizeX()) / 2;
        const size_t start_y = (MAP_SIZE - map.sizeY()) / 2;
        std::set<size_t> cells;
        murosim::grid::drawing::circleCells(map, robot->getPosition(), robot->getCollisionRadius(),
                                            [&cells, start_x, start_y](const Eigen::Vector2i pixel)
        {
            cells.insert((static_cast<size_t>(pixel.y()) + start_y) * MAP_SIZE + static_cast<size_t>(pixel.x()) +
                         start_x);
        });
        return cells;
    }
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(VelocityMapTest, ShapeHasTwoChannelsWithoutHistory)
{
    const VelocityMap observation(MAP_SIZE, NO_HISTORY, FOOTPRINT, NO_STACK);

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 2, MAP_SIZE, MAP_SIZE}));
    EXPECT_EQ(observation.name(), "velocity_map");
    EXPECT_EQ(observation.mapSize(), MAP_SIZE);
    EXPECT_EQ(observation.historySize(), NO_HISTORY);
    EXPECT_TRUE(observation.useCollisionRadius());
}

TEST_F(VelocityMapTest, ShapeHasThreeChannelsWithHistoryAndStack)
{
    const VelocityMap observation(MAP_SIZE, TWO_HISTORY, FOOTPRINT, 2);

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 2, 3, MAP_SIZE, MAP_SIZE}));
}

// ============================================================================
// Velocity Channel Tests
// ============================================================================

TEST_F(VelocityMapTest, OutputIsFinite)
{
    const VelocityMap observation(MAP_SIZE, NO_HISTORY, FOOTPRINT, NO_STACK);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(output.size(), 2 * MAP_SIZE * MAP_SIZE);
    for (size_t i = 0; i < output.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(output[i])) << "cell " << i;
    }
}

// The observation must rasterize robot footprints with the exact same cells
// as the grid map (drawing::circleCells), including the half-cell overlap,
// so the policy never sees a robot thinner than the map's obstacle footprint.
TEST_F(VelocityMapTest, FootprintMatchesGridMapRasterization)
{
    const VelocityMap observation(MAP_SIZE, NO_HISTORY, FOOTPRINT, NO_STACK);

    const std::vector<float> output = observation(sims());

    const std::set<size_t> expected = footprintCells();
    ASSERT_FALSE(expected.empty());
    EXPECT_EQ(nonZeroCells(output, 1), expected); // the vy channel carries the footprint
    EXPECT_TRUE(nonZeroCells(output, 0).empty());  // no x velocity anywhere
}

// Raster rows are y-flipped relative to the world (image convention), so the
// vy channel is negated: a robot moving in +world-y reads a negative vy.
TEST_F(VelocityMapTest, VyChannelUsesTheImageFrame)
{
    const VelocityMap observation(MAP_SIZE, NO_HISTORY, FOOTPRINT, NO_STACK);

    const std::vector<float> output = observation(sims());

    const std::set<size_t> cells = nonZeroCells(output, 1);
    ASSERT_FALSE(cells.empty());
    for (const size_t cell : cells)
    {
        ASSERT_FLOAT_EQ(output[MAP_SIZE * MAP_SIZE + cell], -VELOCITY_Y) << "cell " << cell;
    }
}

TEST_F(VelocityMapTest, CentreCellOnlyMarksOneCell)
{
    const VelocityMap observation(MAP_SIZE, NO_HISTORY, CENTRE_CELL, NO_STACK);

    const std::vector<float> output = observation(sims());

    EXPECT_EQ(nonZeroCells(output, 1).size(), static_cast<size_t>(1));
    EXPECT_FALSE(observation.useCollisionRadius());
}

TEST_F(VelocityMapTest, StationaryRobotLeavesTheMapZero)
{
    robot->setActuatorCommandVelocity(0.0f, 0.0f, 0.0f);
    step();
    const VelocityMap observation(MAP_SIZE, NO_HISTORY, FOOTPRINT, NO_STACK);

    const std::vector<float> output = observation(sims());

    EXPECT_EQ(static_cast<size_t>(std::count(output.begin(), output.end(), 0.0f)), output.size());
}

// ============================================================================
// History Channel Tests
// ============================================================================

TEST_F(VelocityMapTest, HistoryChannelFadesOlderStates)
{
    const VelocityMap observation(MAP_SIZE, TWO_HISTORY, FOOTPRINT, NO_STACK);

    const std::vector<float> output = observation(sims());

    // The newest state writes 1.0, the state before it 0.5 where the newer
    // footprint did not overwrite it; every other cell stays 0
    ASSERT_EQ(output.size(), 3 * MAP_SIZE * MAP_SIZE);
    const size_t history = 2 * MAP_SIZE * MAP_SIZE;
    size_t newest = 0;
    size_t older = 0;
    for (size_t i = 0; i < MAP_SIZE * MAP_SIZE; i++)
    {
        const float value = output[history + i];
        ASSERT_TRUE(value == 0.0f || value == 0.5f || value == 1.0f) << "cell " << i << " reads " << value;
        newest += value == 1.0f;
        older += value == 0.5f;
    }
    EXPECT_EQ(newest, footprintCells().size());
    EXPECT_GT(older, static_cast<size_t>(0)); // the robot moved on, so part of the old footprint shows
}

// ============================================================================
// Contract Tests
// ============================================================================

TEST_F(VelocityMapTest, MapSizeZeroThrows)
{
    EXPECT_THROW(VelocityMap(0, NO_HISTORY, FOOTPRINT, NO_STACK), std::runtime_error);
}

TEST_F(VelocityMapTest, MapSmallerThanGridThrows)
{
    const VelocityMap observation(sim->getGridMap().sizeX() - 1, NO_HISTORY, FOOTPRINT, NO_STACK);

    EXPECT_THROW(observation(sims()), std::runtime_error);
}
