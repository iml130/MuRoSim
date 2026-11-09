// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/simulation/path_map.hpp"
#include "murosimrl/observation/simulation/occupancy_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace
{
using murosimrl::observation::simulation::OccupancyMap;
using murosimrl::observation::simulation::PathMap;

constexpr size_t MAP_SIZE = 128;
constexpr bool WITHOUT_BUFFER = false;
constexpr float PATH = -1.0f;

const std::vector<Eigen::Vector2f> THREE_POINTS = {Eigen::Vector2f(-3.0f, -3.0f), Eigen::Vector2f(0.0f, 0.0f),
                                                   Eigen::Vector2f(3.0f, 3.0f)};
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class PathMapTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPathPoints(THREE_POINTS);
    }

    /// Index of world `point` in the embedded output map.
    size_t embeddedIndex(const Eigen::Vector2f &point) const
    {
        const murosim::grid::Map &map = sim->getGridMap();
        std::vector<float> scratch(MAP_SIZE * MAP_SIZE);
        const Eigen::Vector2i offset = murosimrl::util::grid::embedMapCentered(
            map, MAP_SIZE, murosimrl::util::grid::gridValueLut(WITHOUT_BUFFER), scratch.data());
        Eigen::Vector2i cell;
        map.fromWorldIntoMap(point, cell);
        return static_cast<size_t>(cell.y() + offset.y()) * MAP_SIZE + static_cast<size_t>(cell.x() + offset.x());
    }
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(PathMapTest, ShapeIsSimulationsByChannelByMap)
{
    const PathMap observation(MAP_SIZE, WITHOUT_BUFFER);

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 1, MAP_SIZE, MAP_SIZE}));
    EXPECT_EQ(observation.name(), "path_map");
    EXPECT_EQ(observation.mapSize(), MAP_SIZE);
    EXPECT_FALSE(observation.withBuffer());
}

// ============================================================================
// Path Tests
// ============================================================================

TEST_F(PathMapTest, PathCellsReadMinusOne)
{
    const PathMap observation(MAP_SIZE, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(output.size(), MAP_SIZE * MAP_SIZE);
    for (const Eigen::Vector2f &point : THREE_POINTS)
    {
        EXPECT_FLOAT_EQ(output[embeddedIndex(point)], PATH) << "point " << point.transpose();
    }
    EXPECT_EQ(std::count(output.begin(), output.end(), PATH), 3); // exactly the three path cells
}

TEST_F(PathMapTest, EverythingElseIsTheOccupancyMap)
{
    const PathMap observation(MAP_SIZE, WITHOUT_BUFFER);
    const OccupancyMap reference(MAP_SIZE, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());
    const std::vector<float> occupancy = reference(sims());

    ASSERT_EQ(output.size(), occupancy.size());
    for (size_t i = 0; i < output.size(); i++)
    {
        if (output[i] != PATH)
        {
            ASSERT_FLOAT_EQ(output[i], occupancy[i]) << "cell " << i;
        }
    }
}

TEST_F(PathMapTest, WithoutPathEqualsTheOccupancyMap)
{
    robot->setPathPoints({});
    const PathMap observation(MAP_SIZE, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());

    EXPECT_EQ(output, OccupancyMap(MAP_SIZE, WITHOUT_BUFFER)(sims()));
}

TEST_F(PathMapTest, PathPointsOutsideTheMapAreSkipped)
{
    robot->setPathPoints({Eigen::Vector2f(50.0f, 50.0f), Eigen::Vector2f(0.0f, 0.0f)});
    const PathMap observation(MAP_SIZE, WITHOUT_BUFFER);

    const std::vector<float> output = observation(sims());

    EXPECT_EQ(std::count(output.begin(), output.end(), PATH), 1);
}

// ============================================================================
// Contract Tests
// ============================================================================

TEST_F(PathMapTest, MapSizeZeroThrows)
{
    EXPECT_THROW(PathMap(0, WITHOUT_BUFFER), std::runtime_error);
}

TEST_F(PathMapTest, MapSmallerThanGridThrows)
{
    const PathMap observation(sim->getGridMap().sizeX() - 1, WITHOUT_BUFFER);

    EXPECT_THROW(observation(sims()), std::runtime_error);
}
