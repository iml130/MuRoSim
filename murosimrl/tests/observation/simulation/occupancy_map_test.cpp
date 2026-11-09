// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/simulation/occupancy_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "fixtures.hpp"
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::simulation::Simulation;
using murosimrl::observation::simulation::OccupancyMap;

constexpr size_t MAP_SIZE = 128; // larger than the 101 x 101 fixture map
constexpr bool WITHOUT_BUFFER = false;
constexpr bool WITH_BUFFER = true;
constexpr float FREE = 0.0f;
constexpr float OCCUPIED = 1.0f;
constexpr float BUFFER = 0.5f;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class OccupancyMapTest : public ::testing::Test
{
protected:
    /// The embedded map the util layer produces, the single owner of the geometry.
    std::vector<float> referenceMap(const bool with_buffer) const
    {
        std::vector<float> output(MAP_SIZE * MAP_SIZE);
        murosimrl::util::grid::embedMapCentered(sim->getGridMap(), MAP_SIZE,
                                                murosimrl::util::grid::gridValueLut(with_buffer), output.data());
        return output;
    }

    std::shared_ptr<Simulation> sim = fixtures::makeSimulation("occupancy");
};

// ============================================================================
// Shape Tests
// ============================================================================

TEST_F(OccupancyMapTest, ShapeIsSimulationsByChannelByMap)
{
    const OccupancyMap observation(MAP_SIZE, WITHOUT_BUFFER);

    // stack_size is fixed to 1: the size-1 dimension is the channel axis
    EXPECT_EQ(observation.shape({sim}), (std::vector<size_t>{1, 1, MAP_SIZE, MAP_SIZE}));
    EXPECT_EQ(observation.shape({sim, sim}), (std::vector<size_t>{2, 1, MAP_SIZE, MAP_SIZE}));
    EXPECT_EQ(observation.name(), "occupancy_map");
    EXPECT_EQ(observation.mapSize(), MAP_SIZE);
    EXPECT_FALSE(observation.withBuffer());
}

// ============================================================================
// Map Tests
// ============================================================================

TEST_F(OccupancyMapTest, EmbeddedMapMatchesTheUtilEmbedding)
{
    const OccupancyMap observation(MAP_SIZE, WITH_BUFFER);

    const std::vector<float> output = observation({sim});

    const std::vector<float> expected = referenceMap(WITH_BUFFER);
    ASSERT_EQ(output.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
    {
        ASSERT_FLOAT_EQ(output[i], expected[i]) << "cell " << i;
    }
}

TEST_F(OccupancyMapTest, PaddingReadsOccupiedAndCentreReadsFree)
{
    const OccupancyMap observation(MAP_SIZE, WITHOUT_BUFFER);

    const std::vector<float> output = observation({sim});

    EXPECT_FLOAT_EQ(output[0], OCCUPIED);
    EXPECT_FLOAT_EQ(output[MAP_SIZE * MAP_SIZE - 1], OCCUPIED);
    EXPECT_FLOAT_EQ(output[(MAP_SIZE / 2) * MAP_SIZE + MAP_SIZE / 2], FREE); // the world origin
}

TEST_F(OccupancyMapTest, WithBufferTogglesTheBufferBand)
{
    const std::vector<float> buffered = OccupancyMap(MAP_SIZE, WITH_BUFFER)({sim});
    const std::vector<float> plain = OccupancyMap(MAP_SIZE, WITHOUT_BUFFER)({sim});

    EXPECT_GT(std::count(buffered.begin(), buffered.end(), BUFFER), 0);
    EXPECT_EQ(std::count(plain.begin(), plain.end(), BUFFER), 0);
    // the band reads free without the buffer: the same number of occupied cells either way
    EXPECT_EQ(std::count(buffered.begin(), buffered.end(), OCCUPIED), std::count(plain.begin(), plain.end(), OCCUPIED));
}

TEST_F(OccupancyMapTest, BatchHoldsOneMapPerSimulation)
{
    const std::shared_ptr<Simulation> small = fixtures::makeSimulation("small", fixtures::WORLD_SIZE / 2.0f);
    const OccupancyMap observation(MAP_SIZE, WITHOUT_BUFFER);

    const std::vector<float> output = observation({sim, small});

    const size_t step = MAP_SIZE * MAP_SIZE;
    ASSERT_EQ(output.size(), 2 * step);
    // the smaller world leaves more padding, so more occupied cells
    EXPECT_LT(std::count(output.begin(), output.begin() + step, OCCUPIED),
              std::count(output.begin() + step, output.end(), OCCUPIED));
}

// ============================================================================
// Contract Tests
// ============================================================================

TEST_F(OccupancyMapTest, MapSizeZeroThrows)
{
    EXPECT_THROW(OccupancyMap(0, WITHOUT_BUFFER), std::runtime_error);
}

TEST_F(OccupancyMapTest, MapSmallerThanGridThrows)
{
    const OccupancyMap observation(sim->getGridMap().sizeX() - 1, WITHOUT_BUFFER);

    EXPECT_THROW(observation({sim}), std::runtime_error);
}

TEST_F(OccupancyMapTest, EmptyInputThrows)
{
    const OccupancyMap observation(MAP_SIZE, WITHOUT_BUFFER);

    EXPECT_THROW(observation(fixtures::SimulationList()), std::invalid_argument);
}
