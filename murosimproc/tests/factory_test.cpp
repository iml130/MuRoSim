// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/factory.hpp"
#include "murosimproc/generators/intersection_generator.hpp"
#include "murosimproc/generators/line_generator.hpp"
#include "murosimproc/generators/wfc_generator.hpp"
#include "murosimproc/wfc/tiles.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using murosimproc::Factory;
using murosimproc::generators::Generator;
using murosimproc::generators::IntersectionGenerator;
using murosimproc::generators::LineGenerator;
using murosimproc::generators::RandomRange;
using murosimproc::generators::WfcGenerator;
using murosimproc::obstacle::ShapeDistribution;
using murosimproc::wfc::Tiles;
using murosim::io::DataContainer;
using murosim::simulation::Simulation;

// The factory seeds every world from a random device, so nothing here
// depends on which worlds come out, only on how many and from whom.
constexpr float NODE_RADIUS = 0.25f;
constexpr float WORLD_SIZE = 10.0f;
constexpr size_t COUNT = 12;
constexpr size_t THREADS = 4;
constexpr size_t RETRIES = 3;
constexpr float GRID_SCALE = 0.1f;
constexpr float BUFFER_SCALE = 0.2f;
constexpr double DOMINANT_WEIGHT = 1000.0; // one draw in 1001 picks the other generator

std::shared_ptr<Generator> lineGenerator()
{
    return std::make_shared<LineGenerator>(2, NODE_RADIUS, WORLD_SIZE, RandomRange(0.6f, 1.0f), 0.5f, true);
}

std::shared_ptr<Generator> intersectionGenerator()
{
    return std::make_shared<IntersectionGenerator>(4, NODE_RADIUS, WORLD_SIZE, RandomRange(1.0f, 2.0f), 0.5f);
}

/// Never reaches its node count: the tile set is nothing but walls.
std::shared_ptr<Generator> failingGenerator()
{
    const Tiles walls(fixtures::makeImage({"###", "###", "###"}), 3);
    return std::make_shared<WfcGenerator>(2, NODE_RADIUS, WORLD_SIZE, 1.0f, walls, RandomRange(0.3f, 0.5f), 0.0f,
                                          ShapeDistribution());
}

bool startsWith(const std::string &text, const std::string &prefix)
{
    return text.compare(0, prefix.size(), prefix) == 0;
}

size_t countNamed(const std::vector<DataContainer> &containers, const std::string &prefix)
{
    size_t count = 0;
    for (const DataContainer &container : containers)
    {
        if (startsWith(container.getName(), prefix))
        {
            count++;
        }
    }
    return count;
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(FactoryConstructor, Defaults)
{
    const Factory factory;

    EXPECT_GE(factory.getNumThreads(), static_cast<size_t>(1)); // hardware concurrency
    EXPECT_EQ(factory.getMaxRetries(), static_cast<size_t>(100));
}

TEST(FactoryConstructor, ExplicitThreadsAndRetries)
{
    const Factory factory(THREADS, RETRIES);

    EXPECT_EQ(factory.getNumThreads(), THREADS);
    EXPECT_EQ(factory.getMaxRetries(), RETRIES);
}

TEST(FactoryConstructor, ZeroRetriesThrows)
{
    EXPECT_THROW(Factory(THREADS, 0), std::invalid_argument);
}

// ============================================================================
// Add Generator Tests
// ============================================================================

TEST(FactoryAddGenerator, NullGeneratorThrows)
{
    Factory factory;

    EXPECT_THROW(factory.addGenerator(nullptr), std::invalid_argument);
    EXPECT_THROW(factory.addGenerator(nullptr, 1.0), std::invalid_argument);
}

TEST(FactoryAddGenerator, NonPositiveWeightThrows)
{
    Factory factory;

    EXPECT_THROW(factory.addGenerator(lineGenerator(), 0.0), std::invalid_argument);
    EXPECT_THROW(factory.addGenerator(lineGenerator(), -1.0), std::invalid_argument);
}

TEST(FactoryAddGenerator, EmptyFactoryCannotGenerate)
{
    const Factory factory;

    EXPECT_THROW(factory.generateDataContainers(1), std::logic_error);
    EXPECT_THROW(factory.generateSimulations(1, GRID_SCALE, BUFFER_SCALE), std::logic_error);
}

// ============================================================================
// Data Container Tests
// ============================================================================

TEST(FactoryDataContainers, EveryContainerComesFromTheGenerator)
{
    Factory factory(THREADS);
    factory.addGenerator(lineGenerator());

    const std::vector<DataContainer> containers = factory.generateDataContainers(COUNT);

    ASSERT_EQ(containers.size(), COUNT);
    for (size_t i = 0; i < containers.size(); i++)
    {
        EXPECT_TRUE(startsWith(containers[i].getName(), "line_")) << "container " << i;
        EXPECT_TRUE(containers[i].isValid()) << "container " << i;
        EXPECT_EQ(containers[i].getNodes().size(), static_cast<size_t>(2)) << "container " << i;
    }
}

TEST(FactoryDataContainers, ZeroCountGivesNothing)
{
    Factory factory(THREADS);
    factory.addGenerator(lineGenerator());

    EXPECT_TRUE(factory.generateDataContainers(0).empty());
}

TEST(FactoryDataContainers, SingleThreadWorks)
{
    Factory factory(1);
    factory.addGenerator(lineGenerator());

    EXPECT_EQ(factory.generateDataContainers(COUNT).size(), COUNT);
}

TEST(FactoryDataContainers, EqualWeightsUseBothGenerators)
{
    Factory factory(THREADS);
    factory.addGenerator(lineGenerator());
    factory.addGenerator(intersectionGenerator());

    const std::vector<DataContainer> containers = factory.generateDataContainers(4 * COUNT);

    ASSERT_EQ(containers.size(), 4 * COUNT);
    EXPECT_EQ(countNamed(containers, "line_") + countNamed(containers, "intersection_"), 4 * COUNT);
    EXPECT_GT(countNamed(containers, "line_"), static_cast<size_t>(0));
    EXPECT_GT(countNamed(containers, "intersection_"), static_cast<size_t>(0));
}

// With weights 1000 : 1 the chance of even one intersection world in 48
// draws is below 5 %, and of two below 0.1 %.
TEST(FactoryDataContainers, DominantWeightAlmostAlwaysWins)
{
    Factory factory(THREADS);
    factory.addGenerator(lineGenerator(), DOMINANT_WEIGHT);
    factory.addGenerator(intersectionGenerator(), 1.0);

    const std::vector<DataContainer> containers = factory.generateDataContainers(4 * COUNT);

    EXPECT_GE(countNamed(containers, "line_"), 4 * COUNT - 1);
}

TEST(FactoryDataContainers, FailedAttemptsFallThroughToAnotherDraw)
{
    Factory factory(THREADS);
    factory.addGenerator(failingGenerator());
    factory.addGenerator(lineGenerator());

    const std::vector<DataContainer> containers = factory.generateDataContainers(COUNT);

    ASSERT_EQ(containers.size(), COUNT);
    EXPECT_EQ(countNamed(containers, "line_"), COUNT);
}

TEST(FactoryDataContainers, OnlyFailingGeneratorsThrowAfterTheRetries)
{
    Factory factory(THREADS, RETRIES);
    factory.addGenerator(failingGenerator());

    EXPECT_THROW(factory.generateDataContainers(2), std::runtime_error);
}

// ============================================================================
// Simulation Tests
// ============================================================================

TEST(FactorySimulations, EverySimulationIsBuiltFromAGeneratedWorld)
{
    Factory factory(THREADS);
    factory.addGenerator(intersectionGenerator());

    const std::vector<std::shared_ptr<Simulation>> simulations =
        factory.generateSimulations(COUNT, GRID_SCALE, BUFFER_SCALE);

    ASSERT_EQ(simulations.size(), COUNT);
    for (size_t i = 0; i < simulations.size(); i++)
    {
        ASSERT_NE(simulations[i], nullptr) << "simulation " << i;
        EXPECT_TRUE(startsWith(simulations[i]->getName(), "intersection_")) << "simulation " << i;
        EXPECT_EQ(simulations[i]->getNodes().getNodes().size(), static_cast<size_t>(4)) << "simulation " << i;
    }
}

TEST(FactorySimulations, SimulationsAreDistinctObjects)
{
    Factory factory(THREADS);
    factory.addGenerator(lineGenerator());

    const std::vector<std::shared_ptr<Simulation>> simulations =
        factory.generateSimulations(2, GRID_SCALE, BUFFER_SCALE);

    ASSERT_EQ(simulations.size(), static_cast<size_t>(2));
    EXPECT_NE(simulations[0], simulations[1]);
}

TEST(FactorySimulations, OnlyFailingGeneratorsThrow)
{
    Factory factory(THREADS, RETRIES);
    factory.addGenerator(failingGenerator());

    EXPECT_THROW(factory.generateSimulations(2, GRID_SCALE, BUFFER_SCALE), std::runtime_error);
}
