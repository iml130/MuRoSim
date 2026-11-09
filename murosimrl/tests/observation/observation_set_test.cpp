// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/observation_set.hpp"
#include "murosimrl/observation/robot/command_velocity.hpp"
#include "murosimrl/observation/robot/distance_to_goal.hpp"
#include "murosimrl/observation/robot/lidar.hpp"
#include "murosimrl/observation/robot/orientation_to_goal.hpp"
#include "murosimrl/observation/robot/velocity.hpp"
#include "murosim/threading/thread_pool.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using murosim::threading::ThreadPool;
using murosimrl::observation::ObservationRobot;
using murosimrl::observation::ObservationSet;
using fixtures::LIDAR_RANGE_MAX;
using fixtures::LIDAR_RAYS;

constexpr size_t NUM_ROBOTS = 13; // not divisible by typical chunk counts

size_t totalSize(const std::vector<size_t> &shape)
{
    size_t total = shape.empty() ? 0 : 1;
    for (const size_t dim : shape)
    {
        total *= dim;
    }
    return total;
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// Thirteen robots spread out with distinct poses, goals and commands, so
// every value of every observation differs per robot, stepped a few times
// so stacked observations see distinct states.
class ObservationSetTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        for (size_t i = 0; i < NUM_ROBOTS; i++)
        {
            robots.push_back(fixtures::holonomicRobot("robot_" + std::to_string(i), true));
            sim->addRobot(robots.back());
        }
        sim->reset();
        for (size_t i = 0; i < NUM_ROBOTS; i++)
        {
            const float f = static_cast<float>(i);
            robots[i]->setPose(-4.0f + 0.6f * f, -4.0f + 1.7f * static_cast<float>(i % 5), 0.3f * f);
            robots[i]->setGoal(4.0f - 0.5f * f, 3.0f - 1.9f * static_cast<float>(i % 4), 0.5f);
            robots[i]->setActuatorCommandVelocity(0.1f * static_cast<float>(i % 3) - 0.1f,
                                                  0.05f * static_cast<float>(i % 2), 0.1f);
        }
        for (int i = 0; i < 3; i++)
        {
            sim->simulate();
        }
    }

    std::shared_ptr<murosim::simulation::Simulation> sim = fixtures::makeSimulation("observation_set");
    fixtures::RobotList robots;
};

// ============================================================================
// Bit Exactness Tests
// ============================================================================

// The pooled set must produce bit-identical results to serial evaluation of
// each operator, including the Lidar ops whose internal threading is
// bypassed on the set path.
TEST_F(ObservationSetTest, MatchesSerialBitExact)
{
    const std::vector<std::shared_ptr<ObservationRobot>> ops = {
        std::make_shared<murosimrl::observation::robot::Lidar>(0, LIDAR_RANGE_MAX, true, 2, 1),
        std::make_shared<murosimrl::observation::robot::Lidar>(0, 0.0f, false, 3, 4),
        std::make_shared<murosimrl::observation::robot::DistanceToGoal>(30.0f, 2),
        std::make_shared<murosimrl::observation::robot::Velocity>(false, 2),
        std::make_shared<murosimrl::observation::robot::OrientationToGoal>(0.25f, 1),
        std::make_shared<murosimrl::observation::robot::CommandVelocity>(true, 0),
    };
    ObservationSet set;
    std::vector<std::vector<float>> expected;
    for (const std::shared_ptr<ObservationRobot> &op : ops)
    {
        set.add(op);
        expected.emplace_back((*op)(robots));
    }
    ASSERT_EQ(set.size(), ops.size());

    for (const size_t threads : {1u, 4u})
    {
        ThreadPool pool(threads);
        std::vector<std::vector<float>> actual;
        std::vector<float *> outputs;
        for (size_t i = 0; i < ops.size(); i++)
        {
            actual.emplace_back(totalSize(ops[i]->shape(robots)));
            outputs.emplace_back(actual.back().data());
        }

        set.compute(pool, robots, outputs);

        for (size_t i = 0; i < ops.size(); i++)
        {
            ASSERT_EQ(actual[i].size(), expected[i].size()) << "op " << i;
            for (size_t k = 0; k < expected[i].size(); k++)
            {
                // exact equality: parallel chunking must not change a bit
                ASSERT_EQ(actual[i][k], expected[i][k]) << "op " << i << " element " << k << " threads " << threads;
            }
        }
    }
}

// Every min_chunk_size, one chunk total (16, 1000), a few chunks (4) or
// per-robot chunks (1), must stay bit-identical to serial evaluation.
TEST_F(ObservationSetTest, MinChunkSizeVariantsBitExact)
{
    const std::shared_ptr<ObservationRobot> op =
        std::make_shared<murosimrl::observation::robot::Lidar>(0, LIDAR_RANGE_MAX, true, 2, 1);
    const std::vector<float> expected = (*op)(robots);

    for (const size_t min_chunk : {1u, 4u, 16u, 1000u})
    {
        ObservationSet set(min_chunk);
        ASSERT_EQ(set.minChunkSize(), min_chunk);
        set.add(op);
        ThreadPool pool(4);
        std::vector<float> actual(expected.size());
        std::vector<float *> outputs = {actual.data()};

        set.compute(pool, robots, outputs);

        for (size_t k = 0; k < expected.size(); k++)
        {
            ASSERT_EQ(actual[k], expected[k]) << "element " << k << " min_chunk " << min_chunk;
        }
    }
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(ObservationSetTest, MinChunkSizeZeroIsClampedToOne)
{
    EXPECT_EQ(ObservationSet(0).minChunkSize(), static_cast<size_t>(1));
    EXPECT_EQ(ObservationSet().minChunkSize(), static_cast<size_t>(16)); // default
}

TEST_F(ObservationSetTest, OperatorsAndShapesFollowAddOrder)
{
    const std::shared_ptr<ObservationRobot> velocity =
        std::make_shared<murosimrl::observation::robot::Velocity>(false, 1);
    const std::shared_ptr<ObservationRobot> distance =
        std::make_shared<murosimrl::observation::robot::DistanceToGoal>(0.0f, 1);
    ObservationSet set;

    set.add(velocity);
    set.add(distance);

    ASSERT_EQ(set.operators().size(), static_cast<size_t>(2));
    EXPECT_EQ(set.operators()[0], velocity);
    EXPECT_EQ(set.operators()[1], distance);
    // a stack size of 1 still carries the stack dimension
    EXPECT_EQ(set.shapes(robots), (std::vector<std::vector<size_t>>{{NUM_ROBOTS, 1, 3}, {NUM_ROBOTS, 1, 1}}));
}

TEST_F(ObservationSetTest, AddingNullThrows)
{
    ObservationSet set;

    EXPECT_THROW(set.add(nullptr), std::invalid_argument);
}

// ============================================================================
// Contract Tests
// ============================================================================

// A failing validate() (a lidar index the robots do not have) must surface
// as an exception from compute(), before any job runs.
TEST_F(ObservationSetTest, ValidateFailurePropagates)
{
    ObservationSet set;
    set.add(std::make_shared<murosimrl::observation::robot::Lidar>(1, LIDAR_RANGE_MAX, true, 1, 1));
    ThreadPool pool(2);
    std::vector<float> buffer(NUM_ROBOTS * LIDAR_RAYS);
    std::vector<float *> outputs = {buffer.data()};

    EXPECT_THROW(set.compute(pool, robots, outputs), std::runtime_error);
}

TEST_F(ObservationSetTest, OutputCountMismatchThrows)
{
    ObservationSet set;
    set.add(std::make_shared<murosimrl::observation::robot::Velocity>(false, 1));
    ThreadPool pool(2);
    const std::vector<float *> outputs; // empty: one output missing

    EXPECT_THROW(set.compute(pool, robots, outputs), std::invalid_argument);
}

TEST_F(ObservationSetTest, NullOutputForNonEmptyObservationThrows)
{
    ObservationSet set;
    set.add(std::make_shared<murosimrl::observation::robot::Velocity>(false, 1));
    ThreadPool pool(2);
    const std::vector<float *> outputs = {nullptr};

    EXPECT_THROW(set.compute(pool, robots, outputs), std::invalid_argument);
}
