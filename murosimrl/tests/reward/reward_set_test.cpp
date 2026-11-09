// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/reward/reward_set.hpp"
#include "murosimrl/reward/robot/action_rate.hpp"
#include "murosimrl/reward/robot/delta_distance_to_goal.hpp"
#include "murosimrl/reward/robot/done.hpp"
#include "murosimrl/reward/robot/minimum_lidar_distance.hpp"
#include "murosimrl/reward/robot/on_goal.hpp"
#include "murosim/threading/thread_pool.hpp"
#include "fixtures.hpp"
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using murosim::threading::ThreadPool;
using murosimrl::reward::RewardRobot;
using murosimrl::reward::RewardSet;

constexpr size_t NUM_ROBOTS = 13; // not divisible by typical chunk counts
constexpr float SENTINEL = -12345.0f;

/// Sum of `ops` per robot, evaluated serially in the given order.
std::vector<float> serialSum(const std::vector<std::shared_ptr<RewardRobot>> &ops, const fixtures::RobotList &robots)
{
    std::vector<float> sum(robots.size(), 0.0f);
    for (const std::shared_ptr<RewardRobot> &op : ops)
    {
        const std::vector<float> value = (*op)(robots);
        for (size_t i = 0; i < robots.size(); i++)
        {
            sum[i] += value[i];
        }
    }
    return sum;
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class RewardSetTest : public ::testing::Test
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
        sim->simulate();
        sim->simulate();
    }

    std::shared_ptr<murosim::simulation::Simulation> sim = fixtures::makeSimulation("reward_set");
    fixtures::RobotList robots;
};

// ============================================================================
// Bit Exactness Tests
// ============================================================================

// The pooled sum must be bit-identical to summing the rewards serially in
// add() order (the same additions in the same order per robot).
TEST_F(RewardSetTest, MatchesSerialSumBitExact)
{
    const std::vector<std::shared_ptr<RewardRobot>> ops = {
        std::make_shared<murosimrl::reward::robot::DeltaDistanceToGoal>(2.0f, -4.0f),
        std::make_shared<murosimrl::reward::robot::OnGoal>(0.25f, 0.2f),
        std::make_shared<murosimrl::reward::robot::MinimumLidarDistance>(0.4f, -0.003f),
        std::make_shared<murosimrl::reward::robot::ActionRate>(0.15f, 0.075f, 0.2f, -0.0001f),
        std::make_shared<murosimrl::reward::robot::Done>(3.0f, -1.5f, -2.0f),
    };
    RewardSet set;
    for (const std::shared_ptr<RewardRobot> &op : ops)
    {
        set.add(op);
    }
    const std::vector<float> expected = serialSum(ops, robots);

    for (const size_t threads : {1u, 4u})
    {
        ThreadPool pool(threads);
        std::vector<float> actual(NUM_ROBOTS, SENTINEL);

        set.compute(pool, robots, actual.data());

        for (size_t i = 0; i < NUM_ROBOTS; i++)
        {
            // exact equality: parallel chunking must not change a bit
            ASSERT_EQ(actual[i], expected[i]) << "robot " << i << " threads " << threads;
        }
    }
}

// Every min_chunk_size, one chunk total (16, 1000), a few chunks (4) or
// per-robot chunks (1), must stay bit-identical to the serial sum.
TEST_F(RewardSetTest, MinChunkSizeVariantsBitExact)
{
    const std::vector<std::shared_ptr<RewardRobot>> ops = {
        std::make_shared<murosimrl::reward::robot::DeltaDistanceToGoal>(2.0f, -4.0f),
        std::make_shared<murosimrl::reward::robot::MinimumLidarDistance>(0.4f, -0.003f),
    };
    const std::vector<float> expected = serialSum(ops, robots);

    for (const size_t min_chunk : {1u, 4u, 16u, 1000u})
    {
        RewardSet set(min_chunk);
        ASSERT_EQ(set.minChunkSize(), min_chunk);
        set.add(ops[0]);
        set.add(ops[1]);
        ThreadPool pool(4);
        std::vector<float> actual(NUM_ROBOTS, SENTINEL);

        set.compute(pool, robots, actual.data());

        for (size_t i = 0; i < NUM_ROBOTS; i++)
        {
            ASSERT_EQ(actual[i], expected[i]) << "robot " << i << " min_chunk " << min_chunk;
        }
    }
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(RewardSetTest, MinChunkSizeZeroIsClampedToOne)
{
    EXPECT_EQ(RewardSet(0).minChunkSize(), static_cast<size_t>(1));
    EXPECT_EQ(RewardSet().minChunkSize(), static_cast<size_t>(16)); // default
}

TEST_F(RewardSetTest, OperatorsFollowAddOrder)
{
    const std::shared_ptr<RewardRobot> done = std::make_shared<murosimrl::reward::robot::Done>(1.0f, -1.0f, -1.0f);
    const std::shared_ptr<RewardRobot> on_goal = std::make_shared<murosimrl::reward::robot::OnGoal>(1.0f, 1.0f);
    RewardSet set;

    set.add(done);
    set.add(on_goal);

    ASSERT_EQ(set.size(), static_cast<size_t>(2));
    EXPECT_EQ(set.operators()[0], done);
    EXPECT_EQ(set.operators()[1], on_goal);
    EXPECT_EQ(set.shape(robots), (std::vector<size_t>{NUM_ROBOTS, 1}));
}

TEST_F(RewardSetTest, AddingNullThrows)
{
    RewardSet set;

    EXPECT_THROW(set.add(nullptr), std::invalid_argument);
}

TEST_F(RewardSetTest, EmptySetZeroFills)
{
    RewardSet set;
    ThreadPool pool(2);
    std::vector<float> output(NUM_ROBOTS, SENTINEL);

    set.compute(pool, robots, output.data());

    EXPECT_EQ(output, std::vector<float>(NUM_ROBOTS, 0.0f));
}
