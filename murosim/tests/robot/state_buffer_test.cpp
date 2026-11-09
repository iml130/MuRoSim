// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/robot/state_buffer.hpp"
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::actuator::HolonomicSimple;
using murosim::navigation::Node;
using murosim::robot::State;
using murosim::robot::StateBuffer;
using murosim::sensor::Lidar;

// The buffer allocates at least this many slots and rounds every requested
// size up to the next power of two
constexpr size_t MIN_CAPACITY = 32;
constexpr size_t OVERFILL = 40; // more states than MIN_CAPACITY
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class StateBufferTest : public ::testing::Test
{
protected:
    /// State whose pose x coordinate is `x`, so states can be told apart.
    State stateAt(const float x) const
    {
        return State(actuator, lidars, murosim::geometry::primitives::pose(x, 0.0f, 0.0f), goal);
    }

    static float xOf(const State &state)
    {
        return state.getPose().translation().x();
    }

    HolonomicSimple actuator;
    std::vector<Lidar> lidars = {Lidar(Eigen::Affine2f::Identity(), PI, 10, 0.0f, 10.0f)};
    Node goal{0.0f, 0.0f, 1.0f};
    StateBuffer buffer;
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(StateBufferTest, DefaultConstructorIsEmptyWithMinimumCapacity)
{
    EXPECT_EQ(buffer.size(), static_cast<size_t>(0));
    EXPECT_EQ(buffer.getMaxBufferSize(), MIN_CAPACITY);
}

// ============================================================================
// Add State Tests
// ============================================================================

TEST_F(StateBufferTest, AddSingleState)
{
    buffer.addState(stateAt(1.0f));

    ASSERT_EQ(buffer.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(xOf(buffer[0]), 1.0f);
}

TEST_F(StateBufferTest, AddMultipleStates)
{
    buffer.addState(stateAt(1.0f));
    buffer.addState(stateAt(2.0f));
    buffer.addState(stateAt(3.0f));

    EXPECT_EQ(buffer.size(), static_cast<size_t>(3));
}

TEST_F(StateBufferTest, NextStateFillsSlotInPlace)
{
    buffer.nextState() = stateAt(7.0f);

    ASSERT_EQ(buffer.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(xOf(buffer[0]), 7.0f);
}

// ============================================================================
// Access Tests
// ============================================================================

TEST_F(StateBufferTest, IndexZeroIsMostRecent)
{
    buffer.addState(stateAt(1.0f));
    buffer.addState(stateAt(2.0f));
    buffer.addState(stateAt(3.0f));

    ASSERT_EQ(buffer.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(xOf(buffer[0]), 3.0f); // newest
    EXPECT_FLOAT_EQ(xOf(buffer[1]), 2.0f);
    EXPECT_FLOAT_EQ(xOf(buffer[2]), 1.0f); // oldest
}

// ============================================================================
// Capacity Tests
// ============================================================================

TEST_F(StateBufferTest, SetBufferSizeBelowMinimumUsesMinimum)
{
    buffer.setBufferSize(4);

    EXPECT_EQ(buffer.getMaxBufferSize(), MIN_CAPACITY);
}

TEST_F(StateBufferTest, SetBufferSizeRoundsUpToPowerOfTwo)
{
    buffer.setBufferSize(100);

    EXPECT_EQ(buffer.getMaxBufferSize(), static_cast<size_t>(128));
}

TEST_F(StateBufferTest, SetBufferSizeKeepsExactPowerOfTwo)
{
    buffer.setBufferSize(64);

    EXPECT_EQ(buffer.getMaxBufferSize(), static_cast<size_t>(64));
}

TEST_F(StateBufferTest, SetBufferSizeClearsHistory)
{
    buffer.addState(stateAt(1.0f));
    buffer.addState(stateAt(2.0f));

    buffer.setBufferSize(64);

    EXPECT_EQ(buffer.size(), static_cast<size_t>(0));
}

// ============================================================================
// Ring Buffer Tests
// ============================================================================

TEST_F(StateBufferTest, SizeSaturatesAtCapacity)
{
    for (size_t i = 0; i < OVERFILL; i++)
    {
        buffer.addState(stateAt(static_cast<float>(i)));
    }

    EXPECT_EQ(buffer.size(), MIN_CAPACITY);
}

TEST_F(StateBufferTest, OverflowKeepsMostRecentStates)
{
    for (size_t i = 0; i < OVERFILL; i++)
    {
        buffer.addState(stateAt(static_cast<float>(i)));
    }

    // Index i holds the state added i steps ago: x = OVERFILL - 1 - i
    ASSERT_EQ(buffer.size(), MIN_CAPACITY);
    for (size_t i = 0; i < MIN_CAPACITY; i++)
    {
        ASSERT_FLOAT_EQ(xOf(buffer[i]), static_cast<float>(OVERFILL - 1 - i)) << "index " << i;
    }
}

TEST_F(StateBufferTest, OverflowWrapsAroundRepeatedly)
{
    for (size_t i = 0; i < 3 * MIN_CAPACITY; i++)
    {
        buffer.addState(stateAt(static_cast<float>(i)));
    }

    EXPECT_EQ(buffer.size(), MIN_CAPACITY);
    EXPECT_FLOAT_EQ(xOf(buffer[0]), static_cast<float>(3 * MIN_CAPACITY - 1));
    EXPECT_FLOAT_EQ(xOf(buffer[MIN_CAPACITY - 1]), static_cast<float>(2 * MIN_CAPACITY));
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(StateBufferTest, ResetEmptyBuffer)
{
    buffer.reset();

    EXPECT_EQ(buffer.size(), static_cast<size_t>(0));
}

TEST_F(StateBufferTest, ResetFilledBuffer)
{
    buffer.addState(stateAt(1.0f));
    buffer.addState(stateAt(2.0f));
    buffer.addState(stateAt(3.0f));

    buffer.reset();

    EXPECT_EQ(buffer.size(), static_cast<size_t>(0));
    EXPECT_EQ(buffer.getMaxBufferSize(), MIN_CAPACITY); // capacity kept
}

TEST_F(StateBufferTest, AddAfterResetStartsFresh)
{
    buffer.addState(stateAt(1.0f));
    buffer.addState(stateAt(2.0f));
    buffer.reset();

    buffer.addState(stateAt(5.0f));

    ASSERT_EQ(buffer.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(xOf(buffer[0]), 5.0f);
}

TEST_F(StateBufferTest, MultipleResets)
{
    buffer.addState(stateAt(1.0f));
    buffer.reset();
    buffer.addState(stateAt(2.0f));
    buffer.reset();

    buffer.addState(stateAt(3.0f));

    ASSERT_EQ(buffer.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(xOf(buffer[0]), 3.0f);
}

TEST_F(StateBufferTest, ResetAfterOverflowRestartsAtSlotZero)
{
    for (size_t i = 0; i < OVERFILL; i++)
    {
        buffer.addState(stateAt(static_cast<float>(i)));
    }
    buffer.reset();

    buffer.addState(stateAt(-1.0f));
    buffer.addState(stateAt(-2.0f));

    ASSERT_EQ(buffer.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(xOf(buffer[0]), -2.0f);
    EXPECT_FLOAT_EQ(xOf(buffer[1]), -1.0f);
}
