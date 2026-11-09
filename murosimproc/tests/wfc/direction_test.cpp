// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/wfc/direction.hpp"
#include <cstddef>

namespace
{
using murosimproc::wfc::DIRECTION;
} // namespace

// ============================================================================
// Offset Tests
// ============================================================================

TEST(DirectionOffsets, FourCardinalDirections)
{
    EXPECT_EQ(DIRECTION::SIZE, static_cast<size_t>(4));
    EXPECT_EQ(DIRECTION::X.size(), DIRECTION::SIZE);
    EXPECT_EQ(DIRECTION::Y.size(), DIRECTION::SIZE);
}

TEST(DirectionOffsets, EnumValuesIndexTheTables)
{
    EXPECT_EQ(static_cast<size_t>(DIRECTION::NORTH), static_cast<size_t>(0));
    EXPECT_EQ(static_cast<size_t>(DIRECTION::EAST), static_cast<size_t>(1));
    EXPECT_EQ(static_cast<size_t>(DIRECTION::SOUTH), static_cast<size_t>(2));
    EXPECT_EQ(static_cast<size_t>(DIRECTION::WEST), static_cast<size_t>(3));
}

// y grows downwards (image rows), so NORTH is -1 in y
TEST(DirectionOffsets, OffsetsFollowImageCoordinates)
{
    EXPECT_EQ(DIRECTION::X[DIRECTION::NORTH], 0L);
    EXPECT_EQ(DIRECTION::Y[DIRECTION::NORTH], -1L);
    EXPECT_EQ(DIRECTION::X[DIRECTION::EAST], 1L);
    EXPECT_EQ(DIRECTION::Y[DIRECTION::EAST], 0L);
    EXPECT_EQ(DIRECTION::X[DIRECTION::SOUTH], 0L);
    EXPECT_EQ(DIRECTION::Y[DIRECTION::SOUTH], 1L);
    EXPECT_EQ(DIRECTION::X[DIRECTION::WEST], -1L);
    EXPECT_EQ(DIRECTION::Y[DIRECTION::WEST], 0L);
}

TEST(DirectionOffsets, OppositeDirectionsCancel)
{
    EXPECT_EQ(DIRECTION::X[DIRECTION::NORTH] + DIRECTION::X[DIRECTION::SOUTH], 0L);
    EXPECT_EQ(DIRECTION::Y[DIRECTION::NORTH] + DIRECTION::Y[DIRECTION::SOUTH], 0L);
    EXPECT_EQ(DIRECTION::X[DIRECTION::EAST] + DIRECTION::X[DIRECTION::WEST], 0L);
    EXPECT_EQ(DIRECTION::Y[DIRECTION::EAST] + DIRECTION::Y[DIRECTION::WEST], 0L);
}

TEST(DirectionOffsets, EveryDirectionIsAUnitStep)
{
    for (size_t i = 0; i < DIRECTION::SIZE; i++)
    {
        EXPECT_EQ(DIRECTION::X[i] * DIRECTION::X[i] + DIRECTION::Y[i] * DIRECTION::Y[i], 1L) << "direction " << i;
    }
}

// ============================================================================
// Name Tests
// ============================================================================

TEST(DirectionNames, EveryDirectionHasItsName)
{
    EXPECT_EQ(DIRECTION::STRING(DIRECTION::NORTH), "NORTH");
    EXPECT_EQ(DIRECTION::STRING(DIRECTION::EAST), "EAST");
    EXPECT_EQ(DIRECTION::STRING(DIRECTION::SOUTH), "SOUTH");
    EXPECT_EQ(DIRECTION::STRING(DIRECTION::WEST), "WEST");
}
