// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/util/array3d.hpp"
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <vector>

namespace
{
using Array = murosimproc::util::Array3d<uint8_t>;

constexpr size_t SIZE_X = 2;
constexpr size_t SIZE_Y = 2;
constexpr size_t SIZE_Z = 2;
const std::vector<uint8_t> DATA = {1, 2, 3, 4, 5, 6, 7, 8}; // x fastest, then y, then z
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(Array3dConstructor, DefaultConstructorIsEmpty)
{
    const Array array;

    EXPECT_EQ(array.size(), static_cast<size_t>(0));
    EXPECT_EQ(array.sizeX(), static_cast<size_t>(0));
    EXPECT_EQ(array.sizeY(), static_cast<size_t>(0));
    EXPECT_EQ(array.sizeZ(), static_cast<size_t>(0));
    EXPECT_TRUE(array.empty());
}

TEST(Array3dConstructor, SizeConstructorValueInitialises)
{
    const Array array(SIZE_X, SIZE_Y, SIZE_Z);

    EXPECT_EQ(array.size(), SIZE_X * SIZE_Y * SIZE_Z);
    EXPECT_EQ(array.sizeZ(), SIZE_Z);
    EXPECT_FALSE(array.empty());
    for (size_t i = 0; i < array.size(); i++)
    {
        EXPECT_EQ(array[i], 0) << "index " << i;
    }
}

TEST(Array3dConstructor, FillConstructorSetsEveryElement)
{
    const Array array(SIZE_X, SIZE_Y, SIZE_Z, 7);

    for (size_t i = 0; i < array.size(); i++)
    {
        EXPECT_EQ(array[i], 7) << "index " << i;
    }
}

TEST(Array3dConstructor, DataConstructorCopiesXFastest)
{
    const Array array(SIZE_X, SIZE_Y, SIZE_Z, DATA);

    EXPECT_EQ(array(0, 0, 0), 1);
    EXPECT_EQ(array(1, 0, 0), 2); // next x
    EXPECT_EQ(array(0, 1, 0), 3); // next y
    EXPECT_EQ(array(0, 0, 1), 5); // next z
    EXPECT_EQ(array(1, 1, 1), 8);
}

TEST(Array3dConstructor, DataSizeMismatchThrows)
{
    EXPECT_THROW(Array(SIZE_X, SIZE_Y, SIZE_Z, std::vector<uint8_t>(9, 7)), std::invalid_argument);
    EXPECT_THROW(Array(SIZE_X, SIZE_Y, SIZE_Z, std::vector<uint8_t>(7, 7)), std::invalid_argument);
}

// ============================================================================
// Test Fixture
// ============================================================================

class Array3dTest : public ::testing::Test
{
protected:
    Array array{SIZE_X, SIZE_Y, SIZE_Z, DATA};
};

// ============================================================================
// Access Tests
// ============================================================================

TEST_F(Array3dTest, IndexAndCoordinatesAgree)
{
    for (size_t z = 0; z < SIZE_Z; z++)
    {
        for (size_t y = 0; y < SIZE_Y; y++)
        {
            for (size_t x = 0; x < SIZE_X; x++)
            {
                EXPECT_EQ(array.toIndex(x, y, z), x + y * SIZE_X + z * SIZE_X * SIZE_Y) << x << ", " << y << ", " << z;
                EXPECT_EQ(array(x, y, z), array[array.toIndex(x, y, z)]) << x << ", " << y << ", " << z;
            }
        }
    }
}

TEST_F(Array3dTest, WritesThroughBothAccessors)
{
    array(1, 1, 1) = 50;
    array[0] = 60;

    EXPECT_EQ(array[7], 50);
    EXPECT_EQ(array(0, 0, 0), 60);
}

TEST_F(Array3dTest, FillOverwritesEverything)
{
    array.fill(9);

    for (size_t i = 0; i < array.size(); i++)
    {
        EXPECT_EQ(array[i], 9) << "index " << i;
    }
}

TEST_F(Array3dTest, SubArrayCopiesTheRegion)
{
    const Array sub = array.subArray3d(1, 0, 0, 1, 2, 2); // x = 1 only, all y and z

    ASSERT_EQ(sub.size(), static_cast<size_t>(4));
    EXPECT_EQ(sub(0, 0, 0), 2);
    EXPECT_EQ(sub(0, 1, 0), 4);
    EXPECT_EQ(sub(0, 0, 1), 6);
    EXPECT_EQ(sub(0, 1, 1), 8);
}

// ============================================================================
// Comparison Tests
// ============================================================================

TEST_F(Array3dTest, EqualArraysCompareEqual)
{
    EXPECT_TRUE(array == Array(SIZE_X, SIZE_Y, SIZE_Z, DATA));
}

TEST_F(Array3dTest, DifferentValuesOrShapesCompareUnequal)
{
    Array other(SIZE_X, SIZE_Y, SIZE_Z, DATA);
    other(1, 1, 1) = 0;

    EXPECT_FALSE(array == other);
    EXPECT_FALSE(array == Array(SIZE_X * SIZE_Y * SIZE_Z, 1, 1, DATA)); // same data, other shape
}

TEST_F(Array3dTest, HashFollowsEquality)
{
    const std::hash<Array> hash;
    Array other(SIZE_X, SIZE_Y, SIZE_Z, DATA);
    other[0] = 0;

    EXPECT_EQ(hash(array), hash(Array(SIZE_X, SIZE_Y, SIZE_Z, DATA)));
    EXPECT_NE(hash(array), hash(other));
}
