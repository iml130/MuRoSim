// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/util/array2d.hpp"
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <vector>

namespace
{
using Array = murosimproc::util::Array2d<uint8_t>;

constexpr size_t WIDTH = 3;
constexpr size_t HEIGHT = 2;
const std::vector<uint8_t> DATA = {1, 2, 3, 4, 5, 6}; // row-major: row 0 = 1 2 3, row 1 = 4 5 6
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(Array2dConstructor, DefaultConstructorIsEmpty)
{
    const Array array;

    EXPECT_EQ(array.size(), static_cast<size_t>(0));
    EXPECT_EQ(array.sizeX(), static_cast<size_t>(0));
    EXPECT_EQ(array.sizeY(), static_cast<size_t>(0));
    EXPECT_TRUE(array.empty());
}

TEST(Array2dConstructor, SizeConstructorValueInitialises)
{
    const Array array(WIDTH, HEIGHT);

    EXPECT_EQ(array.size(), WIDTH * HEIGHT);
    EXPECT_EQ(array.sizeX(), WIDTH);
    EXPECT_EQ(array.sizeY(), HEIGHT);
    EXPECT_FALSE(array.empty());
    for (size_t i = 0; i < array.size(); i++)
    {
        EXPECT_EQ(array[i], 0) << "index " << i;
    }
}

TEST(Array2dConstructor, FillConstructorSetsEveryElement)
{
    const Array array(WIDTH, HEIGHT, 7);

    for (size_t i = 0; i < array.size(); i++)
    {
        EXPECT_EQ(array[i], 7) << "index " << i;
    }
}

TEST(Array2dConstructor, DataConstructorCopiesRowMajor)
{
    const Array array(WIDTH, HEIGHT, DATA);

    EXPECT_EQ(array[0], 1);
    EXPECT_EQ(array(0, 0), 1);
    EXPECT_EQ(array(2, 0), 3); // end of the first row
    EXPECT_EQ(array(0, 1), 4); // start of the second row
    EXPECT_EQ(array(2, 1), 6);
}

TEST(Array2dConstructor, DataSizeMismatchThrows)
{
    EXPECT_THROW(Array(WIDTH, HEIGHT, std::vector<uint8_t>(WIDTH * HEIGHT + 1, 7)), std::invalid_argument);
    EXPECT_THROW(Array(WIDTH, HEIGHT, std::vector<uint8_t>(WIDTH * HEIGHT - 1, 7)), std::invalid_argument);
}

// ============================================================================
// Test Fixture
// ============================================================================

class Array2dTest : public ::testing::Test
{
protected:
    Array array{WIDTH, HEIGHT, DATA};
};

// ============================================================================
// Access Tests
// ============================================================================

TEST_F(Array2dTest, IndexAndCoordinatesAgree)
{
    for (size_t y = 0; y < HEIGHT; y++)
    {
        for (size_t x = 0; x < WIDTH; x++)
        {
            EXPECT_EQ(array.toIndex(x, y), x + y * WIDTH) << "cell " << x << ", " << y;
            EXPECT_EQ(array(x, y), array[array.toIndex(x, y)]) << "cell " << x << ", " << y;
        }
    }
}

TEST_F(Array2dTest, WritesThroughBothAccessors)
{
    array(1, 1) = 50;
    array[0] = 60;

    EXPECT_EQ(array[4], 50);
    EXPECT_EQ(array(0, 0), 60);
}

TEST_F(Array2dTest, FillOverwritesEverything)
{
    array.fill(9);

    for (size_t i = 0; i < array.size(); i++)
    {
        EXPECT_EQ(array[i], 9) << "index " << i;
    }
}

TEST_F(Array2dTest, SubArrayCopiesTheRegion)
{
    const Array sub = array.subArray2d(1, 0, 2, 2); // columns 1..2, both rows

    ASSERT_EQ(sub.sizeX(), static_cast<size_t>(2));
    ASSERT_EQ(sub.sizeY(), static_cast<size_t>(2));
    EXPECT_EQ(sub(0, 0), 2);
    EXPECT_EQ(sub(1, 0), 3);
    EXPECT_EQ(sub(0, 1), 5);
    EXPECT_EQ(sub(1, 1), 6);
}

TEST_F(Array2dTest, SubArrayIsACopy)
{
    Array sub = array.subArray2d(0, 0, 1, 1);

    sub(0, 0) = 99;

    EXPECT_EQ(array(0, 0), 1);
}

// ============================================================================
// Comparison Tests
// ============================================================================

TEST_F(Array2dTest, EqualArraysCompareEqual)
{
    const Array same(WIDTH, HEIGHT, DATA);

    EXPECT_TRUE(array == same);
}

TEST_F(Array2dTest, DifferentValuesCompareUnequal)
{
    Array other(WIDTH, HEIGHT, DATA);
    other(2, 1) = 0;

    EXPECT_FALSE(array == other);
}

TEST_F(Array2dTest, DifferentShapesCompareUnequal)
{
    const Array transposed(HEIGHT, WIDTH, DATA); // same data, other shape

    EXPECT_FALSE(array == transposed);
}

TEST_F(Array2dTest, HashFollowsEquality)
{
    const std::hash<Array> hash;
    const Array same(WIDTH, HEIGHT, DATA);
    Array other(WIDTH, HEIGHT, DATA);
    other[0] = 0;

    EXPECT_EQ(hash(array), hash(same));
    EXPECT_NE(hash(array), hash(other));
}
