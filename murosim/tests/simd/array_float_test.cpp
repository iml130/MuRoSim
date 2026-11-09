// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/simd/array_float.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace
{
using murosim::simd::ArrayFloat;

const std::vector<float> VALUES = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};

// Large enough to span many SIMD blocks and not a multiple of any lane width
// (4, 8 or 16), so the tail handling of the reductions is exercised
constexpr size_t BIG_SIZE = 4001;
constexpr size_t BIG_MIDDLE = 2000;

/// Array holding 0, 1, 2, ... n - 1.
ArrayFloat ramp(const size_t n)
{
    ArrayFloat array(n);
    for (size_t i = 0; i < n; i++)
    {
        array[i] = static_cast<float>(i);
    }
    return array;
}

/// Array alternating +1, -1, ... with the last element 0, so that mean and
/// variance are exactly 0 and 1 for odd n.
ArrayFloat alternating(const size_t n)
{
    ArrayFloat array(n);
    for (size_t i = 0; i < n; i++)
    {
        array[i] = (i % 2 == 0) ? 1.0f : -1.0f;
    }
    array[n - 1] = 0.0f;
    return array;
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(ArrayFloatConstructor, DefaultConstructorIsEmpty)
{
    const ArrayFloat array;

    EXPECT_EQ(array.size(), static_cast<size_t>(0));
    EXPECT_EQ(array.sizeAligned(), static_cast<size_t>(0));
    EXPECT_TRUE(array.empty());
}

TEST(ArrayFloatConstructor, SizeConstructorAllocatesAndFillsWithNan)
{
    const ArrayFloat array(10);

    EXPECT_EQ(array.size(), static_cast<size_t>(10));
    EXPECT_GE(array.sizeAligned(), static_cast<size_t>(10));
    EXPECT_FALSE(array.empty());
    for (size_t i = 0; i < array.sizeAligned(); i++)
    {
        EXPECT_TRUE(std::isnan(array.data()[i])) << "index " << i;
    }
}

TEST(ArrayFloatConstructor, VectorConstructorCopiesValues)
{
    const ArrayFloat array(VALUES);

    ASSERT_EQ(array.size(), VALUES.size());
    EXPECT_GE(array.sizeAligned(), VALUES.size());
    EXPECT_FALSE(array.empty());
    for (size_t i = 0; i < VALUES.size(); i++)
    {
        EXPECT_FLOAT_EQ(array[i], VALUES[i]) << "index " << i;
    }
}

TEST(ArrayFloatConstructor, VectorConstructorPadsWithNan)
{
    const ArrayFloat array(VALUES);

    for (size_t i = array.size(); i < array.sizeAligned(); i++)
    {
        EXPECT_TRUE(std::isnan(array.data()[i])) << "index " << i;
    }
}

// ============================================================================
// Constant Tests
// ============================================================================

TEST(ArrayFloatConstants, SimdSizeIsPositive)
{
    EXPECT_GT(ArrayFloat::SIMD_SIZE, static_cast<size_t>(0));
}

TEST(ArrayFloatConstants, QuietNanIsNan)
{
    EXPECT_TRUE(std::isnan(ArrayFloat::QUIET_NAN));
}

// ============================================================================
// Alignment Tests
// ============================================================================

TEST(ArrayFloatAlignment, SizeAlignedRoundsUpToSimdWidth)
{
    const size_t simd_size = ArrayFloat::SIMD_SIZE;

    EXPECT_EQ(ArrayFloat(simd_size).sizeAligned(), simd_size);
    EXPECT_EQ(ArrayFloat(simd_size - 1).sizeAligned(), simd_size);
    EXPECT_EQ(ArrayFloat(simd_size + 1).sizeAligned(), simd_size * 2);
    EXPECT_EQ(ArrayFloat(simd_size * 2).sizeAligned(), simd_size * 2);
}

// ============================================================================
// Test Fixture
// ============================================================================

class ArrayFloatTest : public ::testing::Test
{
protected:
    ArrayFloat array{VALUES};
};

// ============================================================================
// Element Access Tests
// ============================================================================

TEST_F(ArrayFloatTest, IndexReadsStoredValues)
{
    for (size_t i = 0; i < VALUES.size(); i++)
    {
        EXPECT_FLOAT_EQ(array[i], VALUES[i]) << "index " << i;
    }
}

TEST_F(ArrayFloatTest, IndexWritesValues)
{
    for (size_t i = 0; i < array.size(); i++)
    {
        array[i] = VALUES[i] + 1.0f;
    }

    for (size_t i = 0; i < array.size(); i++)
    {
        EXPECT_FLOAT_EQ(array[i], VALUES[i] + 1.0f) << "index " << i;
    }
}

TEST_F(ArrayFloatTest, DataAllowsWritingPadding)
{
    for (size_t i = array.size(); i < array.sizeAligned(); i++)
    {
        array.data()[i] = -10.0f;
    }

    for (size_t i = array.size(); i < array.sizeAligned(); i++)
    {
        EXPECT_FLOAT_EQ(array.data()[i], -10.0f) << "index " << i;
    }
}

TEST_F(ArrayFloatTest, ConstDataReadsValues)
{
    const ArrayFloat &const_array = array;
    const float *data = const_array.data();

    ASSERT_NE(data, nullptr);
    EXPECT_FLOAT_EQ(data[0], 1.0f);
    EXPECT_FLOAT_EQ(data[8], 9.0f);
}

TEST_F(ArrayFloatTest, ToVectorCopiesLogicalElements)
{
    const std::vector<float> vector = array.toVector();

    EXPECT_EQ(vector, VALUES); // padding is not copied
}

TEST_F(ArrayFloatTest, CopyToVectorOverwritesTarget)
{
    std::vector<float> vector(array.size(), 0.0f);

    array.copyToVector(vector);

    EXPECT_EQ(vector, VALUES);
}

TEST_F(ArrayFloatTest, EmptyIsFalseWithElements)
{
    EXPECT_FALSE(array.empty());
}

// ============================================================================
// Sub Array Tests
// ============================================================================

TEST_F(ArrayFloatTest, SubArrayFloatCopiesHalfOpenRange)
{
    const ArrayFloat sub = array.subArrayFloat(1, 3);

    ASSERT_EQ(sub.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(sub[0], 2.0f);
    EXPECT_FLOAT_EQ(sub[1], 3.0f);
    EXPECT_FALSE(sub.empty());
}

TEST_F(ArrayFloatTest, SubArrayFloatForward)
{
    const ArrayFloat sub = array.subArrayFloat(2, 6, false);

    ASSERT_EQ(sub.size(), static_cast<size_t>(4));
    EXPECT_FLOAT_EQ(sub[0], 3.0f);
    EXPECT_FLOAT_EQ(sub[1], 4.0f);
    EXPECT_FLOAT_EQ(sub[2], 5.0f);
    EXPECT_FLOAT_EQ(sub[3], 6.0f);
}

TEST_F(ArrayFloatTest, SubArrayFloatBackwardsReversesOrder)
{
    const ArrayFloat sub = array.subArrayFloat(2, 6, true);

    ASSERT_EQ(sub.size(), static_cast<size_t>(4));
    EXPECT_FLOAT_EQ(sub[0], 6.0f);
    EXPECT_FLOAT_EQ(sub[1], 5.0f);
    EXPECT_FLOAT_EQ(sub[2], 4.0f);
    EXPECT_FLOAT_EQ(sub[3], 3.0f);
}

TEST_F(ArrayFloatTest, SubArrayFloatSingleElement)
{
    const ArrayFloat sub = array.subArrayFloat(1, 2);

    ASSERT_EQ(sub.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(sub[0], 2.0f);
}

TEST_F(ArrayFloatTest, SubArrayFloatEmptyRange)
{
    const ArrayFloat sub = array.subArrayFloat(1, 1);

    EXPECT_EQ(sub.size(), static_cast<size_t>(0));
    EXPECT_TRUE(sub.empty());
}

// ============================================================================
// Min Tests
// ============================================================================

TEST_F(ArrayFloatTest, MinOfValues)
{
    EXPECT_FLOAT_EQ(array.min(), 1.0f);
}

TEST_F(ArrayFloatTest, MinFollowsChanges)
{
    array[0] = 0.0f;
    EXPECT_FLOAT_EQ(array.min(), 0.0f);

    array[8] = -9.0f;
    EXPECT_FLOAT_EQ(array.min(), -9.0f);
}

TEST(ArrayFloatMin, LargeArray)
{
    ArrayFloat big = ramp(BIG_SIZE);

    EXPECT_FLOAT_EQ(big.min(), 0.0f);

    big[BIG_MIDDLE] = -1.0f;
    EXPECT_FLOAT_EQ(big.min(), -1.0f);
}

TEST(ArrayFloatMin, EmptyIsNan)
{
    EXPECT_TRUE(std::isnan(ArrayFloat().min()));
}

// ============================================================================
// Max Tests
// ============================================================================

TEST_F(ArrayFloatTest, MaxOfValues)
{
    EXPECT_FLOAT_EQ(array.max(), 9.0f);
}

TEST_F(ArrayFloatTest, MaxFollowsChanges)
{
    array[8] = 10.0f;
    EXPECT_FLOAT_EQ(array.max(), 10.0f);

    array[8] = -10.0f;
    EXPECT_FLOAT_EQ(array.max(), 8.0f);
}

TEST(ArrayFloatMax, LargeArray)
{
    ArrayFloat big = ramp(BIG_SIZE);

    EXPECT_FLOAT_EQ(big.max(), static_cast<float>(BIG_SIZE - 1));

    big[BIG_MIDDLE] = 10000.0f;
    EXPECT_FLOAT_EQ(big.max(), 10000.0f);
}

TEST(ArrayFloatMax, EmptyIsNan)
{
    EXPECT_TRUE(std::isnan(ArrayFloat().max()));
}

// ============================================================================
// Sum Tests
// ============================================================================

TEST_F(ArrayFloatTest, SumOfValues)
{
    EXPECT_FLOAT_EQ(array.sum(), 45.0f);
}

TEST_F(ArrayFloatTest, SumFollowsChanges)
{
    array[8] += 10.0f;
    EXPECT_FLOAT_EQ(array.sum(), 55.0f);

    array[8] = -9.0f;
    EXPECT_FLOAT_EQ(array.sum(), 27.0f);
}

TEST(ArrayFloatSum, LargeArray)
{
    ArrayFloat big = ramp(BIG_SIZE);

    // 0 + 1 + ... + 4000 = 4000 * 4001 / 2, exact in float (below 2^24)
    const float expected = static_cast<float>((BIG_SIZE - 1) * BIG_SIZE / 2);
    EXPECT_FLOAT_EQ(big.sum(), expected);

    big[BIG_SIZE - 1] -= 10000.0f;
    EXPECT_FLOAT_EQ(big.sum(), expected - 10000.0f);
}

TEST(ArrayFloatSum, SingleElement)
{
    EXPECT_FLOAT_EQ(ArrayFloat(std::vector<float>{1.0f}).sum(), 1.0f);
}

TEST(ArrayFloatSum, EmptyIsNan)
{
    EXPECT_TRUE(std::isnan(ArrayFloat().sum()));
}

// ============================================================================
// Mean Tests
// ============================================================================

TEST_F(ArrayFloatTest, MeanOfValues)
{
    EXPECT_FLOAT_EQ(array.mean(), 5.0f);
}

TEST_F(ArrayFloatTest, MeanFollowsShiftAndNegation)
{
    for (size_t i = 0; i < array.size(); i++)
    {
        array[i] -= 1.0f;
    }
    EXPECT_FLOAT_EQ(array.mean(), 4.0f);

    for (size_t i = 0; i < array.size(); i++)
    {
        array[i] = -array[i];
    }
    EXPECT_FLOAT_EQ(array.mean(), -4.0f);
}

TEST(ArrayFloatMean, LargeAlternatingArrayIsZero)
{
    EXPECT_FLOAT_EQ(alternating(BIG_SIZE).mean(), 0.0f);
}

TEST(ArrayFloatMean, SingleElement)
{
    EXPECT_FLOAT_EQ(ArrayFloat(std::vector<float>{1.0f}).mean(), 1.0f);
}

TEST(ArrayFloatMean, EmptyIsNan)
{
    EXPECT_TRUE(std::isnan(ArrayFloat().mean()));
}

// ============================================================================
// Variance Tests
// ============================================================================

TEST_F(ArrayFloatTest, VarianceOfValuesIsSampleVariance)
{
    EXPECT_FLOAT_EQ(array.variance(), 7.5f); // sum of squared deviations 60 / (9 - 1)
}

TEST_F(ArrayFloatTest, VarianceIsInvariantUnderNegationAndShift)
{
    for (size_t i = 0; i < array.size(); i++)
    {
        array[i] = -array[i];
    }
    EXPECT_FLOAT_EQ(array.variance(), 7.5f);

    for (size_t i = 0; i < array.size(); i++)
    {
        array[i] += 1.0f;
    }
    EXPECT_FLOAT_EQ(array.variance(), 7.5f);
}

TEST(ArrayFloatVariance, ConstantArrayIsZero)
{
    ArrayFloat big(BIG_SIZE);
    for (size_t i = 0; i < big.size(); i++)
    {
        big[i] = 0.0f;
    }

    EXPECT_FLOAT_EQ(big.variance(), 0.0f);
}

TEST(ArrayFloatVariance, LargeAlternatingArrayIsOne)
{
    // 4000 squared deviations of 1 and one of 0, divided by 4000
    EXPECT_FLOAT_EQ(alternating(BIG_SIZE).variance(), 1.0f);
}

TEST(ArrayFloatVariance, SingleElementIsNan)
{
    EXPECT_TRUE(std::isnan(ArrayFloat(std::vector<float>{1.0f}).variance()));
}

TEST(ArrayFloatVariance, EmptyIsNan)
{
    EXPECT_TRUE(std::isnan(ArrayFloat().variance()));
}

// ============================================================================
// Copy Tests
// ============================================================================

TEST(ArrayFloatCopy, CopyConstructorIsDeep)
{
    const ArrayFloat original({1.0f, 2.0f, 3.0f, 4.0f, 5.0f});

    ArrayFloat copy(original);

    ASSERT_EQ(copy.size(), original.size());
    EXPECT_EQ(copy.sizeAligned(), original.sizeAligned());
    for (size_t i = 0; i < original.size(); i++)
    {
        EXPECT_FLOAT_EQ(copy[i], original[i]) << "index " << i;
    }

    copy[0] = 99.0f;
    EXPECT_FLOAT_EQ(original[0], 1.0f); // original untouched
}

TEST(ArrayFloatCopy, CopyAssignmentIsDeep)
{
    const ArrayFloat original({1.0f, 2.0f, 3.0f, 4.0f, 5.0f});
    ArrayFloat copy;

    copy = original;

    ASSERT_EQ(copy.size(), original.size());
    EXPECT_EQ(copy.sizeAligned(), original.sizeAligned());
    for (size_t i = 0; i < original.size(); i++)
    {
        EXPECT_FLOAT_EQ(copy[i], original[i]) << "index " << i;
    }

    copy[0] = 99.0f;
    EXPECT_FLOAT_EQ(original[0], 1.0f); // original untouched
}

TEST(ArrayFloatCopy, SelfAssignmentKeepsContents)
{
    ArrayFloat array({1.0f, 2.0f, 3.0f});
    ArrayFloat &alias = array; // avoids the self-assignment warning

    array = alias;

    ASSERT_EQ(array.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(array[0], 1.0f);
    EXPECT_FLOAT_EQ(array[1], 2.0f);
    EXPECT_FLOAT_EQ(array[2], 3.0f);
}

// Regression: size_ was only updated when the aligned capacities differed, so
// assigning between arrays with different logical sizes but equal aligned
// sizes kept the stale size and read NaN padding.
TEST(ArrayFloatCopy, AssignmentUpdatesSizeWhenAlignedSizeMatches)
{
    // 7 and 5 share the same aligned size for every supported SIMD width
    ArrayFloat a({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f});
    const ArrayFloat b({10.0f, 20.0f, 30.0f, 40.0f, 50.0f});
    ASSERT_EQ(a.sizeAligned(), b.sizeAligned());

    a = b;

    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < b.size(); i++)
    {
        EXPECT_FLOAT_EQ(a[i], b[i]) << "index " << i;
    }
}

// ============================================================================
// Move Semantics Tests
// ============================================================================

TEST(ArrayFloatMoveSemantics, MoveConstructorTakesBuffer)
{
    ArrayFloat original({1.0f, 2.0f, 3.0f, 4.0f, 5.0f});
    const size_t size_aligned = original.sizeAligned();

    const ArrayFloat moved(std::move(original));

    ASSERT_EQ(moved.size(), static_cast<size_t>(5));
    EXPECT_EQ(moved.sizeAligned(), size_aligned);
    EXPECT_FLOAT_EQ(moved[0], 1.0f);
    EXPECT_FLOAT_EQ(moved[4], 5.0f);
    EXPECT_EQ(original.size(), static_cast<size_t>(0)); // moved-from is empty
    EXPECT_EQ(original.sizeAligned(), static_cast<size_t>(0));
}

TEST(ArrayFloatMoveSemantics, MoveAssignmentTakesBuffer)
{
    ArrayFloat original({1.0f, 2.0f, 3.0f, 4.0f, 5.0f});
    const size_t size_aligned = original.sizeAligned();
    ArrayFloat moved;

    moved = std::move(original);

    ASSERT_EQ(moved.size(), static_cast<size_t>(5));
    EXPECT_EQ(moved.sizeAligned(), size_aligned);
    EXPECT_FLOAT_EQ(moved[0], 1.0f);
    EXPECT_FLOAT_EQ(moved[4], 5.0f);
    EXPECT_EQ(original.size(), static_cast<size_t>(0)); // moved-from is empty
    EXPECT_EQ(original.sizeAligned(), static_cast<size_t>(0));
}

TEST(ArrayFloatMoveSemantics, SelfMoveAssignmentKeepsContents)
{
    ArrayFloat array({1.0f, 2.0f, 3.0f});
    ArrayFloat &alias = array; // avoids the self-move warning

    array = std::move(alias);

    ASSERT_EQ(array.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(array[0], 1.0f);
    EXPECT_FLOAT_EQ(array[1], 2.0f);
    EXPECT_FLOAT_EQ(array[2], 3.0f);
}
