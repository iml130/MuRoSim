// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Reductions written once against murosim::simd::Vec; the build-selected
// SIMD backend is chosen in vec.hpp.

#include "murosim/simd/util.hpp"
#include "murosim/simd/vec.hpp"

#include <limits>

namespace murosim::simd::util {

float sum(const float * const array, const size_t size)
{
    size_t index = 0;
    float sum = 0.0f;

    if (Vec::LANES <= size)
    {
        const size_t size_simd = size - Vec::LANES;
        Vec::Type accumulator = Vec::zero();

        for (; index <= size_simd; index += Vec::LANES)
        {
            accumulator = accumulator + Vec::loadu(array + index);
        }

        sum += Vec::horizontalSum(accumulator);
    }

    for (; index < size; ++index)
    {
        sum += array[index];
    }

    return sum;
}

float variance(const float * const array, const size_t size, const float mean)
{
    if (size <= 1)
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    size_t index = 0;
    float variance = 0.0f;

    if (Vec::LANES <= size)
    {
        const size_t size_simd = size - Vec::LANES;

        const Vec::Type mean_v = Vec::set1(mean);
        Vec::Type accumulator = Vec::zero();

        for (; index <= size_simd; index += Vec::LANES)
        {
            const Vec::Type diff = Vec::loadu(array + index) - mean_v;
            accumulator = accumulator + diff * diff;
        }

        variance += Vec::horizontalSum(accumulator);
    }

    for (; index < size; ++index)
    {
        const float diff = array[index] - mean;
        variance += diff * diff;
    }

    return variance / static_cast<float>(size - 1);
}

} // namespace murosim::simd::util
