// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SIMD_UTIL_HPP_
#define MUROSIM_SIMD_UTIL_HPP_

#include <cstdlib>
#include <cstddef>
#include <new>
#include "declarations.hpp"
#include <exception>
#include <stdexcept>

namespace murosim::simd::util {

/// Sums size elements of array using the build-selected murosim::simd::Vec
/// backend for the bulk of the range, with a scalar tail for the remainder.
/// array must have at least size valid elements.
float sum(const float * const array, const size_t size);

/// Sample variance (sum of squared deviations from mean, divided by
/// size - 1) of size elements of array, using murosim::simd::Vec for the bulk
/// of the range with a scalar tail. Returns quiet NaN if size <= 1.
/// @param mean the precomputed mean of the same range (typically from sum()).
float variance(const float * const array, const size_t size, const float mean);

/// Allocates a buffer of size floats, aligned to and padded up to a multiple
/// of the SIMD lane width, so SIMD loads/stores never run past the
/// allocation. size == 0 returns nullptr and sets size_simd to 0.
/// @param size requested logical element count.
/// @param size_simd out parameter, set to the actual allocated element count
///        (a multiple of murosim::simd::declarations::SIMD_FLOAT_SIZE).
/// @return owning pointer to be released with std::free (see
///         murosim::simd::AlignedFloatDeleter); throws std::bad_alloc on
///         allocation failure.
inline float * allocAlignedMemoryFloat(const size_t size, size_t &size_simd)
{
    float * data  = nullptr;

    if (size == 0)
    {
        size_simd = 0;
    }
    else
    {
        size_simd = ((size / murosim::simd::declarations::SIMD_FLOAT_SIZE) + ((size % murosim::simd::declarations::SIMD_FLOAT_SIZE == 0)? 0: 1)) * murosim::simd::declarations::SIMD_FLOAT_SIZE;
        data = static_cast<float *>(std::aligned_alloc(sizeof(float) * murosim::simd::declarations::SIMD_FLOAT_SIZE, sizeof(float) * size_simd));
        if (data == nullptr)
        {
            throw std::bad_alloc();
        }
    }

    return data;
}

} // namespace murosim::simd::util

#endif // MUROSIM_SIMD_UTIL_HPP_
