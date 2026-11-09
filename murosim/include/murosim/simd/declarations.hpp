// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SIMD_DECLARATIONS_HPP_
#define MUROSIM_SIMD_DECLARATIONS_HPP_

#include <cstddef>

/// Backend selection mirrors murosim/simd/vec.hpp: exactly one of the
/// AVX-512, AVX, SSE2, NEON or scalar branches below is compiled, chosen from
/// the same -march feature macros. See vec.hpp for the rationale; this header
/// only carries what murosim::simd::util and ArrayFloat need before Vec
/// itself is visible: the lane count of the selected float vector type
/// (SIMD_FLOAT_SIZE, used to size and align ArrayFloat's storage) and, on
/// SSE2 without SSE4.1, a portable blendv_ps fallback.

#if defined(__AVX512F__)

#include <immintrin.h>

namespace murosim::simd::declarations
{
    /// Number of packed floats in one AVX-512 vector register (16).
    constexpr size_t SIMD_FLOAT_SIZE = sizeof(__m512) / sizeof(float);
} // namespace murosim::simd::declarations

#elif defined(__AVX__)

#include <immintrin.h>

namespace murosim::simd::declarations
{
    /// Number of packed floats in one AVX vector register (8).
    constexpr size_t SIMD_FLOAT_SIZE = sizeof(__m256) / sizeof(float);

} // namespace murosim::simd::declarations

#elif defined(__SSE2__)

#include <immintrin.h>

namespace murosim::simd::declarations
{
    /// Number of packed floats in one SSE vector register (4).
    constexpr size_t SIMD_FLOAT_SIZE = sizeof(__m128) / sizeof(float);

    /// Lane-wise select between a and b according to mask, matching the
    /// semantics of SSE4.1's _mm_blendv_ps. Dispatches to the intrinsic when
    /// SSE4.1 is available; otherwise emulates it with SSE2-only bitwise ops
    /// so Vec::blend works on plain SSE2 targets too.
    inline __m128 _mm_blendv_ps(const __m128 a, const __m128 b, const __m128 mask)
    {
    #if defined(__SSE4_1__)
        return ::_mm_blendv_ps(a,b,mask);
    #else
        const __m128 _mask = _mm_cmpneq_ps(mask, _mm_setzero_ps());
        return _mm_or_ps(_mm_andnot_ps(_mask, a), _mm_and_ps(b, _mask));
    #endif
    }
} // namespace murosim::simd::declarations

#elif defined(__ARM_NEON)

#include <arm_neon.h>

namespace murosim::simd::declarations
{
    /// Number of packed floats in one NEON vector register (4).
    constexpr size_t SIMD_FLOAT_SIZE = 4;

} // namespace murosim::simd::declarations

#else

namespace murosim::simd::declarations
{
    /// Scalar fallback: one float per "vector".
    constexpr size_t SIMD_FLOAT_SIZE = 1;
}

#endif

#endif // MUROSIM_SIMD_DECLARATIONS_HPP_
