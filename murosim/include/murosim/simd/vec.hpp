// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SIMD_VEC_HPP_
#define MUROSIM_SIMD_VEC_HPP_

#include "declarations.hpp"

#include <cmath>
#include <cstddef>

namespace murosim::simd
{

/// Thin wrapper around the float vector type of the build-selected SIMD
/// backend. Exactly one of the five `Vec` definitions below is compiled,
/// picked purely from compiler feature macros set by -march (checked in this
/// order): AVX-512 (16 lanes), AVX (8 lanes), SSE2 (4 lanes), ARM NEON
/// (4 lanes), or a 1-lane scalar float fallback when none apply.
///
/// The collision and reduction kernels (src/collision/array_kernels.cpp,
/// src/simd/util.cpp) are written once against this common interface;
/// supporting a new backend means adding one more `Vec` block here instead of
/// re-implementing every kernel. All five definitions expose the same members,
/// documented once here rather than on each block:
///
/// - `Type`: the native vector type of the backend (e.g. `__m256`).
/// - `Mask`: the type produced by comparisons and consumed by `blend`.
/// - `LANES`: number of packed floats per `Type`.
/// - `set1(value)`: broadcast a scalar to all lanes.
/// - `zero()`: an all-zero vector.
/// - `load(ptr)` / `loadu(ptr)`: load `LANES` floats from an aligned /
///   possibly-unaligned pointer.
/// - `store(ptr, value)`: store `LANES` floats to an aligned pointer.
/// - `div(a, b)`, `sqrt(value)`, `min(a, b)`: lane-wise arithmetic.
/// - `Type` also supports the native arithmetic operators (`+`, `-`, `*`,
///   unary `-`) on every backend.
/// - `lessEqual(a, b)`: lane-wise ordered `<=`; comparisons are ordered, so a
///   lane holding NaN always compares false. This is what keeps the NaN
///   padding of ArrayFloat from ever reporting a spurious collision.
/// - `maskAnd(a, b)`, `maskOr(a, b)`: combine masks lane-wise.
/// - `anyTrue(mask)`: true if any lane of mask is set.
/// - `blend(fallback, value, mask)`: lane-wise select, `value` where `mask` is
///   set, `fallback` otherwise.
/// - `horizontalSum(value)`: sum of all lanes as a single float.

#if defined(__AVX512F__)

/// AVX-512 backend: 16 lanes, `Type = __m512`, `Mask = __mmask16`.
struct Vec
{
    using Type = __m512;
    using Mask = __mmask16;

    static constexpr size_t LANES = 16;

    static Type set1(const float value) { return _mm512_set1_ps(value); }
    static Type zero() { return _mm512_setzero_ps(); }
    static Type load(const float *ptr) { return _mm512_load_ps(ptr); }
    static Type loadu(const float *ptr) { return _mm512_loadu_ps(ptr); }
    static void store(float *ptr, const Type value) { _mm512_store_ps(ptr, value); }

    static Type div(const Type a, const Type b) { return _mm512_div_ps(a, b); }
    static Type sqrt(const Type value) { return _mm512_sqrt_ps(value); }
    static Type min(const Type a, const Type b) { return _mm512_min_ps(a, b); }

    static Mask lessEqual(const Type a, const Type b) { return _mm512_cmp_ps_mask(a, b, _CMP_LE_OQ); }
    static Mask maskAnd(const Mask a, const Mask b) { return _mm512_kand(a, b); }
    static Mask maskOr(const Mask a, const Mask b) { return _mm512_kor(a, b); }
    static bool anyTrue(const Mask mask) { return _mm512_mask2int(mask) != 0; }
    static Type blend(const Type fallback, const Type value, const Mask mask) { return _mm512_mask_blend_ps(mask, fallback, value); }

    static float horizontalSum(const Type value) { return _mm512_reduce_add_ps(value); }
};

#elif defined(__AVX__)

/// AVX backend: 8 lanes, `Type = __m256`, `Mask = __m256` (comparisons return
/// a float vector of all-ones/all-zero lanes).
struct Vec
{
    using Type = __m256;
    using Mask = __m256;

    static constexpr size_t LANES = 8;

    static Type set1(const float value) { return _mm256_set1_ps(value); }
    static Type zero() { return _mm256_setzero_ps(); }
    static Type load(const float *ptr) { return _mm256_load_ps(ptr); }
    static Type loadu(const float *ptr) { return _mm256_loadu_ps(ptr); }
    static void store(float *ptr, const Type value) { _mm256_store_ps(ptr, value); }

    static Type div(const Type a, const Type b) { return _mm256_div_ps(a, b); }
    static Type sqrt(const Type value) { return _mm256_sqrt_ps(value); }
    static Type min(const Type a, const Type b) { return _mm256_min_ps(a, b); }

    static Mask lessEqual(const Type a, const Type b) { return _mm256_cmp_ps(a, b, _CMP_LE_OQ); }
    static Mask maskAnd(const Mask a, const Mask b) { return _mm256_and_ps(a, b); }
    static Mask maskOr(const Mask a, const Mask b) { return _mm256_or_ps(a, b); }
    static bool anyTrue(const Mask mask) { return _mm256_movemask_ps(mask) != 0; }
    static Type blend(const Type fallback, const Type value, const Mask mask) { return _mm256_blendv_ps(fallback, value, mask); }

    static float horizontalSum(const Type value)
    {
        __m128 low = _mm256_castps256_ps128(value);
        const __m128 high = _mm256_extractf128_ps(value, 1);
        low = _mm_add_ps(low, high);
        low = _mm_hadd_ps(low, low);
        low = _mm_hadd_ps(low, low);
        return _mm_cvtss_f32(low);
    }
};

#elif defined(__SSE2__)

/// SSE2 backend: 4 lanes, `Type = __m128`, `Mask = __m128`. `blend` uses
/// declarations::_mm_blendv_ps, which emulates SSE4.1's blendv on plain SSE2.
struct Vec
{
    using Type = __m128;
    using Mask = __m128;

    static constexpr size_t LANES = 4;

    static Type set1(const float value) { return _mm_set1_ps(value); }
    static Type zero() { return _mm_setzero_ps(); }
    static Type load(const float *ptr) { return _mm_load_ps(ptr); }
    static Type loadu(const float *ptr) { return _mm_loadu_ps(ptr); }
    static void store(float *ptr, const Type value) { _mm_store_ps(ptr, value); }

    static Type div(const Type a, const Type b) { return _mm_div_ps(a, b); }
    static Type sqrt(const Type value) { return _mm_sqrt_ps(value); }
    static Type min(const Type a, const Type b) { return _mm_min_ps(a, b); }

    static Mask lessEqual(const Type a, const Type b) { return _mm_cmple_ps(a, b); }
    static Mask maskAnd(const Mask a, const Mask b) { return _mm_and_ps(a, b); }
    static Mask maskOr(const Mask a, const Mask b) { return _mm_or_ps(a, b); }
    static bool anyTrue(const Mask mask) { return _mm_movemask_ps(mask) != 0; }
    static Type blend(const Type fallback, const Type value, const Mask mask) { return declarations::_mm_blendv_ps(fallback, value, mask); }

    static float horizontalSum(const Type value)
    {
        // SSE2-safe reduction (no hadd): lanes 0+2 / 1+3, then 0+1
        __m128 sum = _mm_add_ps(value, _mm_movehl_ps(value, value));
        sum = _mm_add_ss(sum, _mm_shuffle_ps(sum, sum, 0x55));
        return _mm_cvtss_f32(sum);
    }
};

#elif defined(__ARM_NEON)

/// ARM NEON backend: 4 lanes, `Type = float32x4_t`, `Mask = uint32x4_t`.
struct Vec
{
    using Type = float32x4_t;
    using Mask = uint32x4_t;

    static constexpr size_t LANES = 4;

    static Type set1(const float value) { return vdupq_n_f32(value); }
    static Type zero() { return vdupq_n_f32(0.0f); }
    static Type load(const float *ptr) { return vld1q_f32(ptr); }
    static Type loadu(const float *ptr) { return vld1q_f32(ptr); }
    static void store(float *ptr, const Type value) { vst1q_f32(ptr, value); }

    static Type div(const Type a, const Type b) { return vdivq_f32(a, b); }
    static Type sqrt(const Type value) { return vsqrtq_f32(value); }
    static Type min(const Type a, const Type b) { return vminq_f32(a, b); }

    static Mask lessEqual(const Type a, const Type b) { return vcleq_f32(a, b); }
    static Mask maskAnd(const Mask a, const Mask b) { return vandq_u32(a, b); }
    static Mask maskOr(const Mask a, const Mask b) { return vorrq_u32(a, b); }
    static bool anyTrue(const Mask mask) { return vmaxvq_u32(mask) != 0; }
    static Type blend(const Type fallback, const Type value, const Mask mask) { return vbslq_f32(mask, value, fallback); }

    static float horizontalSum(const Type value) { return vaddvq_f32(value); }
};

#else

/// Scalar fallback backend used when no SIMD instruction set was detected:
/// 1 lane, `Type = float`, `Mask = bool`.
struct Vec
{
    using Type = float;
    using Mask = bool;

    static constexpr size_t LANES = 1;

    static Type set1(const float value) { return value; }
    static Type zero() { return 0.0f; }
    static Type load(const float *ptr) { return *ptr; }
    static Type loadu(const float *ptr) { return *ptr; }
    static void store(float *ptr, const Type value) { *ptr = value; }

    static Type div(const Type a, const Type b) { return a / b; }
    static Type sqrt(const Type value) { return std::sqrt(value); }
    static Type min(const Type a, const Type b) { return (a < b) ? a : b; }

    static Mask lessEqual(const Type a, const Type b) { return a <= b; }
    static Mask maskAnd(const Mask a, const Mask b) { return a && b; }
    static Mask maskOr(const Mask a, const Mask b) { return a || b; }
    static bool anyTrue(const Mask mask) { return mask; }
    static Type blend(const Type fallback, const Type value, const Mask mask) { return mask ? value : fallback; }

    static float horizontalSum(const Type value) { return value; }
};

#endif

} // namespace murosim::simd

#endif // MUROSIM_SIMD_VEC_HPP_
