// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_RANDOM_XOSHIRO256_HPP_
#define MUROSIM_RANDOM_XOSHIRO256_HPP_

#include "murosim/random/splitmix64.hpp"

#include <cstdint>
#include <limits>

// xoshiro256 family by David Blackman and Sebastiano Vigna (vigna@acm.org),
// released into the public domain (https://prng.di.unimi.it/).
//
// 256-bit state (uint64_t[4]), 64-bit output, period 2^256 - 1. Pure shifts,
// rotates, xors and adds: fast, branch-free and platform-independent (no
// undefined behaviour, no floating point), so the same seed yields the same
// sequence everywhere -- which is what keeps experiments reproducible.
//
// Three output scramblers are provided as a speed/quality knob:
//   - Plus     (s0 + s3)               : fastest. Lowest 3 bits have low linear
//                                         complexity; irrelevant for std
//                                         distributions, which use the high bits.
//   - PlusPlus (rotl(s0 + s3, 23) + s0): robust for raw integer bits too.
//   - StarStar (rotl(s1 * 5, 7) * 9)   : alternative high-quality scrambler.
// The state transition is identical for all three.

namespace murosim::random
{

namespace detail
{

/// Rotates a 64-bit value left by k bits (0 < k < 64).
inline uint64_t rotl(const uint64_t x, const int k)
{
    return (x << k) | (x >> (64 - k));
}

} // namespace detail

// Shared state, seeding and transition for the xoshiro256 variants.
/// Not copy/move-restricted: copying duplicates the 256-bit state, producing
/// an independent generator that continues the same sequence from that point.
class Xoshiro256Base
{
protected:
    uint64_t state_[4];
    // Implementation base, not an interface: no virtual functions (the
    // generator stays a plain 32-byte value with inlined calls), so deleting
    // through a base pointer is forbidden at compile time instead.
    ~Xoshiro256Base() = default;

    // The seed is run through SplitMix64 to fill the full 256-bit state. This
    // decorrelates nearby seeds and guarantees a non-zero state (xoshiro must
    // not be all-zero).
    explicit Xoshiro256Base(const uint64_t seed)
    {
        SplitMix64 seeder(seed);
        state_[0] = seeder();
        state_[1] = seeder();
        state_[2] = seeder();
        state_[3] = seeder();
    }

    // Canonical xoshiro256 state transition.
    inline void advance()
    {
        const uint64_t t = state_[1] << 17;
        state_[2] ^= state_[0];
        state_[3] ^= state_[1];
        state_[1] ^= state_[2];
        state_[0] ^= state_[3];
        state_[2] ^= t;
        state_[3] = detail::rotl(state_[3], 45);
    }

public:
    using result_type = uint64_t;

    /// Minimum value a variant's next()/operator() can return (0).
    static constexpr result_type min()
    {
        return std::numeric_limits<result_type>::min();
    }

    /// Maximum value a variant's next()/operator() can return.
    static constexpr result_type max()
    {
        return std::numeric_limits<result_type>::max();
    }

    // Equivalent to 2^128 calls to next(); used to generate non-overlapping
    // subsequences for parallel streams from a single seed.
    void jump()
    {
        static const uint64_t JUMP[] = {
            0x180ec6d33cfd0abaULL, 0xd5a61266f0c9392cULL,
            0xa9582618e03fc9aaULL, 0x39abdc4529b1661cULL};

        uint64_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;
        for (const uint64_t bits : JUMP)
        {
            for (int b = 0; b < 64; ++b)
            {
                if (bits & (UINT64_C(1) << b))
                {
                    s0 ^= state_[0];
                    s1 ^= state_[1];
                    s2 ^= state_[2];
                    s3 ^= state_[3];
                }
                advance();
            }
        }
        state_[0] = s0;
        state_[1] = s1;
        state_[2] = s2;
        state_[3] = s3;
    }
};

// Fastest variant. High bits are full quality; preferred default for
// floating-point and ranged-integer use via std distributions.
class Xoshiro256Plus : public Xoshiro256Base
{
public:
    /// Seeds deterministically via SplitMix64; the same seed always yields the same sequence.
    explicit Xoshiro256Plus(const uint64_t seed) : Xoshiro256Base(seed) {}

    /// Advances the state and returns the next 64-bit output.
    inline result_type next()
    {
        const result_type result = state_[0] + state_[3];
        advance();
        return result;
    }

    /// Equivalent to next(); satisfies UniformRandomBitGenerator.
    inline result_type operator()()
    {
        return next();
    }
};

// Robust all-purpose scrambler; safe even when consuming the raw low bits.
class Xoshiro256PlusPlus : public Xoshiro256Base
{
public:
    /// Seeds deterministically via SplitMix64; the same seed always yields the same sequence.
    explicit Xoshiro256PlusPlus(const uint64_t seed) : Xoshiro256Base(seed) {}

    /// Advances the state and returns the next 64-bit output.
    inline result_type next()
    {
        const result_type result = detail::rotl(state_[0] + state_[3], 23) + state_[0];
        advance();
        return result;
    }

    /// Equivalent to next(); satisfies UniformRandomBitGenerator.
    inline result_type operator()()
    {
        return next();
    }
};

// Alternative high-quality scrambler.
class Xoshiro256StarStar : public Xoshiro256Base
{
public:
    /// Seeds deterministically via SplitMix64; the same seed always yields the same sequence.
    explicit Xoshiro256StarStar(const uint64_t seed) : Xoshiro256Base(seed) {}

    /// Advances the state and returns the next 64-bit output.
    inline result_type next()
    {
        const result_type result = detail::rotl(state_[1] * 5, 7) * 9;
        advance();
        return result;
    }

    /// Equivalent to next(); satisfies UniformRandomBitGenerator.
    inline result_type operator()()
    {
        return next();
    }
};

} // namespace murosim::random

#endif // MUROSIM_RANDOM_XOSHIRO256_HPP_
