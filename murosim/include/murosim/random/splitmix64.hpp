// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_RANDOM_SPLITMIX64_HPP_
#define MUROSIM_RANDOM_SPLITMIX64_HPP_

#include <cstdint>
#include <limits>

namespace murosim::random
{

// SplitMix64 (Steele/Lea/Vigna): a fast 64-bit generator with a 64-bit state.
// Primary use here is to expand a single seed into the larger state of other
// generators, but it is a perfectly usable standalone PRNG (passes BigCrush).
class SplitMix64
{
public:
    using result_type = uint64_t;

private:
    uint64_t state_;

public:
    /// Seeds deterministically; any 64-bit value is a valid seed (unlike
    /// xoshiro256, an all-zero state is not a problem here).
    explicit SplitMix64(const uint64_t seed) : state_(seed) {}

    /// Advances the state and returns the next 64-bit output.
    inline result_type next()
    {
        uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    /// Equivalent to next(); satisfies UniformRandomBitGenerator.
    inline result_type operator()()
    {
        return next();
    }

    /// Minimum value next()/operator() can return (0).
    static constexpr result_type min()
    {
        return std::numeric_limits<result_type>::min();
    }

    /// Maximum value next()/operator() can return.
    static constexpr result_type max()
    {
        return std::numeric_limits<result_type>::max();
    }
};

} // namespace murosim::random

#endif // MUROSIM_RANDOM_SPLITMIX64_HPP_
