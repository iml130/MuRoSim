// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_RANDOM_RANDOM_GENERATOR_HPP_
#define MUROSIM_RANDOM_RANDOM_GENERATOR_HPP_

#include "murosim/random/xoshiro256.hpp"

#include <limits>
#include <random>

namespace murosim::random
{

/// UniformRandomBitGenerator wrapping Xoshiro256Plus for use throughout the
/// simulator (robot placement, sensor noise, world generation) and directly
/// with `<random>` distributions. Not thread-safe: each thread/simulation
/// instance that needs independent, reproducible streams should hold its own
/// RandomGenerator, typically spawned via createRandomGenerator().
class RandomGenerator
{
public:
    using engine = murosim::random::Xoshiro256Plus;
    using result_type = engine::result_type;

private:
    result_type seed_;
    engine engine_;

public:
    // Seeds from std::random_device. random_device yields 32-bit values on most
    // platforms, so two draws are combined to fill the full 64-bit seed.
    RandomGenerator() :
        seed_((static_cast<result_type>(std::random_device{}()) << 32) ^
              static_cast<result_type>(std::random_device{}())),
        engine_(seed_)
    {
    }

    /// Seeds deterministically; the same seed always yields the same sequence.
    RandomGenerator(const result_type seed) :
        seed_(seed),
        engine_(seed)
    {
    }

    // The effective seed, useful for reproducing default-constructed runs.
    inline result_type seed() const
    {
        return seed_;
    }

    // Spawns a child generator seeded from this generator's stream. Advances
    // this generator by one step.
    inline RandomGenerator createRandomGenerator()
    {
        return RandomGenerator(engine_());
    }

    /// Draws the next 64-bit value; satisfies UniformRandomBitGenerator.
    inline result_type operator()()
    {
        return engine_();
    }

    /// Minimum value operator() can return (0).
    static constexpr result_type min()
    {
        return std::numeric_limits<result_type>::min();
    }

    /// Maximum value operator() can return.
    static constexpr result_type max()
    {
        return std::numeric_limits<result_type>::max();
    }
};

} // namespace murosim::random

#endif  // MUROSIM_RANDOM_RANDOM_GENERATOR_HPP_
