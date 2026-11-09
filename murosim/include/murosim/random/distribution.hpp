// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_RANDOM_DISTRIBUTION_HPP_
#define MUROSIM_RANDOM_DISTRIBUTION_HPP_

#include "murosim/random/random_generator.hpp"
#include <cmath>
#include <cstdint>

/// Single-precision distributions built directly on the generator's 64-bit
/// output. They replace `<random>` in the per-ray hot loops: libstdc++'s
/// `generate_canonical` evaluates two long-double logarithms on every call,
/// which Clang cannot fold at compile time (four `logl` calls per normal
/// draw, 8 % of a simulation step). The stream differs from
/// `std::uniform_real_distribution` / `std::normal_distribution`, so a seed
/// gives different noise values than before, but stays reproducible.
namespace murosim::random
{
/// Uniform float in [0, 1) from the top 24 bits of one generator draw
/// (exactly representable, equidistant, never 1.0).
inline float uniformUnit(RandomGenerator &random_generator)
{
    constexpr float SCALE = 1.0f / 16777216.0f; // 2^-24
    return static_cast<float>(random_generator() >> 40) * SCALE;
}

/// Uniform float in [min, max).
inline float uniformFloat(RandomGenerator &random_generator, const float min, const float max)
{
    return min + (max - min) * uniformUnit(random_generator);
}

/// Normal distribution N(mean, stddev) in float via the Marsaglia polar
/// method; every second draw is served from the cached spare value.
class NormalDistribution
{
private:
    float mean_;
    float stddev_;
    float spare_ = 0.0f;
    bool has_spare_ = false;

public:
    NormalDistribution(const float mean, const float stddev) : mean_(mean), stddev_(stddev)
    {
    }

    float operator()(RandomGenerator &random_generator)
    {
        if (has_spare_)
        {
            has_spare_ = false;
            return mean_ + stddev_ * spare_;
        }
        float u = 0.0f;
        float v = 0.0f;
        float s = 0.0f;
        do
        {
            u = uniformFloat(random_generator, -1.0f, 1.0f);
            v = uniformFloat(random_generator, -1.0f, 1.0f);
            s = u * u + v * v;
        } while (s >= 1.0f || s == 0.0f);
        const float factor = std::sqrt(-2.0f * std::log(s) / s);
        spare_ = v * factor;
        has_spare_ = true;
        return mean_ + stddev_ * u * factor;
    }
};
} // namespace murosim::random

#endif // MUROSIM_RANDOM_DISTRIBUTION_HPP_
