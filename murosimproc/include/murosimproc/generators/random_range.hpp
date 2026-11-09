// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_RANDOM_RANGE_HPP_
#define MUROSIMPROC_GENERATORS_RANDOM_RANGE_HPP_

#include <murosim/random/random_generator.hpp>
#include <random>
#include <stdexcept>
#include <string>

namespace murosimproc::generators
{

/// A closed interval `[min, max]` used throughout the generators to describe either a fixed value
/// (`min == max`) or a range a parameter is drawn uniformly from at generation time (domain randomization).
class RandomRange {
    float min_, max_;
public:
    /// Single-value constructor (non-explicit for implicit float → RandomRange)
    RandomRange(float value)
        : min_(value), max_(value)
    {}

    /// Range constructor. Throws std::invalid_argument for min > max: a
    /// reversed range is always an argument swap, and silently clamping it
    /// to a constant would collapse domain randomization with no signal
    /// (every sibling generator parameter validates the same way).
    RandomRange(float min, float max)
        : min_(min), max_(max)
    {
        if (max < min)
        {
            throw std::invalid_argument("RandomRange min (" + std::to_string(min) + ") must not exceed max (" + std::to_string(max) + ")!");
        }
    }

    /// Returns the lower bound of the range.
    float min() const { return min_; }
    /// Returns the upper bound of the range.
    float max() const { return max_; }

    /// Draws a value uniformly from `[min, max]` using `rng`, advancing its state; returns `min()` directly (no draw) if the range is a fixed value.
    float operator()(murosim::random::RandomGenerator &rng) const
    {
        if (min_ == max_) {
            return min_;
        }
        std::uniform_real_distribution<float> dist(min_, max_);
        return dist(rng);
    }
};

} // namespace murosimproc::generators

#endif // MUROSIMPROC_GENERATORS_RANDOM_RANGE_HPP_
