// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT
//
// Portions of this file are adapted from fast-wfc
// (https://github.com/math-fehr/fast-wfc), Copyright (c) 2018-2019 Mathieu Fehr
// and Nathanaël Courant, released under the MIT License. The upstream notice
// is reproduced in THIRD_PARTY_LICENSES.md.

#include "murosimproc/wfc/wave.hpp"

#include <cmath>
#include <optional>

namespace murosimproc::wfc
{

Wave::Wave(const size_t size_x, const size_t size_y, const std::vector<double> &weights): weights_(weights), size_x_(size_x), size_y_(size_y), min_abs_half_(getMinAbsHalf()), contradiction_(false)
{
    data_ = murosimproc::util::Array2d<uint8_t>(size_x * size_y, weights.size(), 1);
    memoisations_ = std::vector<ShannonEntropy>(size_x * size_y);
    fillMemoisation();
}

void Wave::fillMemoisation()
{
    weights_log_ = std::vector<double>(weights_.size());
    double weights_sum = 0.0;
    double weights_sum_log = 0.0;

    for (size_t i = 0; i < weights_.size(); i++)
    {
        const double weight = weights_[i];
        weights_log_[i] = weight * std::log(weight);
        weights_sum += weight;
        weights_sum_log += weights_log_[i];
    }

    const double entropy = std::log(weights_sum) - weights_sum_log / weights_sum;
    const ShannonEntropy shanon_entropy = ShannonEntropy(static_cast<uint16_t>(weights_.size()), weights_sum, weights_sum_log, entropy);
    std::fill(std::begin(memoisations_), std::end(memoisations_), shanon_entropy);
}

double Wave::getMinAbsHalf() const
{
    double min = std::numeric_limits<double>::max();

    for (size_t i = 0; i < weights_.size(); i++)
    {
        const double min_abs_value = std::abs(weights_[i]);
        min = std::min(min, min_abs_value);
    }

    return min / 2.0;
}

EntropyResult Wave::getMinimalEntropy(murosim::random::RandomGenerator &random_generator) const
{
    if (contradiction_)
    {
        return EntropyResult::contradiction();
    }

    std::uniform_real_distribution<double> distribution(0.0, min_abs_half_);

    double min = std::numeric_limits<double>::max();
    std::optional<size_t> argmin = std::nullopt;

    for (size_t i = 0; i < memoisations_.size(); i++)
    {
        if (memoisations_[i].counter_pattern != 1)
        {
            const double entropy = memoisations_[i].entropy;

            if (entropy <= min)
            {
                const double noise = distribution(random_generator);

                if (entropy + noise < min)
                {
                    min = entropy + noise;
                    argmin = i;
                }
            }
        }
    }

    if (!argmin.has_value())
    {
        return EntropyResult::finished();
    }

    return EntropyResult::cell(*argmin);
}



bool Wave::get(const size_t x, const size_t y, const size_t pattern) const
{
    return data_(x + y * size_x_, pattern);
}

bool Wave::get(const size_t index, const size_t pattern) const
{
    return data_(index, pattern);
}

void Wave::set(const size_t x, const size_t y, const size_t pattern, const bool value)
{
    set(x + y * size_x_, pattern, value);
}

void Wave::set(const size_t index, const size_t pattern, const bool value)
{
    if (value != static_cast<bool>(data_(index, pattern)))
    {
        data_(index, pattern) = static_cast<uint8_t>(value);
        ShannonEntropy &entropy = memoisations_[index];

        double sum = entropy.weight_sum;
        entropy.entropy  += entropy.weight_sum_log / sum - log(sum);
        entropy.weight_sum -= weights_[pattern];
        entropy.weight_sum_log -= weights_log_[pattern];

        sum = entropy.weight_sum;
        if (sum > 0.0)
        {
            entropy.entropy -= entropy.weight_sum_log / sum - log(sum);
        }

        entropy.counter_pattern--;

        if (entropy.counter_pattern == 0)
        {
            contradiction_ = true;
        }
    }
}

murosimproc::util::Array2d<uint16_t> Wave::createPatternArray2d() const
{
    murosimproc::util::Array2d<uint16_t> pattern = murosimproc::util::Array2d<uint16_t>(size_x_, size_y_);

    for (size_t i = 0; i < data_.sizeX(); i++)
    {
        [[maybe_unused]] size_t pattern_count = 0; // only checked by the assert below
        for (size_t j = 0; j < data_.sizeY(); j++)
        {
            if (data_(i, j) == 1)
            {
                pattern[i] = static_cast<uint16_t>(j);
                pattern_count++;
            }
        }
        assert(pattern_count == 1 && "Each cell should have exactly one pattern after collapse");
    }

    return pattern;
}

} // namespace murosimproc::wfc
