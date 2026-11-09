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

#include "murosimproc/wfc/wfc.hpp"

namespace murosimproc::wfc
{

WaveFunctionCollapse::WaveFunctionCollapse(const Tiles &tiles): tiles_(tiles)
{

}

murosimproc::util::Array2d<uint8_t> WaveFunctionCollapse::run(
    const size_t size_x,
    const size_t size_y,
    const murosim::random::RandomGenerator::result_type seed,
    const size_t max_attempts)
{
    for (size_t attempt = 0; attempt < max_attempts; ++attempt)
    {
        // Use different seed for each retry attempt
        const auto attempt_seed = seed + static_cast<murosim::random::RandomGenerator::result_type>(attempt);

        Wave wave(size_x, size_y, tiles_.getWeights());
        Propagator propagator(size_x, size_y, tiles_.getCompatible());
        murosim::random::RandomGenerator random_generator(attempt_seed);

        bool success = false;
        bool failed = false;

        while (!success && !failed)
        {
            const ObserveResult result = observe(wave, propagator, random_generator);

            switch (result)
            {
                case ObserveResult::Finished:
                    success = true;
                    break;
                case ObserveResult::Contradiction:
                    failed = true;
                    break;
                case ObserveResult::Continue:
                    propagator.propagate(wave);
                    break;
            }
        }

        if (success)
        {
            return createImage(wave);
        }
        // Contradiction occurred, try again with next seed
    }

    // All attempts failed
    return murosimproc::util::Array2d<uint8_t>();
}

ObserveResult WaveFunctionCollapse::observe(Wave &wave, Propagator &propagator, murosim::random::RandomGenerator &random_generator) const
{
    const EntropyResult entropy_result = wave.getMinimalEntropy(random_generator);
    const std::vector<double> &weights = tiles_.getWeights();

    if (entropy_result.isContradiction())
    {
        return ObserveResult::Contradiction;
    }

    if (entropy_result.isFinished())
    {
        return ObserveResult::Finished;
    }

    const size_t cell_index = entropy_result.cell_index;

    double sum_weights = 0.0;
    for (size_t i = 0; i < weights.size(); i++)
    {
        sum_weights += static_cast<double>(wave.get(cell_index, i)) * weights[i];
    }

    std::uniform_real_distribution<double> distribution(0.0, sum_weights);
    double random_value = distribution(random_generator);
    size_t chosen_value = weights.size() - 1;

    for (size_t i = 0; i < weights.size(); i++)
    {
        random_value -= static_cast<double>(wave.get(cell_index, i)) * weights[i];
        if (random_value <= 0.0)
        {
            chosen_value = i;
            break;
        }
    }

    for (size_t i = 0; i < weights.size(); i++)
    {
        if (wave.get(cell_index, i) != (chosen_value == i))
        {
            propagator.add(cell_index, i);
            wave.set(cell_index, i, false);
        }
    }

    return ObserveResult::Continue;
}

murosimproc::util::Array2d<uint8_t> WaveFunctionCollapse::createImage(const Wave &wave) const
{
    const size_t pattern_size_x = tiles_.getPatterns()[0].sizeX();
    const size_t pattern_size_y = tiles_.getPatterns()[0].sizeY();
    const murosimproc::util::Array2d<uint16_t> pattern_map = wave.createPatternArray2d();

    murosimproc::util::Array2d<uint8_t> image = murosimproc::util::Array2d<uint8_t>(pattern_map.sizeX() * pattern_size_x , pattern_map.sizeY() * pattern_size_y);

    for (size_t index_x = 0; index_x < pattern_map.sizeX(); index_x++)
    {
        for (size_t index_y = 0; index_y < pattern_map.sizeY(); index_y++)
        {
            const murosimproc::util::Array2d<uint8_t> &pattern = tiles_.getPatterns()[pattern_map(index_x, index_y)];
            const size_t x = index_x * pattern_size_x;
            const size_t y = index_y * pattern_size_y;

            for (size_t px = 0; px < pattern.sizeX(); px++)
            {
                for (size_t py = 0; py < pattern.sizeY(); py++)
                {
                    image(x + px, y + py) = pattern(px, py);
                }
            }
        }
    }

    return image;
}

} // namespace murosimproc::wfc
