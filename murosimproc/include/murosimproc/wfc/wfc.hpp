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

#ifndef MUROSIMPROC_WFC_WFC_HPP_
#define MUROSIMPROC_WFC_WFC_HPP_

#include <murosim/random/random_generator.hpp>
#include "murosimproc/wfc/propagator.hpp"
#include "murosimproc/wfc/wave.hpp"
#include "murosimproc/wfc/tiles.hpp"

#include <random>

namespace murosimproc::wfc
{

/// Default number of retry attempts when WFC encounters a contradiction
constexpr size_t DEFAULT_MAX_ATTEMPTS = 10;

/**
 * @brief Result of a single observation step
 */
enum class ObserveResult
{
    Continue,      ///< More cells to collapse, continue algorithm
    Finished,      ///< All cells collapsed successfully
    Contradiction  ///< No valid patterns remaining, generation failed
};

/// Runs the Wave Function Collapse algorithm with a fixed `Tiles` pattern set: repeatedly picks the
/// lowest-entropy uncollapsed cell, randomly chooses one of its remaining patterns weighted by pattern
/// weight, and propagates the resulting constraints, retrying with a new seed on contradiction.
class WaveFunctionCollapse
{
private:
    Tiles tiles_;
    /// Performs one observation step: collapses the current lowest-entropy cell to a single, weight-randomly-chosen pattern (propagating the removal of its other patterns), or reports that generation is finished or has hit a contradiction.
    ObserveResult observe(Wave &wave, Propagator &propagator, murosim::random::RandomGenerator &random_generator) const;
    /// Renders a fully-collapsed `wave` into a pixel grid by stamping each cell's chosen pattern at its `pattern_size`-scaled position.
    murosimproc::util::Array2d<uint8_t> createImage(const Wave &wave) const;
public:
    /// Constructs a WFC runner using `tiles` as the pattern set (copied).
    WaveFunctionCollapse(const Tiles &tiles);

    /**
     * @brief Run WFC algorithm with retry on contradiction
     * @param size_x Width of output grid, in pattern cells
     * @param size_y Height of output grid, in pattern cells
     * @param seed Random seed; attempt `i` (0-based) uses `seed + i`, so the same `seed` and `tiles_` always retry through the same sequence of attempts
     * @param max_attempts Maximum retry attempts on contradiction (default: 10)
     * @return Generated pattern grid (`size_x * pattern_size` by `size_y * pattern_size` pixels), or an empty array if every attempt hit a contradiction
     */
    murosimproc::util::Array2d<uint8_t> run(
        const size_t size_x,
        const size_t size_y,
        const murosim::random::RandomGenerator::result_type seed,
        const size_t max_attempts = DEFAULT_MAX_ATTEMPTS);
};


} // namespace murosimproc::wfc

#endif // MUROSIMPROC_WFC_WFC_HPP_
