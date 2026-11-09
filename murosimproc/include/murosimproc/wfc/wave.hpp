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

#ifndef MUROSIMPROC_WFC_WAVE_HPP_
#define MUROSIMPROC_WFC_WAVE_HPP_

#include <murosim/random/random_generator.hpp>
#include <murosimproc/util/array2d.hpp>

namespace murosimproc::wfc
{

/**
 * @brief Result of entropy calculation for cell selection
 */
struct EntropyResult
{
    enum class Status { Cell, Finished, Contradiction };

    Status status;
    size_t cell_index;  // Only valid when status == Cell

    /// Builds a result naming `index` as the cell with minimal entropy to collapse next.
    static EntropyResult cell(size_t index) { return {Status::Cell, index}; }
    /// Builds a result signalling that every cell has been collapsed to a single pattern.
    static EntropyResult finished() { return {Status::Finished, 0}; }
    /// Builds a result signalling that some cell has no possible pattern left.
    static EntropyResult contradiction() { return {Status::Contradiction, 0}; }

    /// Returns whether this result names a cell to collapse next (`cell_index` is valid).
    bool isCell() const { return status == Status::Cell; }
    /// Returns whether every cell has been collapsed to a single pattern.
    bool isFinished() const { return status == Status::Finished; }
    /// Returns whether some cell has run out of possible patterns.
    bool isContradiction() const { return status == Status::Contradiction; }
};

/// The wave-function-collapse "wave": for each of `size_x * size_y` grid cells, tracks which of the
/// `weights.size()` patterns are still possible, and incrementally maintains each cell's Shannon entropy
/// so the cell with the lowest (noise-broken) entropy can be found in `getMinimalEntropy()` without
/// rescanning the whole grid. Also tracks whether any cell has been driven to zero possible patterns
/// (a contradiction, meaning the current run must be discarded/restarted).
class Wave
{
private:

    struct ShannonEntropy
    {
        uint16_t counter_pattern;
        double weight_sum;
        double weight_sum_log;
        double entropy;
        ShannonEntropy():
            counter_pattern(0),
            weight_sum(0.0),
            weight_sum_log(0.0),
            entropy(0.0){}
        ShannonEntropy(const uint16_t patterns, const double sum, const double sum_log, const double value):
            counter_pattern(patterns),
            weight_sum(sum),
            weight_sum_log(sum_log),
            entropy(value){}
    };

    std::vector<ShannonEntropy> memoisations_;
    std::vector<double> weights_log_;
    std::vector<double> weights_;
    size_t size_x_;
    size_t size_y_;
    double min_abs_half_;

    bool contradiction_;
    murosimproc::util::Array2d<uint8_t> data_;

    /// (Re-)initializes every cell's memoised entropy to the value for "all patterns still possible".
    void fillMemoisation();
    /// Returns half of the smallest absolute pattern weight, used as the upper bound for the tie-breaking noise in `getMinimalEntropy()`.
    double getMinAbsHalf() const;
public:
    /// Finds the cell with the lowest entropy that is not yet collapsed to a single pattern, breaking near-ties
    /// with random noise (scaled by `getMinAbsHalf()`) so cell selection order is not purely deterministic by index.
    /// @return `EntropyResult::contradiction()` if a prior `set()` emptied some cell's pattern set, `EntropyResult::finished()` if every cell already has exactly one possible pattern, otherwise `EntropyResult::cell(index)`.
    EntropyResult getMinimalEntropy(murosim::random::RandomGenerator &random_generator) const;
    /// Constructs a wave of `size_x` by `size_y` cells, each initially allowing all `weights.size()` patterns.
    /// @param weights Relative weight of each pattern, used both for random pattern choice and entropy calculation.
    Wave(const size_t size_x, const size_t size_y, const std::vector<double> &weights);
    /// Returns whether `pattern` is still possible for the cell at flat `index`.
    bool get(const size_t index, const size_t pattern) const;
    /// Returns whether `pattern` is still possible for the cell at `(x, y)`.
    bool get(const size_t x, const size_t y, const size_t pattern) const;
    /// Enables or disables `pattern` for the cell at flat `index`, updating its memoised entropy; disabling the last remaining pattern of a cell marks the wave as contradictory.
    void set(const size_t index, const size_t pattern, const bool value);
    /// Enables or disables `pattern` for the cell at `(x, y)`; see the index overload.
    void set(const size_t x, const size_t y, const size_t pattern, const bool value);
    /// Reads out the single surviving pattern index of every cell into a `size_x` by `size_y` grid.
    /// Asserts (debug builds only) that each cell has collapsed to exactly one pattern; undefined otherwise.
    murosimproc::util::Array2d<uint16_t> createPatternArray2d() const;
};


} // namespace murosimproc::wfc

#endif // MUROSIMPROC_WFC_WAVE_HPP_
