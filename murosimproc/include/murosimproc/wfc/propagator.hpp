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

#ifndef MUROSIMPROC_WFC_PROPAGATOR_HPP_
#define MUROSIMPROC_WFC_PROPAGATOR_HPP_

#include "murosimproc/wfc/wave.hpp"
#include "murosimproc/wfc/direction.hpp"
#include "murosimproc/util/array3d.hpp"
#include "murosimproc/util/array2d.hpp"


#include <array>

namespace murosimproc::wfc
{

/// Propagates the consequences of removing a pattern from one WFC cell to its neighbours: for every grid
/// cell and pattern it counts, per cardinal direction, how many patterns in the neighbouring cell in that
/// direction would still be compatible with it, and once that count reaches zero for some direction the
/// pattern is removed there too (queued via `add()`, applied in `propagate()`). This keeps the wave's
/// possible-pattern sets consistent with the tile adjacency rules after every cell collapse.
class Propagator
{
private:
    /// A pending removal: pattern `pattern` is no longer possible at grid cell `(x, y)` and still needs to be propagated to its neighbours.
    struct Entry
    {
        long x;
        long y;
        uint16_t pattern;
        Entry(const long cell_x, const long cell_y, const uint16_t removed_pattern)
            : x(cell_x), y(cell_y), pattern(removed_pattern) {}
    };
    std::vector<Entry> propagating_;
    std::vector<std::array<std::vector<uint16_t>, DIRECTION::SIZE>> compatible_table_;
    murosimproc::util::Array3d<std::array<uint16_t, DIRECTION::SIZE>> compatible_;
    /// Returns whether `(x, y)` lies within the propagator's grid bounds.
    bool inRange(const long x, const long y) const;
    /// Resets the per-cell, per-pattern, per-direction compatible-neighbour-pattern counts to their initial (full) values.
    void initCompatible();
public:
    /// Constructs a propagator for a `size_x` by `size_y` grid using the given pattern adjacency table (as produced by `Tiles::getCompatible()`).
    Propagator(const size_t size_x, const size_t size_y, const std::vector<std::array<std::vector<uint16_t>, DIRECTION::SIZE>> &compatible);
    /// Drains the pending-removal queue, disabling any neighbouring pattern whose compatible-neighbour count in `wave` has dropped to zero, until the queue is empty.
    void propagate(murosimproc::wfc::Wave &wave);
    /// Queues `pattern` for removal at the cell with flat `index`, resetting its compatibility counters.
    void add(const size_t index, const size_t pattern);
    /// Queues `pattern` for removal at grid cell `(x, y)`, resetting its compatibility counters.
    void add(const size_t x, const size_t y, const size_t pattern);
    /// Same as the unsigned overload, taking signed coordinates (used internally while propagating to a neighbour).
    void add(const long x, const long y, const size_t pattern);
};


} // namespace murosimproc::wfc

#endif // MUROSIMPROC_WFC_PROPAGATOR_HPP_
