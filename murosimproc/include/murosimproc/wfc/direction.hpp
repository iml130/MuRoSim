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

#ifndef MUROSIMPROC_WFC_DIRECTION_HPP_
#define MUROSIMPROC_WFC_DIRECTION_HPP_

#include <array>
#include <cstddef>
#include <string_view>


namespace murosimproc::wfc
{

/// The four cardinal grid directions used by WFC pattern adjacency and propagation, plus the
/// coordinate-offset and name lookup tables indexed by `CARDINAL`.
struct DIRECTION {
    /// The four cardinal directions, also used as indices into `X`, `Y` and the per-direction adjacency arrays.
    enum CARDINAL{NORTH, EAST, SOUTH, WEST};
    /// Grid x-offset for each `CARDINAL` value.
    static constexpr std::array<long, 4> X = { 0, 1,  0, -1};
    /// Grid y-offset for each `CARDINAL` value.
    static constexpr std::array<long, 4> Y = {-1, 0,  1,  0};
    /// Number of cardinal directions (4).
    static constexpr size_t SIZE = X.size();
    /// Returns the name of `cardinal_direction` ("NORTH", "EAST", "SOUTH", "WEST", or "UNKNOWN" for an invalid value).
    static std::string_view STRING(const CARDINAL cardinal_direction)
    {
        switch (cardinal_direction)
        {
        case CARDINAL::NORTH:
            return "NORTH";
        case CARDINAL::EAST:
            return "EAST";
        case CARDINAL::SOUTH:
            return "SOUTH";
        case CARDINAL::WEST:
            return "WEST";
        default:
            return "UNKNOWN";
        }
    }
};

} // namespace murosimproc::wfc

#endif // MUROSIMPROC_WFC_DIRECTION_HPP_
