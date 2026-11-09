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

#include "murosimproc/wfc/propagator.hpp"

#include "murosimproc/wfc/direction.hpp"

#include <cassert>

namespace murosimproc::wfc
{

Propagator::Propagator(const size_t size_x, const size_t size_y, const std::vector<std::array<std::vector<uint16_t>, DIRECTION::SIZE>> &compatible): compatible_table_(compatible), compatible_(size_x, size_y, compatible.size())
{
    initCompatible();
}

void Propagator::initCompatible()
{
    std::array<uint16_t, DIRECTION::SIZE> value = {0,0,0,0};
    const std::array<size_t, DIRECTION::SIZE> opposite = {DIRECTION::CARDINAL::SOUTH, DIRECTION::CARDINAL::WEST, DIRECTION::CARDINAL::NORTH, DIRECTION::CARDINAL::EAST};

    for (size_t x = 0; x < compatible_.sizeX(); x++)
    {
        for(size_t y = 0; y < compatible_.sizeY(); y++)
        {
            for (size_t pattern = 0; pattern < compatible_.sizeZ(); pattern++)
            {
                for (size_t direction = 0; direction < DIRECTION::SIZE; direction++)
                {
                   value[direction] = static_cast<uint16_t>(compatible_table_[pattern][opposite[direction]].size());
                }

                compatible_(x, y, pattern) = value;
            }
        }
    }
}

bool Propagator::inRange(const long x, const long y) const
{
    return 0 <= x && x < static_cast<long>(compatible_.sizeX()) && 0 <= y && y < static_cast<long>(compatible_.sizeY());
}

void Propagator::propagate(murosimproc::wfc::Wave &wave)
{
    while (!propagating_.empty())
    {
        const Entry entry = propagating_.back();
        propagating_.pop_back();

        for (size_t direction = 0; direction < DIRECTION::SIZE; direction++)
        {
            const long x = entry.x + DIRECTION::X[direction];
            const long y = entry.y + DIRECTION::Y[direction];

            if (inRange(x, y))
            {
                const std::vector<uint16_t> &patterns = compatible_table_[entry.pattern][direction];

                for (const uint16_t p : patterns)
                {
                    std::array<uint16_t, DIRECTION::SIZE> &value =
                        compatible_(static_cast<size_t>(x), static_cast<size_t>(y), p);
                    value[direction]--;

                    if (value[direction] == 0)
                    {
                        add(x, y, p);
                        wave.set(static_cast<size_t>(x), static_cast<size_t>(y), p, false);
                    }
                }
            }
        }
    }
}

void Propagator::add(const size_t index, const size_t pattern)
{
    assert(index < compatible_.sizeX() * compatible_.sizeY());
    assert(pattern < compatible_.sizeZ());

    add(index % compatible_.sizeX(), index / compatible_.sizeX(), pattern);
}

void Propagator::add(const size_t x, const size_t y, const size_t pattern)
{
    assert(x < compatible_.sizeX());
    assert(y < compatible_.sizeY());
    assert(pattern < compatible_.sizeZ());

    compatible_(static_cast<size_t>(x), static_cast<size_t>(y), pattern) = {0,0,0,0};
    propagating_.emplace_back(x, y, pattern);
}

void Propagator::add(const long x, const long y, const size_t pattern)
{
    assert(x >= 0 && static_cast<size_t>(x) < compatible_.sizeX());
    assert(y >= 0 && static_cast<size_t>(y) < compatible_.sizeY());
    assert(pattern < compatible_.sizeZ());

    compatible_(static_cast<size_t>(x), static_cast<size_t>(y), pattern) = {0,0,0,0};
    propagating_.emplace_back(x, y, pattern);
}

} // namespace murosimproc::wfc
