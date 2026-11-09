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

#include "murosimproc/wfc/tiles.hpp"

#include "murosimproc/wfc/direction.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
namespace murosimproc::wfc
{

Tiles::Tiles(const murosimproc::util::Array2d<uint8_t> &image, size_t pattern_size, size_t stride)
    : pattern_size_(pattern_size)
    , stride_(stride)
{
    createTilesFromImage(image, pattern_size, stride);
    createCompatible();
    normalizeWeights();
}

const std::vector<murosimproc::util::Array2d<uint8_t>> &Tiles::getPatterns() const
{
    return tiles_;
}

const std::vector<double> &Tiles::getWeights() const
{
    return weights_;
}

const std::vector<std::array<std::vector<uint16_t>, DIRECTION::SIZE>> &Tiles::getCompatible() const
{
    return compatible_;
}

bool Tiles::isPatternMatching(const murosimproc::util::Array2d<uint8_t> &p1, const murosimproc::util::Array2d<uint8_t> &p2, DIRECTION::CARDINAL cardinal_direction)
{
    assert(p1.sizeX() == p2.sizeX());
    assert(p1.sizeY() == p2.sizeY());

    const size_t size_x = p1.sizeX() - 1;
    const size_t size_y = p1.sizeY() - 1;

    switch (cardinal_direction) {
    case DIRECTION::CARDINAL::NORTH:
        for (size_t x = 0; x <= size_x; x++) {
            if (p1(x, 0) != p2(x, size_y)) {
                return false;
            }
        }
        break;

    case DIRECTION::CARDINAL::EAST:
        for (size_t y = 0; y <= size_y; y++) {
            if (p1(size_x, y) != p2(0, y)) {
                return false;
            }
        }
        break;

    case DIRECTION::CARDINAL::SOUTH:
        for (size_t x = 0; x <= size_x; x++) {
            if (p1(x, size_y) != p2(x, 0)) {
                return false;
            }
        }
        break;

    case DIRECTION::CARDINAL::WEST:
        for (size_t y = 0; y <= size_y; y++) {
            if (p1(0, y) != p2(size_x, y)) {
                return false;
            }
        }
        break;

    default:
        return false;
    }

    return true;
}

murosimproc::util::Array2d<uint8_t> Tiles::transformTile(const murosimproc::util::Array2d<uint8_t> &tile, Transform transform) const
{
    murosimproc::util::Array2d<uint8_t> t = murosimproc::util::Array2d<uint8_t>(tile.sizeX(), tile.sizeY());

    switch (transform)
    {
    case Transform::MIRROR:
        for (size_t x = 0; x < tile.sizeX(); x++)
        {
            for (size_t y = 0; y < tile.sizeY(); y++)
            {
                t(x, y) = tile(tile.sizeX() - 1 - x, y);
            }
        }
        break;

    case Transform::MIRROR_ROTATE_90:
        for (size_t x = 0; x < tile.sizeX(); x++)
        {
            for (size_t y = 0; y < tile.sizeY(); y++)
            {
                t(x, y) = tile(tile.sizeY() - 1 - y, tile.sizeX() - 1 - x);
            }
        }
        break;

    case Transform::MIRROR_ROTATE_180:
        for (size_t x = 0; x < tile.sizeX(); x++)
        {
            for (size_t y = 0; y < tile.sizeY(); y++)
            {
                t(x, y) = tile(x, tile.sizeY() - 1 - y);
            }
        }
        break;

    case Transform::MIRROR_ROTATE_270:
        for (size_t x = 0; x < tile.sizeX(); x++)
        {
            for (size_t y = 0; y < tile.sizeY(); y++)
            {
                t(x, y) = tile(y, x);
            }
        }
        break;

    case Transform::ROTATE_90:
        for (size_t x = 0; x < tile.sizeX(); x++)
        {
            for (size_t y = 0; y < tile.sizeY(); y++)
            {
                t(x, y) = tile(tile.sizeY() - 1 - y, x);
            }
        }
        break;

    case Transform::ROTATE_180:
        for (size_t x = 0; x < tile.sizeX(); x++)
        {
            for (size_t y = 0; y < tile.sizeY(); y++)
            {
                t(x, y) = tile(tile.sizeX() - 1 - x, tile.sizeY() - 1 - y);
            }
        }
        break;

    case Transform::ROTATE_270:
        for (size_t x = 0; x < tile.sizeX(); x++)
        {
            for (size_t y = 0; y < tile.sizeY(); y++)
            {
                t(x, y) = tile(y, tile.sizeX() - 1 - x);
            }
        }
        break;

    default:
        break;
    }

    return t;
}

void Tiles::normalizeWeights()
{
    assert(tiles_.size() == weights_.size());

    if (!tiles_.empty())
    {
        double sum = 0.0;

        for (size_t i = 0; i < weights_.size(); i++)
        {
            sum += weights_[i];
        }

        const double normalize_inv = 1.0 / sum;

        for (size_t i = 0; i < weights_.size(); i++)
        {
            weights_[i] *= normalize_inv;
        }
    }
}

void Tiles::createCompatible()
{
    compatible_ = std::vector<std::array<std::vector<uint16_t>, DIRECTION::SIZE>>(tiles_.size());

    for (size_t p1 = 0; p1 < tiles_.size(); p1++)
    {
        for (size_t d = 0; d < DIRECTION::SIZE; d++)
        {
            const DIRECTION::CARDINAL cardinal_direction = static_cast<DIRECTION::CARDINAL>(d);

            for (size_t p2 = 0; p2 < tiles_.size(); p2++)
            {
                if (isPatternMatching(tiles_[p1], tiles_[p2], cardinal_direction))
                {
                    compatible_[p1][d].push_back(static_cast<uint16_t>(p2));
                }
            }
        }
    }
}

void Tiles::createTilesFromImage(const murosimproc::util::Array2d<uint8_t> &image, size_t pattern_size, size_t stride)
{
    assert(2 <= pattern_size);
    assert(pattern_size <= image.sizeX());
    assert(pattern_size <= image.sizeY());
    assert(stride >= 1);

    using pattern_map_t = std::unordered_map<murosimproc::util::Array2d<uint8_t>, size_t>;

    // Calculate number of extraction positions with given stride
    const size_t max_x = image.sizeX() - pattern_size;
    const size_t max_y = image.sizeY() - pattern_size;

    std::array<murosimproc::util::Array2d<uint8_t>, 8> patterns;
    pattern_map_t map;

    for (size_t y = 0; y <= max_y; y += stride)
    {
        for (size_t x = 0; x <= max_x; x += stride)
        {
            // Extract base pattern and all 8 transformations (including identity)
            patterns[0] = image.subArray2d(x, y, pattern_size, pattern_size);
            patterns[1] = transformTile(patterns[0], Transform::MIRROR);
            patterns[2] = transformTile(patterns[0], Transform::MIRROR_ROTATE_90);
            patterns[3] = transformTile(patterns[0], Transform::MIRROR_ROTATE_180);
            patterns[4] = transformTile(patterns[0], Transform::MIRROR_ROTATE_270);
            patterns[5] = transformTile(patterns[0], Transform::ROTATE_90);
            patterns[6] = transformTile(patterns[0], Transform::ROTATE_180);
            patterns[7] = transformTile(patterns[0], Transform::ROTATE_270);

            for (size_t i = 0; i < patterns.size(); i++)
            {
                auto result = map.insert(std::make_pair(patterns[i], tiles_.size()));

                if (!result.second)
                {
                    // Pattern already exists, increase weight
                    weights_[result.first->second] += 1.0;
                }
                else
                {
                    // New unique pattern
                    tiles_.push_back(patterns[i]);
                    weights_.push_back(1.0);
                }
            }
        }
    }

    if (tiles_.size() > std::numeric_limits<uint16_t>::max())
    {
        throw std::overflow_error(
            "Too many unique patterns (" + std::to_string(tiles_.size()) +
            "); maximum supported is " + std::to_string(std::numeric_limits<uint16_t>::max()));
    }
}

bool Tiles::hasDuplicateTiles() const
{
    for (size_t i = 0; i < tiles_.size(); i++)
    {
        for (size_t j = i + 1; j < tiles_.size(); j++)
        {
            if (tiles_[i] == tiles_[j])
            {
                return true;
            }
        }
    }

    return false;
}

std::vector<std::pair<size_t, size_t>> Tiles::getDuplicateTiles() const
{
    std::vector<std::pair<size_t, size_t>> duplicates;

    for (size_t i = 0; i < tiles_.size(); i++)
    {
        for (size_t j = i + 1; j < tiles_.size(); j++)
        {
            if (tiles_[i] == tiles_[j])
            {
                duplicates.push_back(std::make_pair(i, j));
            }
        }
    }

    return duplicates;
}

} // namespace murosimproc::wfc
