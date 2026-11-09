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

#ifndef MUROSIMPROC_WFC_TILES_HPP_
#define MUROSIMPROC_WFC_TILES_HPP_

#include "murosimproc/util/array2d.hpp"
#include "murosimproc/wfc/direction.hpp"

#include <stdint.h>
#include <array>

namespace murosimproc::wfc
{

/// The WFC tile set derived from a training image: every unique `pattern_size` by `pattern_size` window
/// (extracted with the given stride, plus its 8 mirror/rotation variants) becomes a pattern; a pattern's
/// weight is its occurrence count (including symmetry duplicates), later normalized to sum to 1; and
/// `getCompatible()` records, per pattern and cardinal direction, which other patterns may be its
/// neighbour there (their overlapping borders must match pixel-for-pixel).
class Tiles
{
public:
    /**
     * @brief Construct tiles from a training image
     * @param image Training image with cell values (0=empty, 1=wall, 2=obstacle)
     * @param pattern_size NxN window size for pattern extraction
     * @param stride Step size when sliding the extraction window (default: 1)
     */
    Tiles(const murosimproc::util::Array2d<uint8_t> &image, size_t pattern_size, size_t stride = 1);

    /// Returns the extracted (deduplicated, symmetry-expanded) patterns.
    const std::vector<murosimproc::util::Array2d<uint8_t>> &getPatterns() const;
    /// Returns the normalized weight of each pattern, parallel to `getPatterns()`.
    const std::vector<double> &getWeights() const;
    /// Returns, for each pattern and each `DIRECTION::CARDINAL`, the indices of patterns allowed as its neighbour in that direction.
    const std::vector<std::array<std::vector<uint16_t>, DIRECTION::SIZE>> &getCompatible() const;

    /// Get the pattern size used for extraction
    size_t getPatternSize() const { return pattern_size_; }

    /// Get the stride used for extraction
    size_t getStride() const { return stride_; }

    /// Returns whether any two extracted patterns are pixel-identical (an O(n^2) check; debug helper).
    bool hasDuplicateTiles() const;
    /// Returns every pair of indices `(i, j)` with `i < j` whose patterns are pixel-identical (debug helper).
    std::vector<std::pair<size_t, size_t>> getDuplicateTiles() const;

private:
    /// Pattern transformation types for augmentation
    enum class Transform : uint8_t
    {
        MIRROR,             ///< Horizontal flip.
        MIRROR_ROTATE_90,   ///< Horizontal flip followed by a 90-degree rotation.
        MIRROR_ROTATE_180,  ///< Horizontal flip followed by a 180-degree rotation.
        MIRROR_ROTATE_270,  ///< Horizontal flip followed by a 270-degree rotation.
        ROTATE_90,          ///< 90-degree rotation.
        ROTATE_180,         ///< 180-degree rotation.
        ROTATE_270          ///< 270-degree rotation.
    };

    std::vector<murosimproc::util::Array2d<uint8_t>> tiles_;
    std::vector<double> weights_;
    std::vector<std::array<std::vector<uint16_t>, DIRECTION::SIZE>> compatible_;
    size_t pattern_size_;
    size_t stride_;

    /// Slides a `pattern_size` window across `image` in steps of `stride`, adding each window and its 8
    /// mirror/rotation variants as patterns (merging exact duplicates into a single, higher-weighted entry).
    /// @throws std::overflow_error if more than 65535 unique patterns are found (pattern indices are `uint16_t`).
    void createTilesFromImage(const murosimproc::util::Array2d<uint8_t> &image, size_t pattern_size, size_t stride);
    /// Returns `tile` mirrored and/or rotated according to `transform`.
    murosimproc::util::Array2d<uint8_t> transformTile(const murosimproc::util::Array2d<uint8_t> &tile, Transform transform) const;
    /// Returns whether `p1`'s edge facing `cardinal_direction` matches `p2`'s opposite edge pixel-for-pixel, i.e. whether the two patterns may be placed adjacent to each other in that direction.
    static bool isPatternMatching(const murosimproc::util::Array2d<uint8_t> &p1, const murosimproc::util::Array2d<uint8_t> &p2, DIRECTION::CARDINAL cardinal_direction);
    /// Rescales `weights_` in place so they sum to 1.
    void normalizeWeights();
    /// Fills `compatible_` by testing every ordered pattern pair against `isPatternMatching()` in every direction.
    void createCompatible();
};

} // namespace murosimproc::wfc

#endif // MUROSIMPROC_WFC_TILES_HPP_
