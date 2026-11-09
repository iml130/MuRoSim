// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_UTIL_GRID_HPP_
#define MUROSIMRL_UTIL_GRID_HPP_

#include "murosim/grid/map.hpp"

#include <Eigen/Core>

namespace murosimrl::util::grid
{

/// Map grid cell values to observation floats (FREE 0.0, OCCUPIED/UNREACHABLE
/// 1.0; BUFFER 0.5 with with_buffer, else free). Reuses murosim's 256-entry
/// Map LUTs, so any uint8 cell value is a safe index.
/// @param with_buffer Whether BUFFER cells map to 0.5 instead of FREE's 0.0.
/// @return Pointer to a 256-entry lookup table indexed by raw cell value; not owned by the caller.
const float* gridValueLut(const bool with_buffer);

/// Copy a patch_dim x patch_dim window of `map` centered on `center` (map
/// cells) into `patch`, mapping cell values through `lut`. Cells outside the
/// map read 1.0 (occupied). `patch` must hold patch_dim * patch_dim floats.
/// @param map Grid map to read from.
/// @param center Window center, in map cell coordinates.
/// @param patch_dim Side length of the square window, in cells.
/// @param lut Cell-value-to-float lookup table, e.g. from gridValueLut().
/// @param patch Output buffer of patch_dim * patch_dim floats, row-major.
void extractMapPatch(const murosim::grid::Map &map, const Eigen::Vector2i &center, const size_t patch_dim, const float* lut, float* patch);

/// Embed the full grid map centered in a map_size x map_size buffer, mapping
/// cell values through `lut`; the border padding reads 1.0 (occupied). The map
/// must fit (see requireGridMapFitSize). Returns the (x, y) cell offset of the
/// map inside the buffer, for callers that overlay additional pixels.
/// @param map Grid map to embed.
/// @param map_size Side length of the square output buffer, in cells; must be at least the map's size.
/// @param lut Cell-value-to-float lookup table, e.g. from gridValueLut().
/// @param output Output buffer of map_size * map_size floats, row-major.
/// @return Cell offset (x, y) of the map's origin inside `output`.
Eigen::Vector2i embedMapCentered(const murosim::grid::Map &map, const size_t map_size, const float* lut, float* output);

} // namespace murosimrl::util::grid

#endif // MUROSIMRL_UTIL_GRID_HPP_
