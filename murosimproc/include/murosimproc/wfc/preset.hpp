// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_WFC_PRESET_HPP_
#define MUROSIMPROC_WFC_PRESET_HPP_

#include "murosimproc/wfc/tiles.hpp"

#include <array>
#include <string>
#include <string_view>

namespace murosimproc::wfc::preset
{

/**
 * @brief Preset pattern types for WFC world generation
 *
 * Each preset is a small ASCII training image ('#' wall, ' ' free, '.'
 * obstacle zone) from which 3x3 tiles are extracted; see the enumerators for
 * the character of the worlds they produce and `getPresetConfig()` for the
 * images themselves.
 */
enum class PresetType : uint8_t
{
    BLOCK,       ///< Balanced layout with walls and obstacles
    OBSTACLE,    ///< Open space with scattered obstacle clusters
    LABYRINTH,   ///< Dense maze with narrow corridors
    WAREHOUSE,   ///< Mixed walls with obstacle zones
    ROOM,        ///< Connected rooms with doorways
    SPARSE,      ///< Minimal walls, scattered L-shaped obstacles
    CAVERN,      ///< Organic thick walls with obstacle pockets
    LINES,       ///< Parallel wall and obstacle lines
    COUNT        ///< Number of preset types (must be last)
};

/**
 * @brief Configuration for a WFC preset pattern
 *
 * Defines all parameters needed to extract tiles from a training pattern:
 * - pattern: ASCII art string defining the training image
 * - pattern_size: NxN window size for tile extraction (e.g., 3 for 3x3)
 * - stride: Step size when sliding extraction window (1 = overlapping, N = non-overlapping)
 * - name: Human-readable preset name
 */
struct PresetConfig
{
    std::string_view pattern;      ///< ASCII pattern ('#'=wall, ' '=empty, '.'=obstacle)
    size_t pattern_size;           ///< NxN tile extraction window size
    size_t stride;                 ///< Extraction window step size
    std::string_view name;         ///< Human-readable name

    /**
     * @brief Extract tiles from this preset configuration
     * @return Tiles object with extracted patterns and compatibility rules
     */
    murosimproc::wfc::Tiles toTiles() const;
};

/// Number of available presets
inline constexpr size_t PRESET_COUNT = static_cast<size_t>(PresetType::COUNT);

/**
 * @brief Get the configuration for a preset type
 * @param preset_type The preset type
 * @return Reference to the preset configuration
 */
const PresetConfig& getPresetConfig(PresetType preset_type);

/**
 * @brief Get all preset configurations
 * @return Array of all preset configurations indexed by PresetType
 */
const std::array<PresetConfig, PRESET_COUNT>& getAllPresetConfigs();

/**
 * @brief Get preset tiles by enum type
 * @param preset_type The preset type to load
 * @return Tiles object configured with the selected preset
 */
murosimproc::wfc::Tiles getPreset(PresetType preset_type);

/**
 * @brief Get preset name as string
 * @param preset_type The preset type
 * @return Human-readable name of the preset
 */
std::string_view getPresetName(PresetType preset_type);

/**
 * @brief Parse an ASCII pattern string into tiles
 * @param str Pattern string ('#'=wall, ' '=empty, '.'=obstacle, newlines separate rows)
 * @param pattern_size NxN extraction window size
 * @param stride Step size for sliding window (default: 1)
 * @return Tiles object with extracted patterns
 */
murosimproc::wfc::Tiles parseStringToTiles(
    std::string_view str,
    size_t pattern_size,
    size_t stride = 1
);

} // namespace murosimproc::wfc::preset

#endif // MUROSIMPROC_WFC_PRESET_HPP_
