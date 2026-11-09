// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/wfc/preset.hpp"

#include <stdexcept>

namespace murosimproc::wfc::preset
{

namespace patterns
{

// Pattern rules:
// '#' = wall (impassable)
// ' ' = empty/navigable (always passable)
// '.' = obstacle zone (random obstacles placed here, must have adjacent ' ' for passage)
// All lines must have same length!

constexpr std::string_view BLOCK =
    "         \n"
    "  #  ##  \n"
    "     ##  \n"
    "         \n"
    "  ..     \n"
    "  ..  .  \n"
    "         \n";

constexpr std::string_view OBSTACLE =
    "...        \n"
    "..         \n"
    ".  .. ... .\n"
    "  . .     .\n"
    "  .    .. .\n"
    "  ... ... .\n"
    "           \n"
    "       ..  \n"
    "  ..   ..  \n";

constexpr std::string_view LABYRINTH =
    "##############\n"
    "#  #    #    #\n"
    "#  # ## # ## #\n"
    "#    #    #  #\n"
    "### ## #  ## #\n"
    "#      #     #\n"
    "# #### # ### #\n"
    "#    ###   # #\n"
    "## # # # # ###\n"
    "#  # #   #   #\n"
    "##############\n";

constexpr std::string_view WAREHOUSE =
    "##############\n"
    "#            #\n"
    "#  ....  ..  #\n"
    "#  ....  ..  #\n"
    "#        ..  #\n"
    "##  ###      #\n"
    "#     #      #\n"
    "# ..  #      #\n"
    "# ..  #### ###\n"
    "#      #     #\n"
    "#  .   # ..  #\n"
    "#      #     #\n"
    "##############\n";

constexpr std::string_view ROOM =
    "#### ####### ##\n"
    "#### #####    #\n"
    "               \n"
    "#### #####    #\n"
    "#### ####### ##\n"
    "##      #### ##\n"
    "##           ##\n"
    "##      #### ##\n"
    "#### #######   \n"
    "#### ##########\n"
    "#### ##########\n";

constexpr std::string_view SPARSE =
    "##############\n"
    "#            #\n"
    "#  ..    .   #\n"
    "#   .   ...  #\n"
    "#        .   #\n"
    "#   .        #\n"
    "#   ..   .   #\n"
    "#        ..  #\n"
    "#  .         #\n"
    "#  ..    .   #\n"
    "#            #\n"
    "##############\n";

constexpr std::string_view CAVERN =
    "##################\n"
    "###       ##### ##\n"
    "##    ..    ##   #\n"
    "#    ...  .     ##\n"
    "#             #  #\n"
    "##   . .   ##### #\n"
    "###  ...  ####   #\n"
    "##         #### ##\n"
    "#  ..  .     #####\n"
    "#   . ..    ######\n"
    "##    ..   #######\n"
    "###      #########\n"
    "##################\n";

constexpr std::string_view LINES =
    "   #    . \n"
    "   #    . \n"
    "   #    . \n"
    "          \n"
    "   #    . \n"
    "   #    . \n"
    "          \n"
    "          \n"
    "   #    . \n"
    "   #    . \n";

} // namespace patterns

namespace
{

/// Lookup table for ASCII character to cell value conversion
constexpr uint8_t charToCell(char c)
{
    switch (c)
    {
        case ' ': return 0;  // Empty/navigable
        case '#': return 1;  // Wall
        case '.': return 2;  // Obstacle
        default:  return 255; // Invalid
    }
}

/// All preset configurations indexed by PresetType
constexpr std::array<PresetConfig, PRESET_COUNT> PRESET_CONFIGS = {{
    {patterns::BLOCK,     3, 1, "Block"},
    {patterns::OBSTACLE,  3, 1, "Obstacle"},
    {patterns::LABYRINTH, 3, 1, "Labyrinth"},
    {patterns::WAREHOUSE, 3, 1, "Warehouse"},
    {patterns::ROOM,      3, 1, "Room"},
    {patterns::SPARSE,    3, 1, "Sparse"},
    {patterns::CAVERN,    3, 1, "Cavern"},
    {patterns::LINES,     3, 1, "Lines"},
}};

} // anonymous namespace

murosimproc::wfc::Tiles PresetConfig::toTiles() const
{
    return parseStringToTiles(pattern, pattern_size, stride);
}

const PresetConfig& getPresetConfig(PresetType preset_type)
{
    const auto index = static_cast<size_t>(preset_type);
    if (index >= PRESET_COUNT) [[unlikely]]
    {
        throw std::out_of_range("Invalid PresetType: " + std::to_string(index));
    }
    return PRESET_CONFIGS[index];
}

const std::array<PresetConfig, PRESET_COUNT>& getAllPresetConfigs()
{
    return PRESET_CONFIGS;
}

murosimproc::wfc::Tiles getPreset(PresetType preset_type)
{
    return getPresetConfig(preset_type).toTiles();
}

std::string_view getPresetName(PresetType preset_type)
{
    return getPresetConfig(preset_type).name;
}

murosimproc::wfc::Tiles parseStringToTiles(
    std::string_view str,
    size_t pattern_size,
    size_t stride)
{
    if (pattern_size < 2) [[unlikely]]
    {
        throw std::invalid_argument(
            "pattern_size must be at least 2, got " + std::to_string(pattern_size));
    }

    // Parse lines from pattern string
    std::vector<std::string_view> lines;
    size_t pos = 0;

    while (pos < str.size())
    {
        size_t end = str.find('\n', pos);
        if (end == std::string_view::npos)
        {
            if (pos < str.size())
            {
                lines.emplace_back(str.data() + pos, str.size() - pos);
            }
            break;
        }
        lines.emplace_back(str.data() + pos, end - pos);
        pos = end + 1;
    }

    if (lines.empty()) [[unlikely]]
    {
        throw std::invalid_argument("Pattern string is empty");
    }

    // Validate line lengths
    const size_t size_x = lines[0].size();
    for (size_t i = 1; i < lines.size(); ++i)
    {
        if (lines[i].size() != size_x) [[unlikely]]
        {
            throw std::invalid_argument(
                "Line " + std::to_string(i) + " has different length (" +
                std::to_string(lines[i].size()) + ") than first line (" +
                std::to_string(size_x) + ")");
        }
    }

    const size_t size_y = lines.size();

    // Validate dimensions
    if (size_x < pattern_size || size_y < pattern_size) [[unlikely]]
    {
        throw std::invalid_argument(
            "Pattern image (" + std::to_string(size_x) + "x" + std::to_string(size_y) +
            ") is smaller than pattern_size (" + std::to_string(pattern_size) + ")");
    }

    if (stride == 0) [[unlikely]]
    {
        throw std::invalid_argument("Stride must be greater than 0");
    }

    // Convert to Array2d
    std::vector<uint8_t> data;
    data.reserve(size_x * size_y);

    for (const auto& line : lines)
    {
        for (char c : line)
        {
            uint8_t cell = charToCell(c);
            if (cell == 255) [[unlikely]]
            {
                throw std::invalid_argument(
                    std::string("Invalid character '") + c + "' in pattern string");
            }
            data.push_back(cell);
        }
    }

    return murosimproc::wfc::Tiles(
        murosimproc::util::Array2d<uint8_t>(size_x, size_y, data),
        pattern_size,
        stride
    );
}

} // namespace murosimproc::wfc::preset
