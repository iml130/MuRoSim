// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/util/grid.hpp"

#include <algorithm>

namespace murosimrl::util::grid
{

const float* gridValueLut(const bool with_buffer)
{
    return with_buffer ? murosim::grid::Map::LUT_WITH_BUFFER_FLOAT.data()
                       : murosim::grid::Map::LUT_NO_BUFFER_FLOAT.data();
}

void extractMapPatch(const murosim::grid::Map &map, const Eigen::Vector2i &center, const size_t patch_dim, const float* lut, float* patch)
{
    const int32_t patch_radius = static_cast<int32_t>(patch_dim / 2);
    const int32_t patch_start_x = center.x() - patch_radius;
    const int32_t patch_start_y = center.y() - patch_radius;
    const int32_t patch_end_x = patch_start_x + static_cast<int32_t>(patch_dim);
    const int32_t patch_end_y = patch_start_y + static_cast<int32_t>(patch_dim);

    const int32_t valid_x_start = std::max(0, patch_start_x);
    const int32_t valid_x_end = std::min(static_cast<int32_t>(map.sizeX()), patch_end_x);
    const int32_t valid_y_start = std::max(0, patch_start_y);
    const int32_t valid_y_end = std::min(static_cast<int32_t>(map.sizeY()), patch_end_y);

    std::fill(patch, patch + patch_dim * patch_dim, 1.0f);

    for (int32_t y = valid_y_start; y < valid_y_end; y++)
    {
        const size_t dest_row = static_cast<size_t>(y - patch_start_y) * patch_dim;

        for (int32_t x = valid_x_start; x < valid_x_end; x++)
        {
            patch[dest_row + static_cast<size_t>(x - patch_start_x)] =
                lut[map(static_cast<size_t>(x), static_cast<size_t>(y))];
        }
    }
}

Eigen::Vector2i embedMapCentered(const murosim::grid::Map &map, const size_t map_size, const float* lut, float* output)
{
    const size_t start_x = (map_size - map.sizeX()) / 2;
    const size_t start_y = (map_size - map.sizeY()) / 2;
    const size_t end_x = start_x + map.sizeX();
    const size_t end_y = start_y + map.sizeY();

    // top padding
    std::fill(output, output + start_y * map_size, 1.0f);
    output += start_y * map_size;

    size_t index_map = 0;

    for (size_t y = start_y; y < end_y; y++)
    {
        size_t x = 0;

        while (x < start_x)
        {
            output[x++] = 1.0f;
        }

        while (x < end_x)
        {
            output[x++] = lut[map[index_map++]];
        }

        while (x < map_size)
        {
            output[x++] = 1.0f;
        }

        output += map_size;
    }

    // bottom padding
    std::fill(output, output + (map_size - end_y) * map_size, 1.0f);

    return Eigen::Vector2i(static_cast<int>(start_x), static_cast<int>(start_y));
}

} // namespace murosimrl::util::grid
