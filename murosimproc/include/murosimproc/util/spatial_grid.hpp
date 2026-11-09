// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_UTIL_SPATIAL_GRID_HPP_
#define MUROSIMPROC_UTIL_SPATIAL_GRID_HPP_

#include "murosim/geometry/circle.hpp"
#include "murosim/collision/circle.hpp"
#include <unordered_map>
#include <vector>
#include <functional>

namespace murosimproc::util
{

/**
 * @brief Spatial grid for efficient collision detection
 *
 * Divides space into uniform grid cells to reduce collision checks
 * from O(n²) to approximately O(n) for uniformly distributed objects.
 */
class SpatialGrid
{
private:
    float cell_size_;

    // Use pair<int,int> as key for grid cells
    struct PairHash {
        std::size_t operator()(const std::pair<int, int> &p) const {
            // Combine hashes using bit shifting
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };

    std::unordered_map<std::pair<int, int>, std::vector<murosim::geometry::Circle>, PairHash> cells_;

    /**
     * @brief Get grid cell coordinates for a point
     */
    std::pair<int, int> getCellCoords(const Eigen::Vector2f &position) const;

    /**
     * @brief Get all cells that a circle might overlap
     */
    std::vector<std::pair<int, int>> getOverlappingCells(const murosim::geometry::Circle &circle) const;

public:
    /**
     * @brief Construct spatial grid with given cell size
     * @param cell_size Size of each grid cell (should be ~2x max object radius for best performance)
     */
    explicit SpatialGrid(float cell_size);

    /**
     * @brief Insert a circle into the spatial grid
     */
    void insert(const murosim::geometry::Circle &circle);

    /**
     * @brief Check if a circle collides with any circle in the grid
     * @return true if collision detected, false otherwise
     */
    bool hasCollision(const murosim::geometry::Circle &circle) const;

    /**
     * @brief Clear all circles from the grid
     */
    void clear();

    /**
     * @brief Get total number of circles stored
     */
    size_t size() const;
};

} // namespace murosimproc::util

#endif // MUROSIMPROC_UTIL_SPATIAL_GRID_HPP_
