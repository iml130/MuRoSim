// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/util/spatial_grid.hpp"
#include <cmath>
#include <stdexcept>

namespace murosimproc::util
{

SpatialGrid::SpatialGrid(float cell_size)
    : cell_size_(cell_size)
{
    if (cell_size <= 0.0f) {
        throw std::invalid_argument(
            "SpatialGrid cell_size must be > 0 (got " +
            std::to_string(cell_size) + ")");
    }
}

std::pair<int, int> SpatialGrid::getCellCoords(const Eigen::Vector2f &position) const
{
    return {
        static_cast<int>(std::floor(position.x() / cell_size_)),
        static_cast<int>(std::floor(position.y() / cell_size_))
    };
}

std::vector<std::pair<int, int>> SpatialGrid::getOverlappingCells(const murosim::geometry::Circle &circle) const
{
    const Eigen::Vector2f center = circle.getPoint();
    const float radius = circle.getRadius();

    // Calculate bounding box of circle in grid coordinates
    const int min_x = static_cast<int>(std::floor((center.x() - radius) / cell_size_));
    const int max_x = static_cast<int>(std::floor((center.x() + radius) / cell_size_));
    const int min_y = static_cast<int>(std::floor((center.y() - radius) / cell_size_));
    const int max_y = static_cast<int>(std::floor((center.y() + radius) / cell_size_));

    std::vector<std::pair<int, int>> overlapping_cells;
    overlapping_cells.reserve(static_cast<size_t>((max_x - min_x + 1) * (max_y - min_y + 1)));

    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            overlapping_cells.emplace_back(x, y);
        }
    }

    return overlapping_cells;
}

void SpatialGrid::insert(const murosim::geometry::Circle &circle)
{
    // Insert circle into all cells it overlaps
    const auto overlapping_cells = getOverlappingCells(circle);

    for (const auto &cell_coords : overlapping_cells) {
        cells_[cell_coords].push_back(circle);
    }
}

bool SpatialGrid::hasCollision(const murosim::geometry::Circle &circle) const
{
    // Check only the cells this circle overlaps
    const auto overlapping_cells = getOverlappingCells(circle);

    for (const auto &cell_coords : overlapping_cells) {
        auto it = cells_.find(cell_coords);
        if (it != cells_.end()) {
            // Check collision with all circles in this cell
            for (const auto &existing_circle : it->second) {
                if (murosim::collision::circleWithCircle(circle, existing_circle)) {
                    return true;
                }
            }
        }
    }

    return false;
}

void SpatialGrid::clear()
{
    cells_.clear();
}

size_t SpatialGrid::size() const
{
    size_t total = 0;
    for (const auto &[coords, circles] : cells_) {
        total += circles.size();
    }
    // Note: circles in multiple cells are counted multiple times
    // This is intentional as it reflects memory usage
    return total;
}

} // namespace murosimproc::util
