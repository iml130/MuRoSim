// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GRID_BRESENHAM_HPP_
#define MUROSIM_GRID_BRESENHAM_HPP_

#include <Eigen/Geometry>

#include <functional>
#include <utility>
#include <vector>

/// Integer-grid rasterization primitives (Bresenham line/circle drawing and
/// footprint kernels) used to rasterize world geometry and robot/obstacle
/// footprints onto a murosim::grid::Map. All coordinates and radii are in
/// grid cells, not metres.
namespace murosim::grid::bresenham
{

// Visits every cell of the line without materializing a point vector — the
// streaming form drawing::line rasterizes with; createLine wraps it for
// callers that want the list.
void lineCells(int x1, int y1, int x2, int y2, const std::function<void(Eigen::Vector2i)> &visit);

/// Returns every grid cell on the Bresenham line from (x1, y1) to (x2, y2), inclusive of both endpoints.
[[nodiscard]] std::vector<Eigen::Vector2i> createLine(int x1, int y1, int x2, int y2);
/// Returns the grid cells forming the outline (not filled) of a circle of
/// radius r centered at (cx, cy) via the midpoint circle algorithm.
/// A radius <= 0 degenerates to the single center cell.
[[nodiscard]] std::vector<Eigen::Vector2i> createCircle(int cx, int cy, int r);
/// Returns the grid cells covering the filled disk of radius r centered at
/// (cx, cy): the outline from createCircle plus every interior cell.
[[nodiscard]] std::vector<Eigen::Vector2i> createCircleFilled(int cx, int cy, int r);
/// Builds a per-column (min_y, max_y) span kernel approximating a filled disk
/// of the given radius, as offsets from its center column; a radius < 2
/// degenerates to a single (0, 0) entry. Used to rasterize circular footprints
/// without recomputing createCircleFilled at every placement.
[[nodiscard]] std::vector<std::pair<int, int>> createKernelCircle(int radius);
/// Builds a per-column (0, 2*length) span kernel for a square footprint of
/// side 2*length+1 cells; a length < 2 degenerates to a single (0, 0) entry.
[[nodiscard]] std::vector<std::pair<int, int>> createKernelRectangle(int length);

} // namespace murosim::grid::bresenham

#endif // MUROSIM_GRID_BRESENHAM_HPP_
