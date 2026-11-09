// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GRID_DRAWING_HPP_
#define MUROSIM_GRID_DRAWING_HPP_

#include "murosim/grid/map.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/geometry/line.hpp"

#include <cstdint>
#include <functional>
#include <vector>

/// Rasterization of geometry (lines, circles) and flood-fill onto a
/// murosim::grid::Map. Used both to build the initial occupancy map from
/// world geometry and, via circleCells, to compute robot/obstacle footprints
/// for observation builders and visualization, so the same cells are reported
/// everywhere. Overloads taking float/Eigen::Vector2f coordinates are in
/// world (metre) units and are converted internally via Map::fromWorldIntoMap;
/// int/Eigen::Vector2i overloads are already in grid-cell units.
namespace murosim::grid::drawing
{

/// Draws the Bresenham line from grid cell (p1x, p1y) to (p2x, p2y), setting each cell to symbol.
void line(Map &map, uint8_t symbol, int p1x, int p1y, int p2x, int p2y);
/// Draws the Bresenham line between two grid cells, setting each cell to symbol.
void line(Map &map, uint8_t symbol, Eigen::Vector2i p1, Eigen::Vector2i p2);
/// Draws the line between two world-coordinate points, setting each covered cell to symbol.
void line(Map &map, uint8_t symbol, float p1x, float p1y, float p2x, float p2y);
/// Draws the line between two world-coordinate points, setting each covered cell to symbol.
void line(Map &map, uint8_t symbol, Eigen::Vector2f p1, Eigen::Vector2f p2);
/// Draws a murosim::geometry::Line (world coordinates), setting each covered cell to symbol.
void line(Map &map, uint8_t symbol, murosim::geometry::Line l);

/// Sets every grid cell within radius cr (grid cells) of (cx, cy) to symbol.
void circle(Map &map, uint8_t symbol, int cx, int cy, int cr);
/// Sets every grid cell within radius r (grid cells) of p to symbol.
void circle(Map &map, uint8_t symbol, Eigen::Vector2i p, int r);
/// Sets every grid cell the world-space circle (cx, cy, cr) overlaps to symbol (see circleCells).
void circle(Map &map, uint8_t symbol, float cx, float cy, float cr);
/// Sets every grid cell the world-space circle (p, r) overlaps to symbol (see circleCells).
void circle(Map &map, uint8_t symbol, Eigen::Vector2f p, float r);
/// Sets every grid cell the world-space murosim::geometry::Circle c overlaps to symbol (see circleCells).
void circle(Map &map, uint8_t symbol, murosim::geometry::Circle c);

// Visits every in-map cell the circle (world coordinates) physically overlaps,
// always including the robot's center cell so even a circle far smaller than
// one cell yields at least one pixel. This is the single rasterization used by
// circle() and by observation builders, so footprints stay identical everywhere.
void circleCells(const Map &map, Eigen::Vector2f cp, float cr, const std::function<void(Eigen::Vector2i)> &visit);

/// Flood-fills the 4-connected region of symbol_old cells reachable from grid
/// cell (px, py), replacing them with symbol_new. A no-op if the start cell is
/// out of the map, is not symbol_old, or symbol_old == symbol_new (which would
/// never mark cells visited and loop forever).
void fill(Map &map, uint8_t symbol_old, uint8_t symbol_new, int px, int py);
/// Flood-fills the 4-connected region of symbol_old cells reachable from p, replacing them with symbol_new.
void fill(Map &map, uint8_t symbol_old, uint8_t symbol_new, Eigen::Vector2i p);
/// Same as fill(map, symbol_old, symbol_new, px, py), additionally appending
/// the linear index of every filled cell to indices (cleared first).
void fill(Map &map, uint8_t symbol_old, uint8_t symbol_new, int px, int py, std::vector<size_t> &indices);
/// Same as fill(map, symbol_old, symbol_new, p), additionally appending the
/// linear index of every filled cell to indices (cleared first).
void fill(Map &map, uint8_t symbol_old, uint8_t symbol_new, Eigen::Vector2i p, std::vector<size_t> &indices);

} // namespace murosim::grid::drawing

#endif // MUROSIM_GRID_DRAWING_HPP_
