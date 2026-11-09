// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_UTIL_IMAGE_HPP_
#define MUROSIMPROC_UTIL_IMAGE_HPP_

#include "murosimproc/util/array2d.hpp"
#include <murosim/geometry/line.hpp>
#include <Eigen/Core>
#include <array>

/// 4-connected neighbour offsets in grid-cell units, in order North, East, South, West.
inline const std::array<Eigen::Vector2i, 4> NEIGHBORS4 = {Eigen::Vector2i(0, -1), Eigen::Vector2i(1, 0), Eigen::Vector2i(0, 1), Eigen::Vector2i(-1, 0)}; // North, East, South, West

namespace murosimproc::util::image
{

/// Flood-fills the 4-connected region of cells equal to `old_value` starting at `point`, setting them to `new_value`.
/// Does nothing if `point` is outside the image, if the cell at `point` does not equal `old_value`, or if `old_value == new_value` (would never terminate).
void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t old_value, const uint8_t new_value);
/// Same as the 4-argument overload, additionally appending the flat index of every cell that was changed to `indices` (cleared first).
void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t old_value, const uint8_t new_value, std::vector<size_t> &indices);
/// Flood-fills the 4-connected region of cells that differ from `new_value` starting at `point`, setting them all to `new_value`;
/// the region may contain several values (e.g. free cells and obstacle zones together). Does nothing if `point` is outside the image or its cell already equals `new_value`.
void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t new_value);
/// Same as the 3-argument overload, additionally appending the flat index of every cell that was changed to `indices` (cleared first).
void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t new_value, std::vector<size_t> &indices);
/// Returns whether `point` lies within the image bounds.
bool isInImage(const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point);
/// Returns whether the signed coordinates `(x, y)` lie within the image bounds.
bool isInImage(const murosimproc::util::Array2d<uint8_t> &image, const int x, const int y);
/// Returns whether the unsigned coordinates `(x, y)` lie within the image bounds.
bool isInImage(const murosimproc::util::Array2d<uint8_t> &image, const size_t x, const size_t y);
/// Cell at `point`, which the caller has checked with `isInImage()` (the signed coordinates are cast, not validated).
inline uint8_t &at(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point)
{
    return image(static_cast<size_t>(point.x()), static_cast<size_t>(point.y()));
}
/// Const overload of `at()`.
inline uint8_t at(const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point)
{
    return image(static_cast<size_t>(point.x()), static_cast<size_t>(point.y()));
}
/// Flat row-major index of `point`, which the caller has checked with `isInImage()`.
inline size_t toIndex(const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point)
{
    return image.toIndex(static_cast<size_t>(point.x()), static_cast<size_t>(point.y()));
}
/// Traces the outlines between free (0) and occupied cells, returning one closed polyline (in world/cell-centre-offset coordinates) per contour loop.
std::vector<std::vector<Eigen::Vector2f>> findContourPoints(const murosimproc::util::Array2d<uint8_t> &image);
/// Same as `findContourPoints` but returns the contours as individual line segments instead of polylines.
std::vector<murosim::geometry::Line> findContourLines(const murosimproc::util::Array2d<uint8_t> &image);

/// True if a robot parked on this free cell (value 0) does not block other
/// robots: the cell is a dead end (exactly one free 4-neighbour) or part of a
/// fully free 2x2 block (open area, others can drive around). Corridor, curve
/// and intersection cells return false.
bool isNodeCandidate(const murosimproc::util::Array2d<uint8_t> &image, int x, int y);

} // namespace murosimproc::util::image

/// Prints an ASCII-art rendering of the image, framed by a border: space for value 0 (free), '#' for 1 (occupied), '.' for 2, '?' for any other value.
std::ostream& operator<<(std::ostream &os, const murosimproc::util::Array2d<uint8_t> &image);

#endif // MUROSIMPROC_UTIL_IMAGE_HPP_
