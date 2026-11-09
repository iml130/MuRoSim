// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GRID_MAP_HPP_
#define MUROSIM_GRID_MAP_HPP_

#include "murosim/geometry/line.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/geometry/world.hpp"

#include <array>
#include <cstdint>
#include <ostream>
#include <vector>

namespace murosim::grid
{

/// A 2D occupancy grid built by rasterizing a murosim::geometry::World: every
/// cell holds a FIELD value (FREE, OCCUPIED, BUFFER or UNREACHABLE). The
/// constructor sizes the grid to the world's bounding box at the given cell
/// size, draws all obstacles as OCCUPIED, then reduces the FREE space to the
/// single region reachable from the given node points (or, with no points,
/// the largest connected FREE region), marking everything else UNREACHABLE,
/// and finally grows an obstacle-adjacent BUFFER band. Used by planning (grid
/// search) and as the basis for occupancy-style observations. The grid's y
/// axis is flipped relative to world y (row 0 is the top / maximum world y);
/// see fromWorldIntoMap/fromMapIntoWorld.
class Map
{
private:
    float map_scale_;
    float map_scale_inv_;
    float buffer_scale_;
    Eigen::Vector2f origin_;
    Eigen::Vector2f length_;
    size_t size_x_;
    size_t size_y_;
    std::vector<uint8_t> map_;

    void createMap(const murosim::geometry::World &world);
    void createFreeRegion(const std::vector<Eigen::Vector2f> &node_points);
    void createFreeRegion();
    std::vector<std::vector<size_t>> fillRegionsFromNodePoints(const std::vector<Eigen::Vector2f> &node_points);
    void applyBufferSpace();
    [[nodiscard]] std::vector<Eigen::Vector2i> createBufferCircle() const;

public:
    /// Cell classification: FREE is traversable, OCCUPIED is a drawn obstacle,
    /// BUFFER is a traversable safety margin around obstacles, and
    /// UNREACHABLE is free space excluded because it is not connected to the
    /// selected free region (e.g. an isolated room).
    enum FIELD : uint8_t { FREE = 0, OCCUPIED = 1, BUFFER = 2, UNREACHABLE = 3 };

    /** \brief Smallest supported cell size [m]; the constructor clamps to it
     * and SimulationConfig validates against it, so config and map agree. */
    static constexpr float GRID_SCALE_MIN = 0.01f;

    /// Lookup table mapping a FIELD value to a uint8_t visualization/export
    /// intensity, with BUFFER rendered as a distinct mid-grey (128).
    static const std::array<uint8_t, 256> LUT_WITH_BUFFER_UINT8_T;
    /// Same as LUT_WITH_BUFFER_UINT8_T, but BUFFER is rendered like FREE (0).
    static const std::array<uint8_t, 256> LUT_NO_BUFFER_UINT8_T;
    /// Lookup table mapping a FIELD value to a float occupancy value in [0, 1],
    /// with BUFFER rendered as a distinct mid-value (0.5).
    static const std::array<float, 256> LUT_WITH_BUFFER_FLOAT;
    /// Same as LUT_WITH_BUFFER_FLOAT, but BUFFER is rendered like FREE (0.0).
    static const std::array<float, 256> LUT_NO_BUFFER_FLOAT;

    /// Constructs an empty, zero-sized map.
    Map();
    /// Builds the occupancy grid from world at the given grid_scale [m/cell]
    /// (clamped to GRID_SCALE_MIN) and buffer_scale [m] obstacle margin. If
    /// points is non-empty, the FREE region is the union of the regions
    /// reachable from each point; otherwise it is the single largest
    /// connected FREE region. Throws std::runtime_error if points is
    /// non-empty but no FREE region could be created around any of them
    /// (e.g. buffer_scale too large).
    Map(const murosim::geometry::World &world, const std::vector<Eigen::Vector2f> &points = std::vector<Eigen::Vector2f>(), float grid_scale = 0.1f, float buffer_scale = 0.2f);

    /// Returns the cell size in metres.
    [[nodiscard]] float getGridScale() const;
    /// Returns the reciprocal of the cell size (1/metres).
    [[nodiscard]] float getGridScaleInv() const;
    /// Returns the obstacle buffer margin in metres.
    [[nodiscard]] float getBufferScale() const;
    /// Returns the world-coordinate origin (minimum corner of the world's AABB).
    [[nodiscard]] Eigen::Vector2f getOrigin() const;
    /// Returns the world-space extent (width, height) covered by the map.
    [[nodiscard]] Eigen::Vector2f getLength() const;
    /// Returns the total number of cells (sizeX() * sizeY()).
    [[nodiscard]] size_t size() const;
    /// Returns the grid width in cells.
    [[nodiscard]] size_t sizeX() const;
    /// Returns the grid height in cells.
    [[nodiscard]] size_t sizeY() const;

    /// Returns the FIELD value at grid cell p (asserts p is in the map).
    [[nodiscard]] const uint8_t& operator()(Eigen::Vector2i p) const;
    /// Returns the FIELD value at grid cell p (asserts p is in the map).
    [[nodiscard]] uint8_t& operator()(Eigen::Vector2i p);

    /// Returns the FIELD value at grid cell (x, y) (asserts it is in the map).
    [[nodiscard]] const uint8_t& operator()(size_t x, size_t y) const;
    /// Returns the FIELD value at grid cell (x, y) (asserts it is in the map).
    [[nodiscard]] uint8_t& operator()(size_t x, size_t y);

    /// Returns the FIELD value at the given linear index (asserts index < size()).
    [[nodiscard]] const uint8_t& operator[](size_t index) const;
    /// Returns the FIELD value at the given linear index (asserts index < size()).
    [[nodiscard]] uint8_t& operator[](size_t index);

    /// Converts a grid cell (x, y) to its linear index into the cell array (asserts it is in the map).
    [[nodiscard]] size_t pointToIndex(int x, int y) const;
    /// Converts a grid cell to its linear index into the cell array (asserts it is in the map).
    [[nodiscard]] size_t pointToIndex(Eigen::Vector2i point) const;
    /// Converts a linear index back to its grid cell (asserts index < size()).
    [[nodiscard]] Eigen::Vector2i indexToPoint(size_t index) const;
    /// Converts a linear index back to its grid cell coordinates (asserts index < size()).
    void indexToPoint(size_t index, int &x, int &y) const;

    /// Converts a world point (wx, wy) [m] to fractional grid coordinates (mx, my).
    void fromWorldIntoMap(float wx, float wy, float &mx, float &my) const;
    /// Converts a world point [m] to fractional grid coordinates (mx, my).
    void fromWorldIntoMap(Eigen::Vector2f wp, float &mx, float &my) const;
    /// Converts a world point (wx, wy) [m] to fractional grid coordinates.
    void fromWorldIntoMap(float wx, float wy, Eigen::Vector2f &mp) const;
    /// Converts a world point [m] to fractional grid coordinates.
    void fromWorldIntoMap(Eigen::Vector2f wp, Eigen::Vector2f &mp) const;
    /// Converts a world point (wx, wy) [m] to the nearest grid cell (mx, my).
    void fromWorldIntoMap(float wx, float wy, int &mx, int &my) const;
    /// Converts a world point [m] to the nearest grid cell (mx, my).
    void fromWorldIntoMap(Eigen::Vector2f wp, int &mx, int &my) const;
    /// Converts a world point (wx, wy) [m] to the nearest grid cell.
    void fromWorldIntoMap(float wx, float wy, Eigen::Vector2i &mp) const;
    /// Converts a world point [m] to the nearest grid cell.
    void fromWorldIntoMap(Eigen::Vector2f wp, Eigen::Vector2i &mp) const;
    /// Converts a grid cell (mx, my) to its world-space point (wx, wy) [m]; the
    /// inverse of the rounding fromWorldIntoMap(..., int&, int&) overloads.
    void fromMapIntoWorld(int mx, int my, float &wx, float &wy) const;
    /// Converts a grid cell to its world-space point (wx, wy) [m]; the inverse
    /// of the rounding fromWorldIntoMap(..., int&, int&) overloads.
    void fromMapIntoWorld(Eigen::Vector2i mp, float &wx, float &wy) const;
    /// Converts a grid cell (mx, my) to its world-space point [m]; the inverse
    /// of the rounding fromWorldIntoMap(..., Eigen::Vector2i&) overloads.
    void fromMapIntoWorld(int mx, int my, Eigen::Vector2f &wp) const;
    /// Converts a grid cell to its world-space point [m]; the inverse of the
    /// rounding fromWorldIntoMap(..., Eigen::Vector2i&) overloads.
    void fromMapIntoWorld(Eigen::Vector2i mp, Eigen::Vector2f &wp) const;

    /// Returns whether (x, y) is a valid grid cell.
    [[nodiscard]] bool isInMap(int x, int y) const;
    /// Returns whether (x, y) is a valid grid cell.
    [[nodiscard]] bool isInMap(size_t x, size_t y) const;
    /// Returns whether point is a valid grid cell.
    [[nodiscard]] bool isInMap(Eigen::Vector2i point) const;

    /// Returns whether the cell at index is FIELD::FREE (asserts index < size()).
    [[nodiscard]] bool isFree(size_t index) const;
    /// Returns whether cell (x, y) is FIELD::FREE (asserts it is in the map).
    [[nodiscard]] bool isFree(int x, int y) const;
    /// Returns whether cell (x, y) is FIELD::FREE (asserts it is in the map).
    [[nodiscard]] bool isFree(size_t x, size_t y) const;
    /// Returns whether point is FIELD::FREE (asserts it is in the map).
    [[nodiscard]] bool isFree(Eigen::Vector2i point) const;

    /// Returns whether the cell at index is FIELD::BUFFER (asserts index < size()).
    [[nodiscard]] bool isBuffer(size_t index) const;
    /// Returns whether cell (x, y) is FIELD::BUFFER (asserts it is in the map).
    [[nodiscard]] bool isBuffer(int x, int y) const;
    /// Returns whether cell (x, y) is FIELD::BUFFER (asserts it is in the map).
    [[nodiscard]] bool isBuffer(size_t x, size_t y) const;
    /// Returns whether point is FIELD::BUFFER (asserts it is in the map).
    [[nodiscard]] bool isBuffer(Eigen::Vector2i point) const;

    /// Returns whether the cell at index is FIELD::OCCUPIED or FIELD::UNREACHABLE (asserts index < size()).
    [[nodiscard]] bool isObstacle(size_t index) const;
    /// Returns whether cell (x, y) is FIELD::OCCUPIED or FIELD::UNREACHABLE (asserts it is in the map).
    [[nodiscard]] bool isObstacle(int x, int y) const;
    /// Returns whether cell (x, y) is FIELD::OCCUPIED or FIELD::UNREACHABLE (asserts it is in the map).
    [[nodiscard]] bool isObstacle(size_t x, size_t y) const;
    /// Returns whether point is FIELD::OCCUPIED or FIELD::UNREACHABLE (asserts it is in the map).
    [[nodiscard]] bool isObstacle(Eigen::Vector2i point) const;

    /// Sets cell (x, y) to symbol; a no-op if (x, y) is outside the map.
    void set(uint8_t symbol, int x, int y);
    /// Sets cell point to symbol; a no-op if point is outside the map.
    void set(uint8_t symbol, Eigen::Vector2i point);

    /// Streams an ASCII-art rendering of the map (' ' free, 'X' occupied, '*' buffer, '#' unreachable), framed in a box.
    friend std::ostream& operator<<(std::ostream &os, const Map& map);
};

} // namespace murosim::grid

#endif // MUROSIM_GRID_MAP_HPP_
