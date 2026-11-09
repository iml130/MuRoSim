// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GEOMETRY_WORLD_HPP_
#define MUROSIM_GEOMETRY_WORLD_HPP_

#include "murosim/simd/array_float.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/geometry/shape.hpp"

namespace murosim::geometry
{

/// The static world geometry (all lines and circles making up the simulated
/// environment), stored as SIMD-friendly struct-of-arrays (murosim::simd::ArrayFloat)
/// so collision and lidar kernels can process every obstacle with vectorized
/// loops instead of iterating a vector of Line/Circle objects. Immutable after
/// construction; also tracks the overall axis-aligned bounding box of all
/// geometry, used e.g. by grid::Map to size the occupancy grid.
class World
{
private:
    murosim::simd::ArrayFloat array_line_point1_x_;
    murosim::simd::ArrayFloat array_line_point1_y_;
    murosim::simd::ArrayFloat array_line_point2_x_;
    murosim::simd::ArrayFloat array_line_point2_y_;

    murosim::simd::ArrayFloat array_circle_x_;
    murosim::simd::ArrayFloat array_circle_y_;
    murosim::simd::ArrayFloat array_circle_radius_;

    Eigen::Vector2f area_min_;
    Eigen::Vector2f area_max_;

    /// Fills the line SIMD arrays from lines and grows the AABB to include them.
    void fillArrayLines(const std::vector<murosim::geometry::Line> &lines);
    /// Fills the circle SIMD arrays from circles and grows the AABB to include them.
    void fillArrayCircles(const std::vector<murosim::geometry::Circle> &circles);
    /// Fills the line and circle SIMD arrays from a shape's lines and circles.
    void fillArrayFromShape(const murosim::geometry::Shape &shape);

public:
    /// Builds the world from explicit line and circle lists.
    World(const std::vector<murosim::geometry::Line> &lines, const std::vector<murosim::geometry::Circle> &circles);
    /// Builds the world from a shape's lines and circles.
    explicit World(const murosim::geometry::Shape &shape);

    /// Returns the first endpoints' x coordinates of every line.
    const murosim::simd::ArrayFloat &getArrayLinePoint1X() const;
    /// Returns the first endpoints' y coordinates of every line.
    const murosim::simd::ArrayFloat &getArrayLinePoint1Y() const;
    /// Returns the second endpoints' x coordinates of every line.
    const murosim::simd::ArrayFloat &getArrayLinePoint2X() const;
    /// Returns the second endpoints' y coordinates of every line.
    const murosim::simd::ArrayFloat &getArrayLinePoint2Y() const;

    /// Returns the x coordinates of every circle's center.
    const murosim::simd::ArrayFloat &getArrayCircleX() const;
    /// Returns the y coordinates of every circle's center.
    const murosim::simd::ArrayFloat &getArrayCircleY() const;
    /// Returns the radii of every circle.
    const murosim::simd::ArrayFloat &getArrayCircleRadius() const;

    /// Returns the minimum x of the world's axis-aligned bounding box.
    float getAabbMinX() const;
    /// Returns the minimum y of the world's axis-aligned bounding box.
    float getAabbMinY() const;
    /// Returns the maximum x of the world's axis-aligned bounding box.
    float getAabbMaxX() const;
    /// Returns the maximum y of the world's axis-aligned bounding box.
    float getAabbMaxY() const;
    /// Returns the minimum corner of the world's axis-aligned bounding box.
    Eigen::Vector2f getAabbMin() const;
    /// Returns the maximum corner of the world's axis-aligned bounding box.
    Eigen::Vector2f getAabbMax() const;
    /// Returns the width of the world's axis-aligned bounding box.
    float getAabbSizeX() const;
    /// Returns the height of the world's axis-aligned bounding box.
    float getAabbSizeY() const;
    /// Returns the (width, height) of the world's axis-aligned bounding box.
    Eigen::Vector2f getAabbSize() const;
    /// Returns the number of lines.
    size_t getLineSize() const;
    /// Returns a copy of the line at index i as a murosim::geometry::Line.
    murosim::geometry::Line getLineAt(const size_t i) const;
    /// Returns the first endpoint of the line at index i.
    Eigen::Vector2f getLinePoint1(const size_t i) const;
    /// Returns the second endpoint of the line at index i.
    Eigen::Vector2f getLinePoint2(const size_t i) const;
    /// Returns the first endpoint's x coordinate of the line at index i.
    float getLinePoint1X(const size_t i) const;
    /// Returns the first endpoint's y coordinate of the line at index i.
    float getLinePoint1Y(const size_t i) const;
    /// Returns the second endpoint's x coordinate of the line at index i.
    float getLinePoint2X(const size_t i) const;
    /// Returns the second endpoint's y coordinate of the line at index i.
    float getLinePoint2Y(const size_t i) const;
    /// Returns the number of circles.
    size_t getCircleSize() const;
    /// Returns a copy of the circle at index i as a murosim::geometry::Circle.
    murosim::geometry::Circle getCircleAt(const size_t i) const;
    /// Returns the center point of the circle at index i.
    Eigen::Vector2f getCirclePoint(const size_t i) const;
    /// Returns the center x coordinate of the circle at index i.
    float getCircleX(const size_t i) const;
    /// Returns the center y coordinate of the circle at index i.
    float getCircleY(const size_t i) const;
    /// Returns the radius of the circle at index i.
    float getCircleRadius(const size_t i) const;
    /// Rebuilds and returns a murosim::geometry::Shape containing all lines and circles.
    murosim::geometry::Shape getShape() const;
};

} // namespace murosim::geometry

#endif // MUROSIM_GEOMETRY_WORLD_HPP_
