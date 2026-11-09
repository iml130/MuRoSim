// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GEOMETRY_SHAPE_HPP_
#define MUROSIM_GEOMETRY_SHAPE_HPP_

#include "circle.hpp"
#include "line.hpp"

#include <vector>
#include <ostream>

namespace murosim::geometry
{

/// A composite 2D shape made of line segments and circles, used to describe
/// robot/obstacle footprints (e.g. built from murosim::geometry::primitives
/// or assembled piece by piece) that are later transformed into the world
/// frame and checked against murosim::collision or rasterized onto a grid::Map.
class Shape
{
private:
    std::vector<Line> lines_;
    std::vector<Circle> circles_;
public:
    /// Constructs an empty shape.
    Shape() = default;
    /// Constructs a shape from copies of the given lines and circles.
    Shape(const std::vector<Line> &lines, const std::vector<Circle> &circles);
    /// Constructs a shape by moving in the given lines and circles.
    Shape(std::vector<Line> && lines, std::vector<Circle> && circles);

    /// Removes all lines and circles.
    void clearShape();
    /// Removes all lines, keeping circles.
    void clearLines();
    /// Removes all circles, keeping lines.
    void clearCircles();

    /// Appends a line (moved).
    void addLine(Line &&line);
    /// Appends a copy of a line.
    void addLine(const Line &line);
    /// Appends a line from two endpoints given as coordinates.
    void addLine(const float x1, const float y1, const float x2, const float y2);
    /// Appends a line from two endpoints.
    void addLine(const Eigen::Vector2f p1, const Eigen::Vector2f p2);
    /// Appends copies of all given lines.
    void addLine(const std::vector<Line> &lines);
    /// Appends all given lines (moved).
    void addLine(std::vector<Line> &&lines);
    /// Appends a circle (moved).
    void addCircle(Circle &&circle);
    /// Appends a copy of a circle.
    void addCircle(const Circle &circle);
    /// Appends a circle from center coordinates and radius.
    void addCircle(const float x, const float y, const float radius);
    /// Appends a circle from a center point and radius.
    void addCircle(const Eigen::Vector2f point, const float radius);
    /// Appends copies of all given circles.
    void addCircle(const std::vector<Circle> &circles);
    /// Appends all given circles (moved).
    void addCircle(std::vector<Circle> &&circles);
    /// Appends copies of another shape's lines and circles.
    void addShape(const Shape &shape);
    /// Appends another shape's lines and circles, moving them out of it
    /// (the source shape is left empty).
    void addShape(Shape && shape);
    /// Appends another shape's lines and circles after transforming them by affine.
    void addShape(const Shape &shape, const Eigen::Affine2f &affine);
    /// Appends another shape's lines and circles after transforming them by
    /// affine, moving them out of it (the source shape is left empty).
    void addShape(Shape && shape, const Eigen::Affine2f &affine);
    /// Appends copies of this shape's lines and circles into the given vectors.
    void copyTo(std::vector<Line> &lines, std::vector<Circle> &circles) const;

    /// Returns the minimum x of the axis-aligned bounding box (0 if the shape is empty).
    float getAabbMinX() const;
    /// Returns the minimum y of the axis-aligned bounding box (0 if the shape is empty).
    float getAabbMinY() const;
    /// Returns the maximum x of the axis-aligned bounding box (0 if the shape is empty).
    float getAabbMaxX() const;
    /// Returns the maximum y of the axis-aligned bounding box (0 if the shape is empty).
    float getAabbMaxY() const;
    /// Returns the minimum corner of the axis-aligned bounding box (origin if the shape is empty).
    Eigen::Vector2f getAabbMin() const;
    /// Returns the maximum corner of the axis-aligned bounding box (origin if the shape is empty).
    Eigen::Vector2f getAabbMax() const;
    /// Returns the width of the axis-aligned bounding box.
    float getAabbSizeX() const;
    /// Returns the height of the axis-aligned bounding box.
    float getAabbSizeY() const;
    /// Returns the (width, height) of the axis-aligned bounding box.
    Eigen::Vector2f getAabbSize() const;

    /// Returns the lines making up the shape.
    const std::vector<Line>& getLines() const;
    /// Returns the lines making up the shape (mutable).
    std::vector<Line>& getLines();
    /// Returns the number of lines.
    size_t getLineSize() const;
    /// Returns a copy of the line at index (asserts index is in range).
    Line getLineAt(const size_t index) const;

    /// Returns the circles making up the shape.
    const std::vector<Circle>& getCircles() const;
    /// Returns the circles making up the shape (mutable).
    std::vector<Circle>& getCircles();
    /// Returns the number of circles.
    size_t getCircleSize() const;
    /// Returns a copy of the circle at index (asserts index is in range).
    Circle getCircleAt(const size_t index) const;

    /// Returns the smallest radius, centered at the shape's local origin, of
    /// a circle that fully encloses every line endpoint and circle.
    float getCollisionRadius() const;

    /// Applies an arbitrary affine transform to every line and circle.
    void transform(const Eigen::Affine2f &affine);
    /** \brief Place at pose (x, y, angle): rotate about the origin, then
     * translate (T * R) -- the conventional pose, equivalent to primitives::pose. */
    void transform(const float x, const float y, const float angle);
    /** \brief Alias for transform(x, y, angle). */
    void setPose(const float x, const float y, const float angle);
    /// Translates every line and circle by (x, y).
    void translate(const float x, const float y);
    /// Translates every line and circle.
    void translate(const Eigen::Vector2f translation);
    /// Rotates every line and circle about the origin by angle radians.
    void rotate(const float angle);
    /// Rotates every line and circle about the origin.
    void rotate(const Eigen::Rotation2Df rotation);
    /// Uniformly scales every line and circle about the origin by s.
    void scale(const float s);
    /// Scales every line and circle about the origin per axis.
    void scale(const float x, const float y);
    /// Scales every line and circle about the origin per axis.
    void scale(const Eigen::Vector2f scale_vec);

    /// Returns a new shape with the affine transform applied to every line and circle.
    friend Shape operator*(const Eigen::Affine2f &affine, const Shape &shape);
    /// Streams a human-readable "Shape(lines={...}, circles={...})" representation.
    friend std::ostream& operator<<(std::ostream &os, const Shape &shape);
};

} // namespace murosim::geometry

#endif // MUROSIM_GEOMETRY_SHAPE_HPP_
