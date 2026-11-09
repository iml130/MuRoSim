// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GEOMETRY_LINE_HPP_
#define MUROSIM_GEOMETRY_LINE_HPP_

#include <Eigen/Geometry>
#include <ostream>

namespace murosim::geometry
{

/// A 2D line segment defined by two endpoints, the basic straight-edge
/// obstacle/wall primitive used throughout geometry, collision, grid and
/// navigation code. Coordinates are in world/local metres depending on the
/// frame the segment is expressed in.
class Line
{
private:
    Eigen::Vector2f point1_;
    Eigen::Vector2f point2_;
public:
    /// Constructs a degenerate segment with both endpoints at the origin.
    Line() noexcept : point1_(0.0f, 0.0f), point2_(0.0f, 0.0f)
    {
    }

    /// Constructs a segment from (x1, y1) to (x2, y2).
    Line(const float x1, const float y1, const float x2, const float y2) noexcept : point1_(x1, y1), point2_(x2, y2)
    {
    }

    /// Constructs a segment from point1 to point2.
    Line(const Eigen::Vector2f point1, const Eigen::Vector2f point2) noexcept : point1_(point1), point2_(point2)
    {
    }

    /// Sets the first endpoint's x coordinate.
    void setPoint1X(const float x) noexcept
    {
        point1_.x() = x;
    }

    /// Sets the first endpoint's y coordinate.
    void setPoint1Y(const float y) noexcept
    {
        point1_.y() = y;
    }

    /// Sets the second endpoint's x coordinate.
    void setPoint2X(const float x) noexcept
    {
        point2_.x() = x;
    }

    /// Sets the second endpoint's y coordinate.
    void setPoint2Y(const float y) noexcept
    {
        point2_.y() = y;
    }

    /// Sets the first endpoint.
    void setPoint1(const Eigen::Vector2f point) noexcept
    {
        point1_ = point;
    }

    /// Sets the first endpoint from x/y coordinates.
    void setPoint1(float x, float y) noexcept
    {
        point1_.x() = x;
        point1_.y() = y;
    }

    /// Sets the second endpoint.
    void setPoint2(const Eigen::Vector2f point) noexcept
    {
        point2_ = point;
    }

    /// Sets the second endpoint from x/y coordinates.
    void setPoint2(float x, float y) noexcept
    {
        point2_.x() = x;
        point2_.y() = y;
    }

    /// Returns the first endpoint.
    [[nodiscard]] Eigen::Vector2f getPoint1() const noexcept
    {
        return point1_;
    }

    /// Returns the second endpoint.
    [[nodiscard]] Eigen::Vector2f getPoint2() const noexcept
    {
        return point2_;
    }

    /// Returns the first endpoint's x coordinate.
    [[nodiscard]] float getPoint1X() const noexcept
    {
        return point1_.x();
    }

    /// Returns the first endpoint's y coordinate.
    [[nodiscard]] float getPoint1Y() const noexcept
    {
        return point1_.y();
    }

    /// Returns the second endpoint's x coordinate.
    [[nodiscard]] float getPoint2X() const noexcept
    {
        return point2_.x();
    }

    /// Returns the second endpoint's y coordinate.
    [[nodiscard]] float getPoint2Y() const noexcept
    {
        return point2_.y();
    }

    /// Returns the Euclidean distance between the two endpoints.
    [[nodiscard]] float getLength() const noexcept
    {
        return (point1_ - point2_).norm();
    }

    /** \brief Place at pose (x, y, angle): rotate about the origin, then
     * translate (T * R) -- the conventional pose, equivalent to
     * primitives::pose(x, y, angle). */
    /// Places both endpoints at pose (x, y, angle): rotate about the origin,
    /// then translate (T * R) -- the conventional pose, equivalent to
    /// primitives::pose(x, y, angle).
    void transform(const float x, const float y, const float angle)
    {
        const Eigen::Affine2f affine = Eigen::Translation2f(x, y) * Eigen::Rotation2Df(angle);
        point1_ = affine * point1_;
        point2_ = affine * point2_;
    }

    /// Applies an arbitrary affine transform to both endpoints.
    void transform(const Eigen::Affine2f &affine)
    {
        point1_ = affine * point1_;
        point2_ = affine * point2_;
    }

    /// Translates both endpoints by (x, y).
    void translate(const float x, const float y) noexcept
    {
        const Eigen::Vector2f translation(x, y);
        point1_ += translation;
        point2_ += translation;
    }

    /// Translates both endpoints.
    void translate(const Eigen::Vector2f translation) noexcept
    {
        point1_ += translation;
        point2_ += translation;
    }

    /// Rotates both endpoints about the origin by angle radians.
    void rotate(const float angle)
    {
        const Eigen::Rotation2Df rotation = Eigen::Rotation2Df(angle);
        point1_ = rotation * point1_;
        point2_ = rotation * point2_;
    }

    /// Rotates both endpoints about the origin.
    void rotate(const Eigen::Rotation2Df rotation)
    {
        point1_ = rotation * point1_;
        point2_ = rotation * point2_;
    }

    /// Uniformly scales both endpoints about the origin by s.
    void scale(const float s)
    {
        scale(s, s);
    }

    /// Scales both endpoints about the origin per axis.
    void scale(const float x, const float y)
    {
        const Eigen::Vector2f scale_vec = Eigen::Vector2f(x, y);
        point1_.array() *= scale_vec.array();
        point2_.array() *= scale_vec.array();
    }

    /// Scales both endpoints about the origin per axis.
    void scale(const Eigen::Vector2f scale_vec)
    {
        point1_.array() *= scale_vec.array();
        point2_.array() *= scale_vec.array();
    }

    /// Exact equality of both endpoints (not commutative under swapped endpoints).
    [[nodiscard]] bool operator==(const Line &other) const noexcept
    {
        return point1_ == other.point1_ && point2_ == other.point2_;
    }

    [[nodiscard]] bool operator!=(const Line &other) const noexcept
    {
        return !(*this == other);
    }

    /// Returns a new line with the affine transform applied to both endpoints.
    friend Line operator*(const Eigen::Affine2f &affine, const Line &line)
    {
        return Line(affine * line.getPoint1(), affine * line.getPoint2());
    }

    /// Streams a human-readable "line(x1=.., y1=.., x2=.., y2=..)" representation.
    friend std::ostream& operator<<(std::ostream &os, const Line &line)
    {
        return os << "line(x1=" << line.getPoint1X() << ", y1=" << line.getPoint1Y() << ", x2=" << line.getPoint2X() << ", y2=" << line.getPoint2Y() << ")";
    }

};

} // namespace murosim::geometry

#endif // MUROSIM_GEOMETRY_LINE_HPP_
