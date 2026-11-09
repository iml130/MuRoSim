// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_GEOMETRY_CIRCLE_HPP_
#define MUROSIM_GEOMETRY_CIRCLE_HPP_

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <ostream>

namespace murosim::geometry
{

/// A 2D circle (center point + radius), the basic round obstacle/robot-shape
/// primitive used throughout geometry, collision, grid and navigation code.
/// The radius is always clamped to be non-negative. Coordinates are in
/// world/local metres depending on the frame the circle is expressed in.
class Circle
{
private:
    Eigen::Vector2f point_;
    float radius_;

    /// Returns the smallest per-axis scale factor of an affine transform
    /// (the square root of the smaller of the two column squared norms), used
    /// to shrink the radius consistently under non-uniform scale/shear.
    static float minScaleFromAffine(const Eigen::Affine2f &affine)
    {
        return std::sqrt(std::min(affine.linear().col(0).squaredNorm(), affine.linear().col(1).squaredNorm()));
    }

public:
    /// Constructs a zero-radius circle at the origin.
    Circle() noexcept : point_(0.0f, 0.0f), radius_(0.0f)
    {
    }

    /// Constructs a circle at (x, y) with the given radius; a negative radius is clamped to 0.
    Circle(const float x, const float y, const float radius) noexcept : point_(x, y), radius_(std::max(0.0f, radius))
    {
    }

    /// Constructs a circle at the given point with the given radius; a negative radius is clamped to 0.
    Circle(const Eigen::Vector2f point, const float radius) noexcept : point_(point), radius_(std::max(0.0f, radius))
    {
    }

    /// Sets the center's x coordinate.
    void setX(const float x) noexcept
    {
        point_.x() = x;
    }

    /// Sets the center's y coordinate.
    void setY(const float y) noexcept
    {
        point_.y() = y;
    }

    /// Sets the center point.
    void setPoint(const Eigen::Vector2f point) noexcept
    {
        point_ = point;
    }

    /// Sets the center point from x/y coordinates.
    void setPoint(const float x, const float y) noexcept
    {
        point_.x() = x;
        point_.y() = y;
    }

    /// Sets the radius; a negative value is clamped to 0.
    void setRadius(const float radius) noexcept
    {
        radius_ = std::max(0.0f, radius);
    }

    /// Returns the center point.
    [[nodiscard]] Eigen::Vector2f getPoint() const noexcept
    {
        return point_;
    }

    /// Returns the center's x coordinate.
    [[nodiscard]] float getX() const noexcept
    {
        return point_.x();
    }

    /// Returns the center's y coordinate.
    [[nodiscard]] float getY() const noexcept
    {
        return point_.y();
    }

    /// Returns the radius (always >= 0).
    [[nodiscard]] float getRadius() const noexcept
    {
        return radius_;
    }

    /** \brief Place at pose (x, y, angle): rotate about the origin, then
     * translate (T * R) -- the conventional pose, equivalent to
     * primitives::pose(x, y, angle). */
    /// Places the circle's center at pose (x, y, angle): rotate about the
    /// origin, then translate (T * R) -- the conventional pose, equivalent to
    /// primitives::pose(x, y, angle). The radius is unaffected (rotation and
    /// translation are isometries).
    void transform(const float x, const float y, const float angle)
    {
        const Eigen::Affine2f affine = Eigen::Translation2f(x, y) * Eigen::Rotation2Df(angle);
        point_ = affine * point_;
    }

    /// Applies an arbitrary affine transform to the center point, and shrinks
    /// the radius by the smaller of the transform's two axis scale factors
    /// (see minScaleFromAffine) so a non-uniform scale does not grow the circle.
    void transform(const Eigen::Affine2f &affine)
    {
        point_ = affine * point_;
        radius_ *= minScaleFromAffine(affine);
    }

    /// Rotates the center point about the origin by angle radians.
    void rotate(const float angle)
    {
        point_ = Eigen::Rotation2Df(angle) * point_;
    }

    /// Rotates the center point about the origin.
    void rotate(const Eigen::Rotation2Df rotation)
    {
        point_ = rotation * point_;
    }

    /// Translates the center point by (x, y).
    void translate(const float x, const float y) noexcept
    {
        point_ += Eigen::Vector2f(x, y);
    }

    /// Translates the center point.
    void translate(const Eigen::Vector2f translation) noexcept
    {
        point_ += translation;
    }

    /// Uniformly scales the center point about the origin and the radius by s.
    void scale(const float s)
    {
        scale(s, s);
    }

    /// Scales the center point about the origin per axis, and scales the
    /// radius by the smaller of the two absolute axis factors.
    void scale(const float x, const float y)
    {
        point_.x() *= x;
        point_.y() *= y;
        radius_ *= std::min(std::abs(x), std::abs(y));
    }

    /// Scales the center point about the origin per axis, and scales the
    /// radius by the smaller of the two absolute axis factors.
    void scale(const Eigen::Vector2f scale_vec)
    {
        point_ = point_.cwiseProduct(scale_vec);
        radius_ *= scale_vec.cwiseAbs().minCoeff();
    }

    /// Exact equality of center point and radius.
    [[nodiscard]] bool operator==(const Circle &other) const noexcept
    {
        return point_ == other.point_ && radius_ == other.radius_;
    }

    [[nodiscard]] bool operator!=(const Circle &other) const noexcept
    {
        return !(*this == other);
    }

    /// Returns a new circle with the affine transform applied (see transform(const Eigen::Affine2f&)).
    friend Circle operator*(const Eigen::Affine2f &affine, const Circle &circle)
    {
        const float scale = minScaleFromAffine(affine);
        return Circle(affine * circle.getPoint(), scale * circle.getRadius());
    }

    /// Streams a human-readable "circle(x=.., y=.., r=..)" representation.
    friend std::ostream& operator<<(std::ostream &os, const Circle &circle)
    {
        return os << "circle(x=" << circle.getX() << ", y=" << circle.getY() << ", r=" << circle.getRadius() << ")";
    }
};

} // namespace murosim::geometry

#endif  // MUROSIM_GEOMETRY_CIRCLE_HPP_
