// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_COLLISION_LASER_HPP_
#define MUROSIM_COLLISION_LASER_HPP_

#include "murosim/sensor/lidar.hpp"
#include "murosim/geometry/world.hpp"
#include "murosim/geometry/shape.hpp"
#include "murosim/simd/array_float.hpp"

namespace murosim::collision
{
    /// Casts every ray -- all originating at the lidar frame's origin, with
    /// (array_x, array_y) holding each ray's unit direction -- against the
    /// line segment (p1x, p1y)-(p2x, p2y), and keeps in array_distance the
    /// nearer of any new hit and the value already stored there (so call
    /// order across obstacles does not matter). array_distance holds each
    /// ray's nearest hit so far, or +inf when nothing was hit yet; rays are
    /// unbounded, so a range limit is applied by the caller (see
    /// Lidar::applyNoise). A value stored on entry is never exceeded.
    void laserWithLine(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float p1x, const float p1y, const float p2x, const float p2y);
    /** \brief Circles are solid disks (matching the collision kernels): a ray
     * origin inside the circle is a contact and every ray reads distance 0. */
    void laserWithCircle(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float cx, const float cy, const float cr);
    /** \brief Like above, but only rays in [index_begin, index_end) are processed.
     * Writes are exact: partial SIMD lanes at the window edges are stored
     * element-wise (see storeWindow in array_kernels.cpp), so rays outside the
     * window are never touched and callers can pass an exact range. */
    void laserWithLine(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float p1x, const float p1y, const float p2x, const float p2y, const size_t index_begin, const size_t index_end);
    void laserWithCircle(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float cx, const float cy, const float cr, const size_t index_begin, const size_t index_end);
    /// Convenience overload of laserWithLine taking the endpoints as vectors.
    inline void laserWithLine(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const Eigen::Vector2f p1, const Eigen::Vector2f p2)
    {
        laserWithLine(array_x, array_y, array_distance, p1.x(), p1.y(), p2.x(), p2.y());
    }
    /// Convenience overload of laserWithLine taking a murosim::geometry::Line.
    inline void laserWithLine(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const murosim::geometry::Line line)
    {
        laserWithLine(array_x, array_y, array_distance, line.getPoint1X(), line.getPoint1Y(), line.getPoint2X(), line.getPoint2Y());
    }
    /// Convenience overload of laserWithCircle taking a murosim::geometry::Circle.
    inline void laserWithCircle(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const murosim::geometry::Circle circle)
    {
        laserWithCircle(array_x, array_y, array_distance, circle.getX(), circle.getY(), circle.getRadius());
    }
    /// Convenience overload of laserWithCircle taking the center as a vector.
    inline void laserWithCircle(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const Eigen::Vector2f center, const float radius)
    {
        laserWithCircle(array_x, array_y, array_distance, center.x(), center.y(), radius);
    }
} // namespace murosim::collision

#endif // MUROSIM_COLLISION_LASER_HPP_
