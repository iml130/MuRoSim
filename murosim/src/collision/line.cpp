// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/collision/line.hpp"

#include <cmath>

namespace murosim::collision
{

bool lineWithWorld(const murosim::geometry::Line& line, const murosim::geometry::World &world)
{
    return lineWithLineArray(line.getPoint1X(), line.getPoint1Y(), line.getPoint2X(), line.getPoint2Y(), world.getArrayLinePoint1X(), world.getArrayLinePoint1Y(), world.getArrayLinePoint2X(), world.getArrayLinePoint2Y())
        || lineWithCircleArray(line.getPoint1X(), line.getPoint1Y(), line.getPoint2X(), line.getPoint2Y(), world.getArrayCircleX(), world.getArrayCircleY(), world.getArrayCircleRadius());
}

bool lineWithShape(const murosim::geometry::Line& line, const murosim::geometry::Shape &shape)
{
    for (const murosim::geometry::Line &l : shape.getLines())
    {
        if (lineWithLine(line, l))
        {
            return true;
        }
    }

    for (const murosim::geometry::Circle &circle : shape.getCircles())
    {
        if (lineWithCircle(line, circle))
        {
            return true;
        }
    }

    return false;
}

bool lineWithShape(const murosim::geometry::Line& line, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose)
{
    // transform the single query line into the shape frame instead of every
    // shape primitive into the world frame
    return lineWithShape(pose.inverse() * line, shape);
}


bool lineWithCircle(const murosim::geometry::Line& line, const murosim::geometry::Circle& circle)
{
    return lineWithCircle(line.getPoint1(), line.getPoint2(), circle.getPoint(), circle.getRadius());
}

bool lineWithCircle(const Eigen::Vector2f &line_point1, const Eigen::Vector2f& line_point2, const Eigen::Vector2f& circle_point, const float circle_radius)
{
    /***
     * Parametrise the segment as P(t) = start + t * d for t in [0, 1], with d
     * its direction and f the vector from the circle centre to the start.
     * Substituting into |P(t) - centre|^2 = r^2 gives a quadratic in t:
     *
     *   (d.d) t^2 + 2 (f.d) t + (f.f - r^2) = 0
     *
     * so a = d.d, b = 2 (f.d) and c = f.f - r^2. The segment crosses the circle
     * boundary when the discriminant b^2 - 4ac is non-negative and at least one
     * root falls inside [0, 1].
     ***/
    const Eigen::Vector2f d = line_point1 - line_point2;
    const Eigen::Vector2f f = line_point2 - circle_point;

    const float radius_sq = circle_radius * circle_radius;
    const float f_sq = f.dot(f);

    // Solid-disk test: the segment collides if it crosses the circle boundary
    // OR lies (partly or wholly) inside the disk. Testing the endpoints first
    // also catches a segment fully contained in the circle.
    const Eigen::Vector2f g = line_point1 - circle_point;
    if (g.dot(g) <= radius_sq || f_sq <= radius_sq)
    {
        return true;
    }

    const float a = d.dot(d);

    // Degenerate segment (a point) that, per the endpoint test above, lies
    // outside the disk: no collision.
    if (a < 1e-12f)
    {
        return false;
    }

    const float b = 2.0f * f.dot(d);
    const float c = f_sq - radius_sq;

    const float discriminant_sq = b*b - 4.0f*a*c;

    // No intersection if discriminant is negative
    if (discriminant_sq < 0.0f)
    {
        return false;
    }

    const float discriminant = std::sqrt(discriminant_sq);

    const float t1 = (-b - discriminant) / (2.0f*a);
    const float t2 = (-b + discriminant) / (2.0f*a);

    return (0.0f <= t1 && t1 <= 1.0f) || (0.0f <= t2 && t2 <= 1.0f);
}

bool lineWithCircle(const float x1, const float y1, const float x2, const float y2, const float cx, const float cy, const float cr)
{
    return lineWithCircle(Eigen::Vector2f(x1,y1), Eigen::Vector2f(x2,y2), Eigen::Vector2f(cx, cy), cr);
}

bool lineWithLine(const murosim::geometry::Line& line1, const murosim::geometry::Line& line2)
{
    return lineWithLine(line1.getPoint1X(), line1.getPoint1Y(), line1.getPoint2X(), line1.getPoint2Y(), line2.getPoint1X(), line2.getPoint1Y(), line2.getPoint2X(), line2.getPoint2Y());
}

bool lineWithLine(const float x1, const float y1, const float x2, const float y2, const float x3, const float y3, const float x4, const float y4)
{
    /**
     * Formular from: Given two points on each line segment.
     * https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
     *
     * numerator t: (x1 - x3)(y3 - y4) - (y1 - y3)(x3 - x4)
     * numerator u: (x2 - x1)(y1 - y3) - (y2 - y1)(x1 - x3)
     * denominator: (x1 - x2)(y3 - y4) - (y1 - y2)(x3 - x4)
     */

    const float denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

    // Parallel lines: no intersection (ignoring collinear overlap case)
    if (std::abs(denominator) < 1e-12f)
    {
        return false;
    }

    const float denominator_inv = 1.0f / denominator;
    const float t = ((x1 - x3)*(y3 - y4) - (y1 - y3)*(x3 - x4)) * denominator_inv;
    const float u = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) * denominator_inv;

    return (0.0f <= t && t <= 1.0f) && (0.0f <= u && u <= 1.0f);
}


} // namespace murosim::collision
