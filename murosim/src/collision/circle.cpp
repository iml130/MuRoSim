// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/collision/circle.hpp"

#include "murosim/collision/line.hpp"

namespace murosim::collision
{

bool circleWithWorld(const murosim::geometry::Circle& circle, const murosim::geometry::World &world)
{
    return circleWithLineArray(circle.getX(), circle.getY(), circle.getRadius(), world.getArrayLinePoint1X(), world.getArrayLinePoint1Y(), world.getArrayLinePoint2X(), world.getArrayLinePoint2Y())
        || circleWithCircleArray(circle.getX(), circle.getY(), circle.getRadius(), world.getArrayCircleX(), world.getArrayCircleY(), world.getArrayCircleRadius());
}

bool circleWithShape(const murosim::geometry::Circle& circle, const murosim::geometry::Shape &shape)
{
    for (const murosim::geometry::Line &line : shape.getLines())
    {
        if (circleWithLine(circle, line))
        {
            return true;
        }
    }

    for (const murosim::geometry::Circle &c : shape.getCircles())
    {
        if (circleWithCircle(circle, c))
        {
            return true;
        }
    }

    return false;
}

bool circleWithShape(const murosim::geometry::Circle& circle, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose)
{
    // transform the single query circle into the shape frame instead of every
    // shape primitive into the world frame
    return circleWithShape(pose.inverse() * circle, shape);
}

bool circleWithCircle(const murosim::geometry::Circle& circle1, const murosim::geometry::Circle& circle2)
{
    const Eigen::Vector2f diff = circle1.getPoint() - circle2.getPoint();
    const float r = circle1.getRadius() + circle2.getRadius();
    return diff.dot(diff) <= (r * r);
}

bool circleWithCircle(const float x1, const float y1, const float r1, const float x2, const float y2, const float r2)
{
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    const float r = r1 + r2;

    return (dx*dx + dy*dy) <= (r*r);
}

bool circleWithLine(const murosim::geometry::Circle& circle, const murosim::geometry::Line& line)
{
    return lineWithCircle(line, circle);
}

bool circleWithLine(const Eigen::Vector2f& circle_point, const float circle_radius, const Eigen::Vector2f& line_point1, const Eigen::Vector2f& line_point2)
{
    return lineWithCircle(line_point1, line_point2, circle_point, circle_radius);
}

bool circleWithLine(const float cx, const float cy, const float cr, const float x1, const float y1, const float x2, const float y2)
{
    return lineWithCircle(Eigen::Vector2f(x1,y1), Eigen::Vector2f(x2,y2), Eigen::Vector2f(cx, cy), cr);
}

} // namespace murosim::collision
