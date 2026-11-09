// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/geometry/world.hpp"

#include <limits>

namespace murosim::geometry
{

World::World(const std::vector<murosim::geometry::Line> &lines, const std::vector<murosim::geometry::Circle> &circles):
    area_min_(Eigen::Vector2f::Zero()),
    area_max_(Eigen::Vector2f::Zero())
{
    if (!lines.empty() || !circles.empty())
    {
        area_min_ = Eigen::Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        area_max_ = Eigen::Vector2f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    }
    fillArrayLines(lines);
    fillArrayCircles(circles);
}

World::World(const murosim::geometry::Shape &shape):
    area_min_(Eigen::Vector2f::Zero()),
    area_max_(Eigen::Vector2f::Zero())
{
    if (!shape.getLines().empty() || !shape.getCircles().empty())
    {
        area_min_ = Eigen::Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        area_max_ = Eigen::Vector2f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    }
    fillArrayFromShape(shape);
}

const murosim::simd::ArrayFloat &World::getArrayLinePoint1X() const
{
    return array_line_point1_x_;
}

const murosim::simd::ArrayFloat &World::getArrayLinePoint1Y() const
{
    return array_line_point1_y_;
}

const murosim::simd::ArrayFloat &World::getArrayLinePoint2X() const
{
    return array_line_point2_x_;
}

const murosim::simd::ArrayFloat &World::getArrayLinePoint2Y() const
{
    return array_line_point2_y_;
}

const murosim::simd::ArrayFloat &World::getArrayCircleX() const
{
    return array_circle_x_;
}

const murosim::simd::ArrayFloat &World::getArrayCircleY() const
{
    return array_circle_y_;
}

const murosim::simd::ArrayFloat &World::getArrayCircleRadius() const
{
    return array_circle_radius_;
}

void World::fillArrayFromShape(const murosim::geometry::Shape &shape)
{
    fillArrayLines(shape.getLines());
    fillArrayCircles(shape.getCircles());
}

void World::fillArrayLines(const std::vector<murosim::geometry::Line> &lines)
{
    // reserve the line vectors
    array_line_point1_x_ = murosim::simd::ArrayFloat(lines.size());
    array_line_point1_y_ = murosim::simd::ArrayFloat(lines.size());
    array_line_point2_x_ = murosim::simd::ArrayFloat(lines.size());
    array_line_point2_y_ = murosim::simd::ArrayFloat(lines.size());

    // fill the line vectors
    for (size_t i = 0; i < lines.size(); ++i)
    {
        const murosim::geometry::Line &line = lines[i];

        array_line_point1_x_[i] = line.getPoint1X();
        array_line_point1_y_[i] = line.getPoint1Y();
        array_line_point2_x_[i] = line.getPoint2X();
        array_line_point2_y_[i] = line.getPoint2Y();

        area_min_.x() = std::min(area_min_.x(), std::min(line.getPoint1X(), line.getPoint2X()));
        area_max_.x() = std::max(area_max_.x(), std::max(line.getPoint1X(), line.getPoint2X()));

        area_min_.y() = std::min(area_min_.y(), std::min(line.getPoint1Y(), line.getPoint2Y()));
        area_max_.y() = std::max(area_max_.y(), std::max(line.getPoint1Y(), line.getPoint2Y()));
    }
}

void World::fillArrayCircles(const std::vector<murosim::geometry::Circle> &circles)
{
    // reserve the circle vectors
    array_circle_x_ = murosim::simd::ArrayFloat(circles.size());
    array_circle_y_ = murosim::simd::ArrayFloat(circles.size());
    array_circle_radius_ = murosim::simd::ArrayFloat(circles.size());


    // fill the circle vectors
    for (size_t i = 0; i < circles.size(); ++i)
    {
        const murosim::geometry::Circle &circle = circles[i];

        array_circle_x_[i] = circle.getX();
        array_circle_y_[i] = circle.getY();
        array_circle_radius_[i] = circle.getRadius();

        area_min_.x() = std::min(area_min_.x(), circle.getX() - circle.getRadius());
        area_max_.x() = std::max(area_max_.x(), circle.getX() + circle.getRadius());

        area_min_.y() = std::min(area_min_.y(), circle.getY() - circle.getRadius());
        area_max_.y() = std::max(area_max_.y(), circle.getY() + circle.getRadius());
    }
}

float World::getAabbSizeX() const
{
    return area_max_.x() - area_min_.x();
}

float World::getAabbSizeY() const
{
    return area_max_.y() - area_min_.y();
}

Eigen::Vector2f World::getAabbSize() const
{
    return area_max_ - area_min_;
}

Eigen::Vector2f World::getAabbMin() const
{
    return area_min_;
}

Eigen::Vector2f World::getAabbMax() const
{
    return area_max_;
}

float World::getAabbMinX() const
{
    return area_min_.x();
}

float World::getAabbMinY() const
{
    return area_min_.y();
}

float World::getAabbMaxX() const
{
    return area_max_.x();
}

float World::getAabbMaxY() const
{
    return area_max_.y();
}

size_t World::getLineSize() const
{
    return array_line_point1_x_.size();
}

murosim::geometry::Line World::getLineAt(const size_t i) const
{
    return murosim::geometry::Line(array_line_point1_x_[i], array_line_point1_y_[i], array_line_point2_x_[i], array_line_point2_y_[i]);
}

Eigen::Vector2f World::getLinePoint1(const size_t i) const
{
    return Eigen::Vector2f(getLinePoint1X(i), getLinePoint1Y(i));
}

Eigen::Vector2f World::getLinePoint2(const size_t i) const
{
    return Eigen::Vector2f(getLinePoint2X(i), getLinePoint2Y(i));
}

float World::getLinePoint1X(const size_t i) const
{
    return array_line_point1_x_[i];
}

float World::getLinePoint1Y(const size_t i) const
{
    return array_line_point1_y_[i];
}

float World::getLinePoint2X(const size_t i) const
{
    return array_line_point2_x_[i];
}

float World::getLinePoint2Y(const size_t i) const
{
    return array_line_point2_y_[i];
}

size_t World::getCircleSize() const
{
    return array_circle_x_.size();
}

murosim::geometry::Circle World::getCircleAt(const size_t i) const
{
    return murosim::geometry::Circle(array_circle_x_[i], array_circle_y_[i], array_circle_radius_[i]);
}

Eigen::Vector2f World::getCirclePoint(const size_t i) const
{
    return Eigen::Vector2f(array_circle_x_[i], array_circle_y_[i]);
}

float World::getCircleX(const size_t i) const
{
    return array_circle_x_[i];
}

float World::getCircleY(const size_t i) const
{
    return array_circle_y_[i];
}

float World::getCircleRadius(const size_t i) const
{
    return array_circle_radius_[i];
}

murosim::geometry::Shape World::getShape() const
{
    murosim::geometry::Shape shape;

    for (size_t i = 0; i < getLineSize(); i++)
    {
        shape.addLine(array_line_point1_x_[i], array_line_point1_y_[i], array_line_point2_x_[i], array_line_point2_y_[i]);
    }

    for (size_t i = 0; i < getCircleSize(); i++)
    {
        shape.addCircle(array_circle_x_[i], array_circle_y_[i], array_circle_radius_[i]);
    }

    return shape;
}

} // namespace murosim::geometry
