// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/geometry/shape.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/geometry/primitives.hpp"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <limits>
#include <utility>

namespace murosim::geometry
{

Shape::Shape(const std::vector<Line> &lines, const std::vector<Circle> &circles): lines_(lines), circles_(circles)
{

}

Shape::Shape(std::vector<Line> && lines, std::vector<Circle> && circles): lines_(std::move(lines)), circles_(std::move(circles))
{

}

void Shape::clearShape()
{
    lines_.clear();
    circles_.clear();
}

void Shape::clearLines()
{
    lines_.clear();
}

void Shape::clearCircles()
{
    circles_.clear();
}

void Shape::addLine(Line &&line)
{
    lines_.emplace_back(std::move(line));
}

void Shape::addLine(const Line &line)
{
    lines_.emplace_back(line);
}

void Shape::addLine(const float x1, const float y1, const float x2, const float y2)
{
    lines_.emplace_back(x1, y1, x2, y2);
}

void Shape::addLine(const Eigen::Vector2f p1, const Eigen::Vector2f p2)
{
    lines_.emplace_back(p1, p2);
}

void Shape::addLine(const std::vector<Line> &lines)
{
    lines_.insert(std::end(lines_), std::cbegin(lines), std::cend(lines));
}

void Shape::addLine(std::vector<Line> &&lines)
{
    lines_.insert(std::end(lines_), std::make_move_iterator(std::begin(lines)), std::make_move_iterator(std::end(lines)));
}

void Shape::addCircle(Circle &&circle)
{
    circles_.emplace_back(std::move(circle));
}

void Shape::addCircle(const Circle &circle)
{
    circles_.emplace_back(circle);
}

void Shape::addCircle(const float x, const float y, const float radius)
{
    circles_.emplace_back(x, y, radius);
}

void Shape::addCircle(const Eigen::Vector2f point, const float radius)
{
    circles_.emplace_back(point, radius);
}

void Shape::addCircle(const std::vector<Circle> &circles)
{
    circles_.insert(std::end(circles_), std::cbegin(circles), std::cend(circles));
}

void Shape::addCircle(std::vector<Circle> &&circles)
{
    circles_.insert(std::end(circles_), std::make_move_iterator(std::begin(circles)), std::make_move_iterator(std::end(circles)));
}

void Shape::addShape(const Shape &shape)
{
    lines_.insert(std::end(lines_), std::cbegin(shape.lines_), std::cend(shape.lines_));
    circles_.insert(std::end(circles_), std::cbegin(shape.circles_), std::cend(shape.circles_));
}

void Shape::addShape(Shape && shape)
{
    std::move(std::begin(shape.lines_), std::end(shape.lines_), std::back_inserter(lines_));
    std::move(std::begin(shape.circles_), std::end(shape.circles_), std::back_inserter(circles_));

    shape.clearShape();
}

void Shape::addShape(const Shape &shape, const Eigen::Affine2f &affine)
{
    for (size_t i = 0; i < shape.lines_.size(); i++)
    {
        lines_.emplace_back(affine * shape.lines_[i]);
    }

    for (size_t i = 0; i < shape.circles_.size(); i++)
    {
        circles_.emplace_back(affine * shape.circles_[i]);
    }
}

void Shape::addShape(Shape && shape, const Eigen::Affine2f &affine)
{
    shape.transform(affine);

    std::move(std::begin(shape.lines_), std::end(shape.lines_), std::back_inserter(lines_));
    std::move(std::begin(shape.circles_), std::end(shape.circles_), std::back_inserter(circles_));

    shape.clearShape();
}

void Shape::copyTo(std::vector<Line> &lines, std::vector<Circle> &circles) const
{
    lines.insert(lines.end(), lines_.begin(), lines_.end());
    circles.insert(circles.end(), circles_.begin(), circles_.end());
}

float Shape::getCollisionRadius() const
{
    float collision_radius = 0.0f;

    for (size_t i = 0; i < lines_.size(); ++i)
    {
        const Line& line = lines_[i];

        collision_radius = std::max(collision_radius, std::max(line.getPoint1().norm(), line.getPoint2().norm()));
    }

    for (size_t i = 0; i < circles_.size(); ++i)
    {
        const Circle &circle = circles_[i];
        collision_radius = std::max(collision_radius, circle.getPoint().norm() + circle.getRadius());
    }

    return  collision_radius;
}

const std::vector<Line>& Shape::getLines() const
{
    return lines_;
}

std::vector<Line>& Shape::getLines()
{
    return lines_;
}

size_t Shape::getLineSize() const
{
    return lines_.size();
}

Line Shape::getLineAt(const size_t index) const
{
    assert(index < lines_.size());
    return lines_[index];
}

const std::vector<Circle>& Shape::getCircles() const
{
    return circles_;
}

std::vector<Circle>& Shape::getCircles()
{
    return circles_;
}

Circle Shape::getCircleAt(const size_t index) const
{
    assert(index < circles_.size());
    return circles_[index];
}

size_t Shape::getCircleSize() const
{
    return circles_.size();
}

float Shape::getAabbMinX() const
{
    if(lines_.empty() && circles_.empty())
    {
        return 0.0f;
    }

    float min = std::numeric_limits<float>::max();

    for (size_t i = 0; i < lines_.size(); i++)
    {
        const murosim::geometry::Line &line = lines_[i];
        min = std::min(min, std::min(line.getPoint1X(), line.getPoint2X()));
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        const murosim::geometry::Circle &circle = circles_[i];
        min = std::min(min, circle.getX() - circle.getRadius());
    }

    return min;
}

float Shape::getAabbMinY() const
{
    if(lines_.empty() && circles_.empty())
    {
        return 0.0f;
    }

    float min = std::numeric_limits<float>::max();

    for (size_t i = 0; i < lines_.size(); i++)
    {
        const murosim::geometry::Line &line = lines_[i];
        min = std::min(min, std::min(line.getPoint1Y(), line.getPoint2Y()));
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        const murosim::geometry::Circle &circle = circles_[i];
        min = std::min(min, circle.getY() - circle.getRadius());
    }

    return min;
}

float Shape::getAabbMaxX() const
{
    if(lines_.empty() && circles_.empty())
    {
        return 0.0f;
    }

    float max = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < lines_.size(); i++)
    {
        const murosim::geometry::Line &line = lines_[i];
        max = std::max(max, std::max(line.getPoint1X(), line.getPoint2X()));
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        const murosim::geometry::Circle &circle = circles_[i];
        max = std::max(max, circle.getX() + circle.getRadius());
    }

    return max;
}

float Shape::getAabbMaxY() const
{
    if(lines_.empty() && circles_.empty())
    {
        return 0.0f;
    }

    float max = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < lines_.size(); i++)
    {
        const murosim::geometry::Line &line = lines_[i];
        max = std::max(max, std::max(line.getPoint1Y(), line.getPoint2Y()));
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        const murosim::geometry::Circle &circle = circles_[i];
        max = std::max(max, circle.getY() + circle.getRadius());
    }

    return max;
}

Eigen::Vector2f Shape::getAabbMin() const
{
    if (lines_.empty() && circles_.empty())
    {
        return Eigen::Vector2f::Zero();
    }

    Eigen::Vector2f area_min = Eigen::Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

    for (size_t i = 0; i < lines_.size(); i++)
    {
        const murosim::geometry::Line &line = lines_[i];
        area_min.x() = std::min(area_min.x(), std::min(line.getPoint1X(), line.getPoint2X()));
        area_min.y() = std::min(area_min.y(), std::min(line.getPoint1Y(), line.getPoint2Y()));
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        const murosim::geometry::Circle &circle = circles_[i];
        area_min.x() = std::min(area_min.x(), circle.getX() - circle.getRadius());
        area_min.y() = std::min(area_min.y(), circle.getY() - circle.getRadius());
    }

    return area_min;
}

Eigen::Vector2f Shape::getAabbMax() const
{
    if (lines_.empty() && circles_.empty())
    {
        return Eigen::Vector2f::Zero();
    }

    Eigen::Vector2f area_max = Eigen::Vector2f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    for (size_t i = 0; i < lines_.size(); i++)
    {
        const murosim::geometry::Line &line = lines_[i];
        area_max.x() = std::max(area_max.x(), std::max(line.getPoint1X(), line.getPoint2X()));
        area_max.y() = std::max(area_max.y(), std::max(line.getPoint1Y(), line.getPoint2Y()));
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        const murosim::geometry::Circle &circle = circles_[i];
        area_max.x() = std::max(area_max.x(), circle.getX() + circle.getRadius());
        area_max.y() = std::max(area_max.y(), circle.getY() + circle.getRadius());
    }

    return area_max;
}

float Shape::getAabbSizeX() const
{
    return getAabbSize().x();
}

float Shape::getAabbSizeY() const
{
    return getAabbSize().y();
}

Eigen::Vector2f Shape::getAabbSize() const
{
    if (lines_.empty() && circles_.empty())
    {
        return Eigen::Vector2f::Zero();
    }

    Eigen::Vector2f area_min = Eigen::Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector2f area_max = Eigen::Vector2f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    for (size_t i = 0; i < lines_.size(); i++)
    {
        const murosim::geometry::Line &line = lines_[i];
        area_min.x() = std::min(area_min.x(), std::min(line.getPoint1X(), line.getPoint2X()));
        area_max.x() = std::max(area_max.x(), std::max(line.getPoint1X(), line.getPoint2X()));
        area_min.y() = std::min(area_min.y(), std::min(line.getPoint1Y(), line.getPoint2Y()));
        area_max.y() = std::max(area_max.y(), std::max(line.getPoint1Y(), line.getPoint2Y()));
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        const murosim::geometry::Circle &circle = circles_[i];
        area_min.x() = std::min(area_min.x(), circle.getX() - circle.getRadius());
        area_max.x() = std::max(area_max.x(), circle.getX() + circle.getRadius());
        area_min.y() = std::min(area_min.y(), circle.getY() - circle.getRadius());
        area_max.y() = std::max(area_max.y(), circle.getY() + circle.getRadius());
    }

    return area_max - area_min;
}

void Shape::transform(const Eigen::Affine2f &affine)
{
    for (size_t i = 0; i < lines_.size(); i++)
    {
        lines_[i] = affine * lines_[i];
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        circles_[i] = affine * circles_[i];
    }
}

void Shape::transform(const float x, const float y, const float angle)
{
    transform(primitives::pose(x, y, angle));
}

void Shape::setPose(const float x, const float y, const float angle)
{
    transform(x, y, angle);
}

void Shape::translate(const float x, const float y)
{
    translate(Eigen::Vector2f(x, y));
}

void Shape::translate(const Eigen::Vector2f translation)
{
    for (size_t i = 0; i < lines_.size(); i++)
    {
        lines_[i].translate(translation);
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        circles_[i].translate(translation);
    }
}

void Shape::rotate(const float angle)
{
    rotate(Eigen::Rotation2Df(angle));
}

void Shape::rotate(const Eigen::Rotation2Df rotation)
{
    for (size_t i = 0; i < lines_.size(); i++)
    {
        lines_[i].rotate(rotation);
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        circles_[i].rotate(rotation);
    }
}

void Shape::scale(const float s)
{
    scale(Eigen::Vector2f(s, s));
}

void Shape::scale(const float x, const float y)
{
    scale(Eigen::Vector2f(x, y));
}

void Shape::scale(const Eigen::Vector2f scale_vec)
{
    for (size_t i = 0; i < lines_.size(); i++)
    {
        lines_[i].scale(scale_vec);
    }

    for (size_t i = 0; i < circles_.size(); i++)
    {
        circles_[i].scale(scale_vec);
    }
}

Shape operator*(const Eigen::Affine2f &affine, const Shape &s)
{
    std::vector<Line> lines;
    std::vector<Circle> circles;

    lines.reserve(s.getLines().size());
    for (const Line &line : s.getLines())
    {
        lines.push_back(affine * line);
    }

    circles.reserve(s.getCircles().size());
    for (const Circle &circle : s.getCircles())
    {
        circles.push_back(affine * circle);
    }

    return Shape(std::move(lines), std::move(circles));
}

std::ostream& operator<<(std::ostream &os, const Shape& s)
{
    os << "Shape(lines={";

    bool first = true;
    for (const Line &l : s.getLines())
    {
        if (!first) os << ", ";
        os << "(" << l.getPoint1X() << ", " << l.getPoint1Y() << ", " << l.getPoint2X() << ", " << l.getPoint2Y() << ")";
        first = false;
    }

    os << "}, circles={";

    first = true;
    for (const Circle &c : s.getCircles())
    {
        if (!first) os << ", ";
        os << "(" << c.getX() << ", " << c.getY() << ", " << c.getRadius() << ")";
        first = false;
    }

    os << "})";

    return os;
}

} // murosim::geometry
