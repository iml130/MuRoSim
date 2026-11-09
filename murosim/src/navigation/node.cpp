// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/navigation/node.hpp"
#include "murosim/math/constants.hpp"

#include <algorithm>
#include <cmath>

namespace murosim::navigation
{

namespace
{

// std::fmod keeps the sign of its first argument, so wrapped angles stay in
// (-2*pi, 2*pi) rather than [0, 2*pi)
float wrapAngle(const float angle)
{
    return std::fmod(angle, 2.0f * murosim::math::PI);
}

} // namespace

// The default ("null") node intentionally uses a zero start_bias, unlike the
// other constructors which default it to 2*pi (full angular range).
Node::Node(): circle_(0.0f, 0.0f, 0.0f),
              start_angle_(0.0f),
              start_bias_(0.0f),
              id_(0),
              group_(0),
              goal_radius_(-1.0f)
{

}

Node::Node(const murosim::geometry::Circle &circle, const float start_angle, const float start_bias, const size_t id, const size_t group)
        : circle_(circle),
          start_angle_(wrapAngle(start_angle)),
          start_bias_(std::clamp(start_bias, 0.0f, 2.0f * murosim::math::PI)),
          id_(id),
          group_(group),
          goal_radius_(-1.0f)
{

}

Node::Node(const murosim::geometry::Circle &circle, const float start_angle, const float start_bias, const size_t id)
        : Node(circle, start_angle, start_bias, id, 0)
{

}

Node::Node(const murosim::geometry::Circle &circle, const float start_angle, const float start_bias)
        : Node(circle, start_angle, start_bias, 0, 0)
{

}

Node::Node(const murosim::geometry::Circle &circle, const float start_angle)
        : Node(circle, start_angle, 2.0f * murosim::math::PI, 0, 0)
{

}

Node::Node(const murosim::geometry::Circle &circle)
        : Node(circle, 0.0f, 2.0f * murosim::math::PI, 0, 0)
{

}

Node::Node(const float x, const float y, const float radius, const float start_angle, const float start_bias, const size_t id, const size_t group)
        : Node(murosim::geometry::Circle(x, y, radius), start_angle, start_bias, id, group)
{

}

Node::Node(const float x, const float y, const float radius, const float start_angle, const float start_bias, const size_t id)
        : Node(murosim::geometry::Circle(x, y, radius), start_angle, start_bias, id, 0)
{

}

Node::Node(const float x, const float y, const float radius, const float start_angle, const float start_bias)
        : Node(murosim::geometry::Circle(x, y, radius), start_angle, start_bias, 0, 0)
{

}

Node::Node(const float x, const float y, const float radius, const float start_angle)
        : Node(murosim::geometry::Circle(x, y, radius), start_angle, 2.0f * murosim::math::PI, 0, 0)
{

}

Node::Node(const float x, const float y, const float radius)
        : Node(murosim::geometry::Circle(x, y, radius), 0.0f, 2.0f * murosim::math::PI, 0, 0)
{

}

void Node::setGoalRadius(const float goal_radius)
{
    goal_radius_ = (goal_radius <= 0)? circle_.getRadius(): goal_radius;
}

void Node::setId(const size_t id)
{
    id_ = id;
}

size_t Node::getId() const
{
    return id_;
}

void Node::setGroup(const size_t group)
{
    group_ = group;
}

size_t Node::getGroup() const
{
    return group_;
}

murosim::geometry::Circle Node::getCircle() const
{
    return circle_;
}

Eigen::Vector2f Node::getPoint() const
{
    return circle_.getPoint();
}

float Node::getX() const
{
    return circle_.getX();
}

float Node::getY() const
{
    return circle_.getY();
}

float Node::getRadius() const
{
    return circle_.getRadius();
}

Eigen::Vector2f Node::getOrientation() const
{
    return Eigen::Rotation2Df(start_angle_) * Eigen::Vector2f(1.0f, 0.0f);
}

float Node::getStartAngle() const
{
    return start_angle_;
}

float Node::getStartBias() const
{
    return start_bias_;
}

float Node::getGoalRadius() const
{
    // fall back to the node circle when no explicit goal radius was set
    // (goals created via Robot::setGoal(x, y, radius) or Node copies never
    // pass through setGoalRadius and would keep the -1 sentinel)
    return (goal_radius_ > 0.0f) ? goal_radius_ : circle_.getRadius();
}

void Node::transform(const Eigen::Affine2f &affine)
{
    circle_.transform(affine);
    // keep the stored orientation in sync with the affine's rotation
    start_angle_ = wrapAngle(start_angle_ + Eigen::Rotation2Df(affine.rotation()).angle());
}

void Node::translate(const float x, const float y)
{
    circle_.translate(x, y);
}

void Node::translate(const Eigen::Vector2f &translation)
{
    circle_.translate(translation);
}

void Node::rotate(const Eigen::Rotation2Df &rotation)
{
    circle_.rotate(rotation);
    start_angle_ = wrapAngle(start_angle_ + rotation.angle());
}

void Node::rotate(const float angle)
{
    circle_.rotate(angle);
    start_angle_ = wrapAngle(start_angle_ + angle);
}

void Node::setStartAngle(const float angle)
{
    start_angle_ = wrapAngle(angle);
}

void Node::setStartBias(const float bias)
{
    start_bias_ = std::clamp(bias, 0.0f, 2.0f * murosim::math::PI);
}

void Node::scale(const float x, const float y)
{
    scale(Eigen::Vector2f(x, y));
}

void Node::scale(const Eigen::Vector2f &scale)
{
    circle_.scale(scale);
    // Keep the heading consistent with the position by mapping the orientation
    // vector through the same scale (mirrors it for a negative factor, leaves it
    // unchanged for a positive uniform scale), matching transform()/rotate().
    start_angle_ = wrapAngle(std::atan2(scale.y() * std::sin(start_angle_),
                                        scale.x() * std::cos(start_angle_)));
}

void Node::scale(const float factor)
{
    scale(Eigen::Vector2f(factor, factor));
}

bool Node::operator<(const Node &node) const
{
    return id_ < node.id_;
}

Node operator*(const Eigen::Affine2f &affine, const Node &n)
{
    const float start_angle = wrapAngle(n.start_angle_ + Eigen::Rotation2Df(affine.rotation()).angle());
    Node result(affine * n.circle_, start_angle, n.start_bias_, n.id_, n.group_);
    result.goal_radius_ = n.goal_radius_;
    return result;
}

} // namespace murosim::navigation
