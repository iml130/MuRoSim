// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_NAVIGATION_NODE_HPP_
#define MUROSIM_NAVIGATION_NODE_HPP_

#include "murosim/geometry/circle.hpp"
#include <vector>

namespace murosim::navigation
{

/// A named navigation point in a murosim::navigation::Container: a circular
/// area (position + radius) a robot can spawn at, drive through as a waypoint,
/// or use as a goal, together with a preferred start heading/bias and an
/// optional group id for clustered start/goal placement. Angles are in
/// radians. Node ids are assigned/used by Container; group ids are remapped
/// to compact bucket indices by Container::groupNodes().
class Node
{
private:
    murosim::geometry::Circle circle_;
    float start_angle_;
    float start_bias_;
    size_t id_;
    size_t group_;
    float goal_radius_;

public:
    /// Constructs the default ("null") node: zero-radius circle at the
    /// origin, zero start angle, and (unlike the other constructors) a
    /// zero start bias rather than the full 2*pi angular range.
    Node();
    /// Constructs a node from an explicit circle, start angle [rad], start
    /// bias [rad] (angular range around start_angle a robot may spawn
    /// within, clamped to [0, 2*pi]), id and group.
    Node(const murosim::geometry::Circle &circle, const float start_angle, const float start_bias, const size_t id, const size_t group);
    /// Same as the 5-argument constructor with group defaulted to 0 (ungrouped).
    Node(const murosim::geometry::Circle &circle, const float start_angle, const float start_bias, const size_t id);
    /// Same as above with id defaulted to 0 and group defaulted to 0.
    Node(const murosim::geometry::Circle &circle, const float start_angle, const float start_bias);
    /// Same as above with start_bias defaulted to the full 2*pi angular range.
    Node(const murosim::geometry::Circle &circle, const float start_angle);
    /// Same as above with start_angle defaulted to 0.
    explicit Node(const murosim::geometry::Circle &circle);
    /// Same as the circle-based 5-argument constructor, with the circle given as (x, y, radius).
    Node(const float x, const float y, const float radius, const float start_angle, const float start_bias, const size_t id, const size_t group);
    /// Same as above with group defaulted to 0.
    Node(const float x, const float y, const float radius, const float start_angle, const float start_bias, const size_t id);
    /// Same as above with id defaulted to 0 and group defaulted to 0.
    Node(const float x, const float y, const float radius, const float start_angle, const float start_bias);
    /// Same as above with start_bias defaulted to the full 2*pi angular range.
    Node(const float x, const float y, const float radius, const float start_angle);
    /// Same as above with start_angle defaulted to 0.
    Node(const float x, const float y, const float radius);

    /// Sets the node id.
    void setId(const size_t id);
    /// Sets the group id.
    void setGroup(const size_t group);
    /// Sets an explicit goal radius; a value <= 0 resets it to fall back to the node circle's radius (see getGoalRadius()).
    void setGoalRadius(const float radius);
    /// Sets the start angle [rad], wrapped into (-2*pi, 2*pi).
    void setStartAngle(const float angle);
    /// Sets the start bias [rad], clamped to [0, 2*pi].
    void setStartBias(const float bias);

    /// Returns the node id.
    size_t getId() const;
    /// Returns the group id (a compact bucket index once owned by a Container).
    size_t getGroup() const;
    /// Returns the node's area as a circle.
    murosim::geometry::Circle getCircle() const;
    /// Returns the node's center point.
    Eigen::Vector2f getPoint() const;
    /// Returns the node's center x coordinate.
    float getX() const;
    /// Returns the node's center y coordinate.
    float getY() const;
    /// Returns the node's radius.
    float getRadius() const;
    /// Returns the unit heading vector for the node's start angle.
    Eigen::Vector2f getOrientation() const;
    /// Returns the start angle [rad].
    float getStartAngle() const;
    /// Returns the start bias [rad].
    float getStartBias() const;
    /// Returns the explicit goal radius if one was set via setGoalRadius(), otherwise the node circle's radius.
    float getGoalRadius() const;

    /// Applies an affine transform to the node's circle and keeps the start
    /// angle in sync with the transform's rotation component.
    void transform(const Eigen::Affine2f &affine);
    /// Rotates the node's circle about the origin by angle radians and advances the start angle by the same amount.
    void rotate(const float angle);
    /// Rotates the node's circle about the origin and advances the start angle by the rotation's angle.
    void rotate(const Eigen::Rotation2Df &rotation);
    /// Translates the node's circle by (x, y).
    void translate(const float x, const float y);
    /// Translates the node's circle.
    void translate(const Eigen::Vector2f &translation);
    /// Uniformly scales the node's circle about the origin by scale.
    void scale(const float factor);
    /// Scales the node's circle about the origin per axis, remapping the start
    /// angle's heading vector through the same per-axis scale.
    void scale(const float x, const float y);
    /// Scales the node's circle about the origin per axis, remapping the start
    /// angle's heading vector through the same per-axis scale.
    void scale(const Eigen::Vector2f &scale);

    /// Orders nodes by id; used to keep a Container's node list sorted.
    bool operator<(const Node &node) const;

    /// Returns a copy of n with affine applied (see transform(const Eigen::Affine2f&)).
    friend Node operator*(const Eigen::Affine2f &affine, const Node &n);
};

} // namespace murosim::navigation

#endif // MUROSIM_NAVIGATION_NODE_HPP_
