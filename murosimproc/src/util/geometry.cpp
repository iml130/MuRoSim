// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/util/geometry.hpp"
#include "murosimproc/util/spatial_grid.hpp"

#include <murosim/collision/circle.hpp>
#include <murosim/geometry/primitives.hpp>
#include "murosim/math/constants.hpp"

namespace murosimproc::util::geometry
{

void addRectangleLinesFromPoints(std::vector<murosim::geometry::Line> &lines,  const Eigen::Vector2f point1, const Eigen::Vector2f point2, const float margin)
{
    const Eigen::Vector2f diff = point2 - point1;
    const float length = (diff).norm();
    const float length_half = length * 0.5f;
    const float angle = std::atan2(diff.y(), diff.x());

    const Eigen::Vector2f center = (point1 + point2) * 0.5f;
    const Eigen::Affine2f transform = murosim::geometry::primitives::pose(center, angle);

    const Eigen::Vector2f p1 = transform * Eigen::Vector2f( length_half + margin,  margin);
    const Eigen::Vector2f p2 = transform * Eigen::Vector2f( length_half + margin, -margin);
    const Eigen::Vector2f p3 = transform * Eigen::Vector2f(-length_half - margin, -margin);
    const Eigen::Vector2f p4 = transform * Eigen::Vector2f(-length_half - margin,  margin);

    lines.emplace_back(p1, p2);
    lines.emplace_back(p2, p3);
    lines.emplace_back(p3, p4);
    lines.emplace_back(p4, p1);
}

void addBorderLines(std::vector<murosim::geometry::Line> &lines, const float world_size)
{
    lines.emplace_back(0.0f, 0.0f, world_size, 0.0f);
    lines.emplace_back(world_size, 0.0f, world_size, world_size);
    lines.emplace_back(world_size, world_size, 0.0f, world_size);
    lines.emplace_back(0.0f, world_size, 0.0f, 0.0f);
}

void addBorderLines(std::vector<murosim::geometry::Line> &lines, const float world_size_x, const float world_size_y)
{
    lines.emplace_back(0.0f, 0.0f, world_size_x, 0.0f);
    lines.emplace_back(world_size_x, 0.0f, world_size_x, world_size_y);
    lines.emplace_back(world_size_x, world_size_y, 0.0f, world_size_y);
    lines.emplace_back(0.0f, world_size_y, 0.0f, 0.0f);
}

float getOrientationToCenterPoint(const Eigen::Vector2f &point, const Eigen::Vector2f &center)
{
    const Eigen::Vector2f diff = point - center;
    return std::atan2(diff.y(), diff.x());
}

std::vector<murosim::geometry::Circle> createCirclesInCircle(const murosim::geometry::Circle &circle, const float ratio_radius, const size_t number_of_tries, murosim::random::RandomGenerator &random_generator)
{
    const float radius_max = circle.getRadius() * 0.5f;
    const float radius_min = circle.getRadius() * (1.0f - std::clamp(ratio_radius, 0.1f, 1.0f)) * 0.5f;
    std::uniform_real_distribution<float> distribution_radius = std::uniform_real_distribution<float>(radius_min, radius_max);
    std::uniform_real_distribution<float> distribution_angle = std::uniform_real_distribution<float>(0.0f, murosim::math::PI * 2.0f);

    std::vector<murosim::geometry::Circle> circles;
    murosimproc::util::SpatialGrid grid(radius_max * 2.0f);

    for (size_t i = 0; i < number_of_tries; i++)
    {
        const float radius = distribution_radius(random_generator);
        const float angle = distribution_angle(random_generator);
        const float distance = std::uniform_real_distribution<float>(0.0f, circle.getRadius() - radius)(random_generator);

        const murosim::geometry::Circle &circle_new = murosim::geometry::Circle(circle.getPoint() + Eigen::Rotation2Df(angle) * Eigen::Vector2f(distance, 0.0f), radius);

        if (!grid.hasCollision(circle_new))
        {
            circles.push_back(circle_new);
            grid.insert(circle_new);
        }
    }

    return circles;
}

std::vector<murosim::geometry::Circle> createCirclesInCircle(const murosim::geometry::Circle &circle, const float ratio_radius, murosim::random::RandomGenerator &random_generator)
{
    return createCirclesInCircle(circle, ratio_radius, 5ul, random_generator);
}


std::vector<murosim::geometry::Line> createPolygonLinesInCircle(const murosim::geometry::Circle &circle, const float ratio_length, murosim::random::RandomGenerator &random_generator)
{
    return createPolygonLinesInCircle(circle, ratio_length, 3ul, 8ul, random_generator);
}

std::vector<murosim::geometry::Line> createPolygonLinesInCircle(const murosim::geometry::Circle &circle, const float ratio_length, const size_t segments_min, const size_t segments_max,  murosim::random::RandomGenerator &random_generator)
{
    assert(segments_min <= segments_max);
    assert(segments_min >= 3);
    assert(segments_max >= 3);

    size_t min = std::max(3ul, segments_min);
    size_t max = std::max(min, segments_max);

    const size_t segments = std::uniform_int_distribution<size_t>(min, max)(random_generator);

    std::uniform_real_distribution<float> distribution_pi = std::uniform_real_distribution<float>(-murosim::math::PI, murosim::math::PI);
    std::uniform_real_distribution<float> distribution_length = std::uniform_real_distribution<float>(circle.getRadius() * std::clamp(ratio_length, 0.0f, 1.0f), circle.getRadius());

    std::vector<float> angles;
    std::vector<Eigen::Vector2f> points;

    for (size_t i = 0; i < segments; i++)
    {
        angles.push_back(distribution_pi(random_generator));
    }

    std::sort(angles.begin(), angles.end());

    std::vector<murosim::geometry::Line> lines;
    const float angle_offset = distribution_pi(random_generator);

    for(const float angle: angles)
    {
        points.push_back(circle.getPoint() + Eigen::Rotation2Df(angle_offset + angle) * Eigen::Vector2f(distribution_length(random_generator), 0.0f));
    }

    for (size_t i = 1; i < points.size(); i++)
    {
        lines.emplace_back(points[i-1], points[i]);
    }

    lines.emplace_back(points[points.size()-1], points[0]);

    return lines;
}


std::vector<murosim::geometry::Circle> createCirclesInCircle(const murosim::geometry::Circle &circle, const float ratio_radius, const size_t target_count, const size_t max_attempts_per_circle, murosim::random::RandomGenerator &random_generator)
{
    const float radius_max = circle.getRadius() * 0.5f;
    const float radius_min = circle.getRadius() * (1.0f - std::clamp(ratio_radius, 0.1f, 1.0f)) * 0.5f;
    std::uniform_real_distribution<float> distribution_radius(radius_min, radius_max);
    std::uniform_real_distribution<float> distribution_angle(0.0f, murosim::math::PI * 2.0f);

    std::vector<murosim::geometry::Circle> circles;
    murosimproc::util::SpatialGrid grid(radius_max * 2.0f);

    for (size_t i = 0; i < target_count; i++)
    {
        for (size_t attempt = 0; attempt < max_attempts_per_circle; attempt++)
        {
            const float radius = distribution_radius(random_generator);
            const float angle = distribution_angle(random_generator);
            const float distance = std::uniform_real_distribution<float>(0.0f, circle.getRadius() - radius)(random_generator);

            const murosim::geometry::Circle circle_new(circle.getPoint() + Eigen::Rotation2Df(angle) * Eigen::Vector2f(distance, 0.0f), radius);

            if (!grid.hasCollision(circle_new))
            {
                circles.push_back(circle_new);
                grid.insert(circle_new);
                break;
            }
        }
    }

    return circles;
}

std::vector<murosim::geometry::Line> createRectangleLinesInCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator)
{
    std::uniform_real_distribution<float> aspect_dist(0.4f, 1.0f);
    std::uniform_real_distribution<float> scale_dist(0.5f, 0.95f);
    std::uniform_real_distribution<float> angle_dist(0.0f, murosim::math::PI * 2.0f);

    const float aspect = aspect_dist(random_generator);
    const float scale = scale_dist(random_generator);
    const float angle = angle_dist(random_generator);
    const float r = circle.getRadius();

    // Compute half-extents that fit inside the circle
    // For a rectangle with half-width hw and half-height hh inscribed in circle:
    // sqrt(hw^2 + hh^2) <= r
    // With aspect = hh/hw: hw = r / sqrt(1 + aspect^2)
    const float hw = r * scale / std::sqrt(1.0f + aspect * aspect);
    const float hh = hw * aspect;

    const Eigen::Vector2f center = circle.getPoint();
    const Eigen::Affine2f transform = murosim::geometry::primitives::pose(center, angle);

    const Eigen::Vector2f p1 = transform * Eigen::Vector2f( hw,  hh);
    const Eigen::Vector2f p2 = transform * Eigen::Vector2f( hw, -hh);
    const Eigen::Vector2f p3 = transform * Eigen::Vector2f(-hw, -hh);
    const Eigen::Vector2f p4 = transform * Eigen::Vector2f(-hw,  hh);

    std::vector<murosim::geometry::Line> lines;
    lines.emplace_back(p1, p2);
    lines.emplace_back(p2, p3);
    lines.emplace_back(p3, p4);
    lines.emplace_back(p4, p1);
    return lines;
}

std::vector<murosim::geometry::Line> createLShapeLinesInCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator)
{
    std::uniform_real_distribution<float> scale_dist(0.5f, 0.9f);
    std::uniform_real_distribution<float> angle_dist(0.0f, murosim::math::PI * 2.0f);
    std::uniform_real_distribution<float> notch_dist(0.3f, 0.6f);

    const float scale = scale_dist(random_generator);
    const float angle = angle_dist(random_generator);
    const float notch_x = notch_dist(random_generator);  // Where the L cuts inward (fraction of width)
    const float notch_y = notch_dist(random_generator);  // Where the L cuts upward (fraction of height)
    const float r = circle.getRadius();

    // L-shape in local coords, scaled to fit in circle
    // The L has 6 vertices. Max extent is 1.0 in each axis.
    // Scale factor to inscribe in circle: r * scale / sqrt(0.5^2 + 0.5^2) = r * scale / 0.707
    const float s = r * scale / std::sqrt(0.5f);

    // Define L-shape vertices in local [-0.5, 0.5] coords
    const float hx = 0.5f;
    const float hy = 0.5f;
    const float cx = -hx + notch_x;  // Notch x position
    const float cy = -hy + notch_y;  // Notch y position

    // 6 vertices of the L (counter-clockwise)
    std::array<Eigen::Vector2f, 6> local_pts = {
        Eigen::Vector2f(-hx, -hy),
        Eigen::Vector2f( hx, -hy),
        Eigen::Vector2f( hx,  cy),
        Eigen::Vector2f( cx,  cy),
        Eigen::Vector2f( cx,  hy),
        Eigen::Vector2f(-hx,  hy)
    };

    const Eigen::Vector2f center = circle.getPoint();
    const Eigen::Affine2f transform = murosim::geometry::primitives::pose(center, angle);

    std::vector<murosim::geometry::Line> lines;
    for (size_t i = 0; i < local_pts.size(); i++)
    {
        const Eigen::Vector2f p1 = transform * (local_pts[i] * s);
        const Eigen::Vector2f p2 = transform * (local_pts[(i + 1) % local_pts.size()] * s);
        lines.emplace_back(p1, p2);
    }
    return lines;
}

} // namespace murosimproc::util::geometry
