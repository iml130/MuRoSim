// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/models/world.hpp"

#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"
#include <stdexcept>

namespace {

void checkSpaceAndWorldSize(const float world_size, const float space)
{
    if (world_size <= 0.0f)
    {
        throw std::invalid_argument("World size must be greater than 0.");
    }

    if (space <= 0.0f)
    {
        throw std::invalid_argument("Space must be greater than 0.");
    }

    if (world_size < space)
    {
        throw std::invalid_argument("World size must be greater than space.");
    }
}

} // namespace

namespace murosim::models::world
{

murosim::io::DataContainer line(const float world_size, const float space)
{
    std::vector<murosim::geometry::Line> lines;
    std::vector<murosim::geometry::Circle> circles;
    std::vector<murosim::navigation::Node> nodes;

    checkSpaceAndWorldSize(world_size, space);

    if(world_size < 2.0f * space)
    {
        throw std::invalid_argument("The world size must be 2 times greater than the space.");
    }

    const float half_world_size = world_size * 0.5f;
    const float half_space = space * 0.5f;

    murosim::geometry::primitives::square(world_size, murosim::geometry::primitives::pose(half_world_size, half_world_size, 0.0f)).copyTo(lines, circles); // borders
    const float y_upper = half_world_size + half_space;
    const float y_lower = half_world_size - half_space;

    lines.emplace_back(0.0f, y_upper, world_size, y_upper);
    lines.emplace_back(0.0f, y_lower, world_size, y_lower);

    nodes.emplace_back(half_space, half_world_size, half_space, 0.0f, murosim::math::PI_4, 0, 0);
    nodes.emplace_back(world_size - half_space, half_world_size, half_space, murosim::math::PI, murosim::math::PI_4, 0, 0);

    return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), "line");
}

murosim::io::DataContainer empty(const float world_size, const float space)
{
    std::vector<murosim::geometry::Line> lines;
    std::vector<murosim::geometry::Circle> circles;
    std::vector<murosim::navigation::Node> nodes;

    checkSpaceAndWorldSize(world_size, space);

    if(world_size < 3.0f * space)
    {
        throw std::invalid_argument("The world size must be 3 times greater than the space.");
    }

    const float half_space = space * 0.5f;
    const float half_world_size = world_size * 0.5f;

    murosim::geometry::primitives::square(world_size, murosim::geometry::primitives::pose(half_world_size, half_world_size, 0.0f)).copyTo(lines, circles);

    nodes.emplace_back(half_world_size, half_world_size, half_space, 0.0f, murosim::math::PI, 0, 0); // Center
    nodes.emplace_back(half_world_size, world_size - half_space, half_space, -murosim::math::PI_2, murosim::math::PI_2, 0, 0); // North
    nodes.emplace_back(world_size - half_space, world_size - half_space, half_space, -0.75f * murosim::math::PI, murosim::math::PI_4, 0, 0); // North East
    nodes.emplace_back(world_size - half_space, half_world_size, half_space, murosim::math::PI, murosim::math::PI_2, 0, 0); // East
    nodes.emplace_back(world_size - half_space, half_space, half_space, 0.75f * murosim::math::PI, murosim::math::PI_4, 0, 0); // South East
    nodes.emplace_back(half_world_size, half_space, half_space, murosim::math::PI_2, murosim::math::PI_2, 0, 0); // South
    nodes.emplace_back(half_space, half_space, half_space, murosim::math::PI_4, murosim::math::PI_4, 0, 0); // South West
    nodes.emplace_back(half_space, half_world_size, half_space, 0.0f, murosim::math::PI_2, 0, 0); // West
    nodes.emplace_back(half_space, world_size - half_space, half_space, -murosim::math::PI_4, murosim::math::PI_4, 0, 0); // North West

    return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), "empty");
}

murosim::io::DataContainer intersection(const float world_size, const float space)
{
    std::vector<murosim::geometry::Line> lines;
    std::vector<murosim::geometry::Circle> circles;
    std::vector<murosim::navigation::Node> nodes;

    checkSpaceAndWorldSize(world_size, space);

    if(world_size < 3.0f * space)
    {
        throw std::invalid_argument("The world size must be 3 times greater than the space.");
    }

    const float half_space = space * 0.5f;
    const float half_world_size = world_size * 0.5f;
    const Eigen::Vector2f center = Eigen::Vector2f(half_world_size, half_world_size);
    const Eigen::Vector2f corner_right_up = center + Eigen::Vector2f(half_space, half_space);
    const Eigen::Vector2f corner_right_down = center + Eigen::Vector2f(half_space, -half_space);
    const Eigen::Vector2f corner_left_up = center + Eigen::Vector2f(-half_space, half_space);
    const Eigen::Vector2f corner_left_down = center + Eigen::Vector2f(-half_space, -half_space);

    murosim::geometry::primitives::square(world_size, murosim::geometry::primitives::pose(half_world_size, half_world_size, 0.0f)).copyTo(lines, circles);

    lines.emplace_back(corner_right_down.x(), corner_right_down.y(), corner_right_down.x(), 0.0f);
    lines.emplace_back(corner_right_down.x(), corner_right_down.y(), world_size, corner_right_down.y());
    lines.emplace_back(corner_right_up.x(), corner_right_up.y(), corner_right_up.x(), world_size);
    lines.emplace_back(corner_right_up.x(), corner_right_up.y(), world_size, corner_right_up.y());
    lines.emplace_back(corner_left_down.x(), corner_left_down.y(), corner_left_down.x(), 0.0f);
    lines.emplace_back(corner_left_down.x(), corner_left_down.y(), 0.0f, corner_left_down.y());
    lines.emplace_back(corner_left_up.x(), corner_left_up.y(), corner_left_up.x(), world_size);
    lines.emplace_back(corner_left_up.x(), corner_left_up.y(), 0.0f, corner_left_up.y());

    nodes.emplace_back(murosim::navigation::Node(center.x(), world_size - half_space, half_space, -murosim::math::PI_2, murosim::math::PI_4, 0, 0)); // North
    nodes.emplace_back(murosim::navigation::Node(world_size - half_space, center.y(), half_space, murosim::math::PI, murosim::math::PI_4, 0, 0)); // East
    nodes.emplace_back(murosim::navigation::Node(center.x(), half_space, half_space, murosim::math::PI_2, murosim::math::PI_4, 0, 0)); // South
    nodes.emplace_back(murosim::navigation::Node(half_space, center.y(), half_space, 0.0f, murosim::math::PI_4, 0, 0)); // West

    return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), "intersection");
}

murosim::io::DataContainer roundabout(const float world_size, const float space)
{
    std::vector<murosim::geometry::Line> lines;
    std::vector<murosim::geometry::Circle> circles;
    std::vector<murosim::navigation::Node> nodes;

    checkSpaceAndWorldSize(world_size, space);

    if(world_size < 5.0f * space)
    {
        throw std::invalid_argument("The world size must be 5 times greater than the space.");
    }

    const float half_space = space * 0.5f;
    const float half_world_size = world_size * 0.5f;
    const Eigen::Vector2f center = Eigen::Vector2f(half_world_size, half_world_size);

    circles.emplace_back(murosim::geometry::Circle(center.x(), center.y(), half_space));

    murosim::geometry::primitives::square(world_size, murosim::geometry::primitives::pose(half_world_size, half_world_size, 0.0f)).copyTo(lines, circles);

    std::vector<murosim::geometry::Line> walls;

    const Eigen::Vector2f p1 = Eigen::Vector2f(half_world_size, half_space);
    const Eigen::Vector2f p2 = Eigen::Vector2f(1.5f * space, half_space);
    const Eigen::Vector2f p3 = Eigen::Vector2f(1.25f * space, 1.25f * space);
    const Eigen::Vector2f p4 = Eigen::Vector2f(half_space, 1.5f * space);
    const Eigen::Vector2f p5 = Eigen::Vector2f(half_space, half_world_size);

    const murosim::geometry::Line l1 = murosim::geometry::Line(p1, p2);
    const murosim::geometry::Line l2 = murosim::geometry::Line(p2, p3);
    const murosim::geometry::Line l3 = murosim::geometry::Line(p3, p4);
    const murosim::geometry::Line l4 = murosim::geometry::Line(p4, p5);

    const Eigen::Affine2f t1 = murosim::geometry::primitives::pose(center, 0.0f);
    const Eigen::Affine2f t2 = murosim::geometry::primitives::pose(center, murosim::math::PI_2);
    const Eigen::Affine2f t3 = murosim::geometry::primitives::pose(center, murosim::math::PI);
    const Eigen::Affine2f t4 = murosim::geometry::primitives::pose(center, -murosim::math::PI_2);

    lines.emplace_back(t1 * l1);
    lines.emplace_back(t1 * l2);
    lines.emplace_back(t1 * l3);
    lines.emplace_back(t1 * l4);

    lines.emplace_back(t2 * l1);
    lines.emplace_back(t2 * l2);
    lines.emplace_back(t2 * l3);
    lines.emplace_back(t2 * l4);

    lines.emplace_back(t3 * l1);
    lines.emplace_back(t3 * l2);
    lines.emplace_back(t3 * l3);
    lines.emplace_back(t3 * l4);

    lines.emplace_back(t4 * l1);
    lines.emplace_back(t4 * l2);
    lines.emplace_back(t4 * l3);
    lines.emplace_back(t4 * l4);

    nodes.emplace_back(murosim::navigation::Node(center.x(), world_size - half_space, half_space, -murosim::math::PI_2, murosim::math::PI_4, 0, 0)); // North
    nodes.emplace_back(murosim::navigation::Node(world_size - half_space, center.y(), half_space, murosim::math::PI, murosim::math::PI_4, 0, 0)); // East
    nodes.emplace_back(murosim::navigation::Node(center.x(), half_space, half_space, murosim::math::PI_2, murosim::math::PI_4, 0, 0)); // South
    nodes.emplace_back(murosim::navigation::Node(half_space, center.y(), half_space, 0.0f, murosim::math::PI_4, 0, 0)); // West

    return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), "roundabout");
}

murosim::io::DataContainer curve(const float world_size, const float space)
{
    std::vector<murosim::geometry::Line> lines;
    std::vector<murosim::geometry::Circle> circles;
    std::vector<murosim::navigation::Node> nodes;

    checkSpaceAndWorldSize(world_size, space);

    if(world_size < 5.0f * space)
    {
        throw std::invalid_argument("The world size must be 5 times greater than the space.");
    }

    const float half_space = space * 0.5f;
    const float half_world_size = world_size * 0.5f;
    const Eigen::Vector2f center = Eigen::Vector2f(half_world_size, half_world_size);
    const Eigen::Vector2f center_c1 = Eigen::Vector2f(center.x() - space, center.y() + space); // center circle left up
    const Eigen::Vector2f center_c2 = Eigen::Vector2f(center.x() + space, center.y() - space); // center circle right down

    murosim::geometry::primitives::square(world_size, murosim::geometry::primitives::pose(half_world_size, half_world_size, 0.0f)).copyTo(lines, circles);

    circles.emplace_back(murosim::geometry::Circle(center_c1, half_space));
    circles.emplace_back(murosim::geometry::Circle(center_c2, half_space));

    lines.emplace_back(murosim::geometry::Line(center_c1.x(), center_c1.y() + half_space, world_size, center_c1.y() + half_space));
    lines.emplace_back(murosim::geometry::Line(center_c1.x(), center_c1.y() - half_space, center_c2.x() + 1.5f * space, center_c1.y() - half_space));
    lines.emplace_back(murosim::geometry::Line(center_c2.x() + 1.5f * space, center_c1.y() - half_space, center_c2.x() + 1.5f * space, center_c2.y() - 1.5f * space));
    lines.emplace_back(murosim::geometry::Line(center_c2.x() + 1.5f * space, center_c2.y() - 1.5f * space, 0.0f, center_c2.y() - 1.5f * space));

    lines.emplace_back(murosim::geometry::Line(0.0f, center_c2.y() - half_space, center_c2.x(), center_c2.y() - half_space));
    lines.emplace_back(murosim::geometry::Line(center_c2.x(), center_c2.y() + half_space, center_c1.x() - 1.5f * space, center_c2.y() + half_space));
    lines.emplace_back(murosim::geometry::Line(center_c1.x() - 1.5f * space, center_c2.y() + half_space, center_c1.x() - 1.5f * space, center_c1.y() + 1.5f * space));
    lines.emplace_back(murosim::geometry::Line(center_c1.x() - 1.5f * space, center_c1.y() + 1.5f * space, world_size, center_c1.y() + 1.5f * space));

    return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), "curve");
}

murosim::io::DataContainer bone(const float world_size, const float space)
{
    std::vector<murosim::geometry::Line> lines;
    std::vector<murosim::geometry::Circle> circles;
    std::vector<murosim::navigation::Node> nodes;

    checkSpaceAndWorldSize(world_size, space);

    if(world_size < 3.0f * space)
    {
        throw std::invalid_argument("The world size must be 3 times greater than the space.");
    }

    const float half_space = space * 0.5f;
    const float half_world_size = world_size * 0.5f;

    const Eigen::Vector2f w1p1 = Eigen::Vector2f(space, 0.0f);
    const Eigen::Vector2f w1p2 = Eigen::Vector2f(space, half_world_size - half_space);
    const Eigen::Vector2f w1p3 = Eigen::Vector2f(world_size - space, half_world_size - half_space);
    const Eigen::Vector2f w1p4 = Eigen::Vector2f(world_size - space, 0.0f);

    const Eigen::Vector2f w2p1 = Eigen::Vector2f(space, world_size);
    const Eigen::Vector2f w2p2 = Eigen::Vector2f(space, half_world_size + half_space);
    const Eigen::Vector2f w2p3 = Eigen::Vector2f(world_size - space, half_world_size + half_space);
    const Eigen::Vector2f w2p4 = Eigen::Vector2f(world_size - space, world_size);

    murosim::geometry::primitives::square(world_size, murosim::geometry::primitives::pose(half_world_size, half_world_size, 0.0f)).copyTo(lines, circles);

    lines.emplace_back(w1p1, w1p2);
    lines.emplace_back(w1p2, w1p3);
    lines.emplace_back(w1p3, w1p4);

    lines.emplace_back(w2p1, w2p2);
    lines.emplace_back(w2p2, w2p3);
    lines.emplace_back(w2p3, w2p4);

    nodes.emplace_back(half_space, half_space, half_space, murosim::math::PI_2, murosim::math::PI_4, 0, 0);
    nodes.emplace_back(half_space, world_size - half_space, half_space, -murosim::math::PI_2, murosim::math::PI_4, 0, 0);
    nodes.emplace_back(world_size - half_space, world_size - half_space, half_space, -murosim::math::PI_2, murosim::math::PI_4, 0, 0);
    nodes.emplace_back(world_size - half_space, half_space, half_space, murosim::math::PI_2, murosim::math::PI_4, 0, 0);

    return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), "bone");
}

} // namespace murosim::models::world
