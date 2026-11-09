// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/generators/intersection_generator.hpp"
#include "murosim/math/constants.hpp"

namespace murosimproc::generators
{

IntersectionGenerator::IntersectionGenerator(
    const size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    const float center_offset)
    : murosimproc::generators::Generator("intersection", min_node_count, node_radius, world_size, spacing)
    , center_offset_(std::min(std::max(0.0f, center_offset), 1.0f))
{
    checkWorldSize();
}

void IntersectionGenerator::checkWorldSize() const
{
    if(getWorldSize().min() <= getSpacing().max() * 3.0f)
    {
        throw std::invalid_argument("The min world size must be three times greater than the max space!");
    }
}

void IntersectionGenerator::generate(murosim::random::RandomGenerator::result_type seed, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> & /*circles*/, std::vector<murosim::navigation::Node> &nodes) const
{
    murosim::random::RandomGenerator random_generator(seed);

    const float world_size = getWorldSize()(random_generator);
    const float space = getSpacing()(random_generator);
    const float node_radius = getNodeRadius();

    const float offset_space = world_size - space * 3.0f;
    const float space_half = space * 0.5f;

    std::uniform_real_distribution<float> distribution(-center_offset_ * offset_space * 0.5f, center_offset_ * offset_space * 0.5f);
    const Eigen::Vector2f center = Eigen::Vector2f(world_size * 0.5f, world_size * 0.5f) +  Eigen::Vector2f(distribution(random_generator), distribution(random_generator));
    const Eigen::Vector2f corner_right_up = center + Eigen::Vector2f(space_half, space_half);
    const Eigen::Vector2f corner_right_down = center + Eigen::Vector2f(space_half, -space_half);
    const Eigen::Vector2f corner_left_up = center + Eigen::Vector2f(-space_half, space_half);
    const Eigen::Vector2f corner_left_down = center + Eigen::Vector2f(-space_half, -space_half);

    // borders
    lines.emplace_back(0.0f, 0.0f, world_size, 0.0f);
    lines.emplace_back(world_size, 0.0f, world_size, world_size);
    lines.emplace_back(world_size, world_size, 0.0f, world_size);
    lines.emplace_back(0.0f, world_size, 0.0f, 0.0f);

    lines.emplace_back(corner_right_down.x(), corner_right_down.y(), corner_right_down.x(), 0.0f);
    lines.emplace_back(corner_right_down.x(), corner_right_down.y(), world_size, corner_right_down.y());

    lines.emplace_back(corner_right_up.x(), corner_right_up.y(), corner_right_up.x(), world_size);
    lines.emplace_back(corner_right_up.x(), corner_right_up.y(), world_size, corner_right_up.y());

    lines.emplace_back(corner_left_down.x(), corner_left_down.y(), corner_left_down.x(), 0.0f);
    lines.emplace_back(corner_left_down.x(), corner_left_down.y(), 0.0f, corner_left_down.y());

    lines.emplace_back(corner_left_up.x(), corner_left_up.y(), corner_left_up.x(), world_size);
    lines.emplace_back(corner_left_up.x(), corner_left_up.y(), 0.0f, corner_left_up.y());

    nodes.emplace_back(murosim::navigation::Node(space_half, center.y(), node_radius, 0.0f, murosim::math::PI_4, 0, 0));
    nodes.emplace_back(murosim::navigation::Node(center.x(), space_half, node_radius, murosim::math::PI_2, murosim::math::PI_4, 0, 0));
    nodes.emplace_back(murosim::navigation::Node(world_size - space_half, center.y(), node_radius, murosim::math::PI, murosim::math::PI_4, 0, 0));
    nodes.emplace_back(murosim::navigation::Node(center.x(), world_size - space_half, node_radius, -murosim::math::PI_2, murosim::math::PI_4, 0, 0));
}


} // namespace murosimproc::generators
