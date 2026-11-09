// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/generators/line_generator.hpp"

#include "murosimproc/util/geometry.hpp"
#include "murosim/math/constants.hpp"
#include <vector>

namespace murosimproc::generators
{

LineGenerator::LineGenerator(
    const size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    const float length_offset,
    const bool is_horizontal)
    : murosimproc::generators::Generator("line", std::max(static_cast<size_t>(2), min_node_count), node_radius, world_size, spacing)
    , length_offset_(std::min(std::max(0.0f, length_offset), 1.0f))
    , is_horizontal_(is_horizontal)
{
    checkWorldSize();
}

void LineGenerator::checkWorldSize() const
{
    const float space_max = getSpacing().max();

    if(getWorldSize().min() <= space_max * 2.0f)
    {
        throw std::invalid_argument("The min world size must be two times greater then the max space!");
    }

    const float node_count = static_cast<float>(getMinNodeCount());

    if (getWorldSize().min() <= node_count * space_max * 2.0f)
    {
        throw std::invalid_argument("The min world size does not fit the node space!");
    }
}

void LineGenerator::generate(murosim::random::RandomGenerator::result_type seed, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> & /*circles*/, std::vector<murosim::navigation::Node> &nodes) const
{
    murosim::random::RandomGenerator random_generator(seed);

    const float world_size = getWorldSize()(random_generator);
    const float space = getSpacing()(random_generator);
    const float node_radius = getNodeRadius();
    const float node_space = space * static_cast<float>(getMinNodeCount());

    // create the default horizontal line szenario
    const float offset_space = world_size - node_space * 2.0f;
    const float offset_length = std::uniform_real_distribution<float>(std::max((1.0f - length_offset_) * offset_space, node_space * 2.0f) , offset_space)(random_generator);
    Eigen::Vector2f center = Eigen::Vector2f(world_size, world_size) * 0.5f;
    Eigen::Vector2f point1 = center + Eigen::Vector2f(offset_length * 0.5f, 0.0f);
    Eigen::Vector2f point2 = center - Eigen::Vector2f(offset_length * 0.5f, 0.0f);

    if (!is_horizontal_)
    {
        const float collision_radius = 1.41421356f * node_space;
        const float range_max = world_size * 0.5f - collision_radius;
        const float range_min = range_max * (1.0f - length_offset_);
        std::uniform_real_distribution<float> distribution_range(range_min, range_max);
        auto sign = [&random_generator]() -> float {
            return std::uniform_int_distribution<int>(0, 1)(random_generator) ? 1.0f : -1.0f;
        };

        for (size_t i = 0; i < 100; i++)
        {
            const Eigen::Vector2f p1 = center + Eigen::Vector2f(sign() * distribution_range(random_generator), sign() * distribution_range(random_generator));
            const Eigen::Vector2f p2 = center + Eigen::Vector2f(sign() * distribution_range(random_generator), sign() * distribution_range(random_generator));
            const float length = (p2 - p1).norm();

            if (collision_radius * 2.0f < length)
            {
                point1 = p1;
                point2 = p2;
                center = (point1 + point2) * 0.5f;
                break;
            }
        }
    }

    murosimproc::util::geometry::addBorderLines(lines, world_size);
    murosimproc::util::geometry::addRectangleLinesFromPoints(lines, point1, point2, node_space);
    nodes.emplace_back(point1.x(), point1.y(), node_radius, murosimproc::util::geometry::getOrientationToCenterPoint(point1, center), murosim::math::PI_4, 0, 0);
    nodes.emplace_back(point2.x(), point2.y(), node_radius, murosimproc::util::geometry::getOrientationToCenterPoint(point2, center), murosim::math::PI_4, 0, 0);
}


} // namespace murosimproc::generators
