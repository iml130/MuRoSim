// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/generators/generator.hpp"
#include "murosim/io/data_container.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace murosimproc::generators
{

Generator::Generator(const std::string &name, size_t min_node_count, float node_radius)
    : name_(name)
    , node_radius_(node_radius)
    , spacing_(1.0f)
    , world_size_(10.0f)
    , min_node_count_(min_node_count)
{
    validateConfiguration();
}

Generator::Generator(const std::string &name, size_t min_node_count, float node_radius, RandomRange world_size, RandomRange spacing)
    : name_(name)
    , node_radius_(node_radius)
    , spacing_(spacing)
    , world_size_(world_size)
    , min_node_count_(min_node_count)
{
    validateConfiguration();
}

std::string Generator::getName() const
{
    return name_;
}

std::string Generator::getNameWithSeed(murosim::random::RandomGenerator::result_type seed) const
{
    constexpr size_t seed_size = sizeof(murosim::random::RandomGenerator::result_type) * 2 + 2;
    std::stringstream ss;
    ss << name_ << "_" << std::setw(seed_size) << std::setfill('0') << seed;
    return ss.str();
}

void Generator::validateConfiguration() const
{
    if (min_node_count_ < 2) [[unlikely]] {
        throw std::invalid_argument(
            "min_node_count must be >= 2 (got " +
            std::to_string(min_node_count_) + ")");
    }

    if (world_size_.min() <= 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "min_world_size must be > 0 (got " +
            std::to_string(world_size_.min()) + ")");
    }

    if (node_radius_ <= 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "node_radius must be > 0 (got " +
            std::to_string(node_radius_) + ")");
    }

    if (spacing_.min() < node_radius_ * 2.0f) [[unlikely]] {
    throw std::invalid_argument(
        "spacing.min() (" + std::to_string(spacing_.min()) +
        ") must be >= 2 * node_radius (" + std::to_string(node_radius_ * 2.0f) +
        ") so nodes fit within corridors");
    }

    if (spacing_.min() > world_size_.min()) [[unlikely]] {
        throw std::invalid_argument(
            "min_spacing (" + std::to_string(spacing_.min()) +
            ") must be <= min_world_size (" +
            std::to_string(world_size_.min()) + ")");
    }

    // Feasibility check: can min_node_count nodes fit in the world?
    // Use worst case: smallest world, largest spacing
    const float worst_world_size = world_size_.min();
    const float worst_spacing = spacing_.max();
    const float usable_area = worst_world_size * worst_world_size;
    const float area_per_node = worst_spacing * worst_spacing;
    const size_t estimated_capacity = static_cast<size_t>(usable_area / area_per_node);

    if (min_node_count_ > estimated_capacity) [[unlikely]] {
        throw std::invalid_argument(
            "min_node_count (" + std::to_string(min_node_count_) +
            ") is infeasible for min_world_size (" +
            std::to_string(worst_world_size) +
            ") and max_spacing (" +
            std::to_string(worst_spacing) +
            ") — estimated capacity is ~" +
            std::to_string(estimated_capacity) + " nodes");
    }
}

const RandomRange& Generator::getSpacing() const
{
    return spacing_;
}

const RandomRange& Generator::getWorldSize() const
{
    return world_size_;
}

size_t Generator::getMinNodeCount() const
{
    return min_node_count_;
}

float Generator::getNodeRadius() const
{
    return node_radius_;
}

murosim::io::DataContainer Generator::operator()(murosim::random::RandomGenerator::result_type seed) const
{
    constexpr size_t MAX_RETRIES = 100;

    for (size_t attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        std::vector<murosim::geometry::Line> lines;
        std::vector<murosim::geometry::Circle> circles;
        std::vector<murosim::navigation::Node> nodes;

        generate(seed + attempt, lines, circles, nodes);

        if (nodes.size() >= min_node_count_) {
            return murosim::io::DataContainer(
                std::move(lines),
                std::move(circles),
                std::move(nodes),
                getNameWithSeed(seed));
        }
    }

    throw std::runtime_error(
        "Generator '" + name_ + "' failed to produce at least " +
        std::to_string(min_node_count_) + " nodes after " +
        std::to_string(MAX_RETRIES) + " attempts");
}

murosim::io::DataContainer Generator::operator()(murosim::random::RandomGenerator &random_generator) const
{
    constexpr size_t MAX_RETRIES = 100;

    for (size_t attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        auto seed = random_generator();
        std::vector<murosim::geometry::Line> lines;
        std::vector<murosim::geometry::Circle> circles;
        std::vector<murosim::navigation::Node> nodes;

        generate(seed, lines, circles, nodes);

        if (nodes.size() >= min_node_count_) {
            return murosim::io::DataContainer(
                std::move(lines),
                std::move(circles),
                std::move(nodes),
                getNameWithSeed(seed));
        }
    }

    throw std::runtime_error(
        "Generator '" + name_ + "' failed to produce at least " +
        std::to_string(min_node_count_) + " nodes after " +
        std::to_string(MAX_RETRIES) + " attempts");
}

} // namespace murosimproc::generators
