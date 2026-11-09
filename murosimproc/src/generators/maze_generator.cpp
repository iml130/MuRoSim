// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/generators/maze_generator.hpp"
#include "murosimproc/maze/algorithm.hpp"
#include "murosimproc/maze/converter.hpp"
#include "murosimproc/util/array2d.hpp"

#include <cmath>
#include <stdexcept>

constexpr size_t MINIMUM_MAZE_SIZE = 3;

namespace murosimproc::generators
{

MazeGenerator::MazeGenerator(
    const size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    RandomRange wall_thickness,
    RandomRange wall_removal_rate)
    : murosimproc::generators::Generator("maze", min_node_count, node_radius, world_size, spacing)
    , wall_thickness_(wall_thickness)
    , wall_removal_rate_(wall_removal_rate)
{
    checkWorldSize();
}

void MazeGenerator::checkWorldSize() const
{
    const size_t max_possible_maze_size = calculateWorldSize(
        getWorldSize().min(),
        getSpacing().max(),
        getWallThickness().max()
    );

    if (max_possible_maze_size < MINIMUM_MAZE_SIZE)
    {
        throw std::runtime_error(
            "the max possible world size is only '" +
            std::to_string(max_possible_maze_size) +
            "' but need at least a world size of " +
            std::to_string(MINIMUM_MAZE_SIZE)
        );
    }
}

size_t MazeGenerator::calculateWorldSize(
    const float world_size,
    const float space,
    const float wall_thickness) const
{
    return static_cast<size_t>(std::floor(world_size) / (space + wall_thickness));
}

const RandomRange& MazeGenerator::getWallThickness() const
{
    return wall_thickness_;
}

const RandomRange& MazeGenerator::getWallRemovalRate() const
{
    return wall_removal_rate_;
}

void MazeGenerator::generate(
    murosim::random::RandomGenerator::result_type seed,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    std::vector<murosim::navigation::Node> &nodes) const
{
    murosim::random::RandomGenerator random_generator(seed);

    const size_t min_node_count = getMinNodeCount();
    const float world_size = getWorldSize()(random_generator);
    const float space = getSpacing()(random_generator);
    const float wall_thickness = getWallThickness()(random_generator);
    const float space_ratio = space / (space + wall_thickness) - 0.5f;
    const size_t maze_size = calculateWorldSize(world_size, space, wall_thickness);

    // Generate maze using Prim's algorithm
    murosimproc::util::Array2d<uint8_t> maze = murosimproc::maze::generateMazePrim(
        maze_size,
        maze_size,
        random_generator
    );

    // Punch holes to create loops
    const float removal_rate = wall_removal_rate_(random_generator);
    murosimproc::maze::removeWalls(maze, removal_rate, random_generator);

    // Convert maze to world geometry
    // node_radius is spacing.min() * 0.5f in world coords; convert to grid coords for the converter
    const float world_scale = world_size / static_cast<float>(std::max(maze.sizeX(), maze.sizeY()) - 1);
    const float node_radius = getNodeRadius() / world_scale;
    murosimproc::maze::convertMazeToWorld(maze, lines, circles, nodes, min_node_count, space_ratio, node_radius, random_generator);

    // Scale the maze to the target world size

    for (auto &line   : lines)   { line.scale(world_scale); }
    for (auto &circle : circles) { circle.scale(world_scale); }
    for (auto &node   : nodes)   { node.scale(world_scale); }
}

} // namespace murosimproc::generators
