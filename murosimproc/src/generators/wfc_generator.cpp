// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/generators/wfc_generator.hpp"
#include "murosimproc/wfc/wfc.hpp"

namespace murosimproc::generators
{

WfcGenerator::WfcGenerator(
    size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    murosimproc::wfc::preset::PresetType preset_type,
    RandomRange obstacle_size,
    RandomRange obstacle_density,
    murosimproc::obstacle::ShapeDistribution shape_distribution)
    : murosimproc::generators::Generator("wfc", min_node_count, node_radius, world_size, spacing)
    , tiles_(murosimproc::wfc::preset::getPreset(preset_type))
    , obstacle_density_(obstacle_density)
    , obstacle_size_(obstacle_size)
    , shape_distribution_(shape_distribution)
{
    validateWFCConfig();
}

WfcGenerator::WfcGenerator(
    size_t min_node_count,
    float node_radius,
    RandomRange world_size,
    RandomRange spacing,
    const murosimproc::wfc::Tiles &tiles,
    RandomRange obstacle_size,
    RandomRange obstacle_density,
    murosimproc::obstacle::ShapeDistribution shape_distribution)
    : murosimproc::generators::Generator("wfc", min_node_count, node_radius, world_size, spacing)
    , tiles_(tiles)
    , obstacle_density_(obstacle_density)
    , obstacle_size_(obstacle_size)
    , shape_distribution_(shape_distribution)
{
    validateWFCConfig();
}

size_t WfcGenerator::calculateGridSize(
    const float world_size,
    const float min_spacing) const
{
    // Get pattern size from tiles (assumes square patterns)
    const size_t pattern_size = tiles_.getPatterns()[0].sizeX();

    // Calculate how many patterns fit in the world
    // Formula: grid_size = world_size / (min_spacing * pattern_size)
    // This ensures: cell_size = world_size / (grid_size * pattern_size) >= min_spacing
    return static_cast<size_t>(
        std::floor(world_size / (min_spacing * static_cast<float>(pattern_size)))
    );
}

void WfcGenerator::validateWFCConfig() const
{
    // Validate tiles
    if (tiles_.getPatterns().empty()) [[unlikely]] {
        throw std::invalid_argument("Tiles must have at least one pattern");
    }

    if (tiles_.getWeights().empty()) [[unlikely]] {
        throw std::invalid_argument("Tiles must have at least one weight");
    }

    // Validate that all patterns have the same size
    const size_t pattern_size_x = tiles_.getPatterns()[0].sizeX();
    const size_t pattern_size_y = tiles_.getPatterns()[0].sizeY();

    if (pattern_size_x != pattern_size_y) [[unlikely]] {
        throw std::invalid_argument(
            "WFC patterns must be square (got " +
            std::to_string(pattern_size_x) + "x" +
            std::to_string(pattern_size_y) + ")");
    }

    if (pattern_size_x == 0) [[unlikely]] {
        throw std::invalid_argument("Pattern size must be > 0");
    }

    // A negative density would make the expected obstacle count negative, and
    // converting that to size_t in placeObstacles() is undefined behaviour.
    if (obstacle_density_.min() < 0.0f) [[unlikely]] {
        throw std::invalid_argument(
            "obstacle_density min must be >= 0 (got " +
            std::to_string(obstacle_density_.min()) + ")");
    }

    // Calculate minimum possible grid size with worst-case parameters
    const size_t min_grid_size = calculateGridSize(
        getWorldSize().min(),
        getSpacing().max()  // Use max spacing as worst case (larger spacing = smaller grid)
    );

    if (min_grid_size < defaults::WFC_MIN_GRID_SIZE) [[unlikely]] {
        const float required_world_size =
            getSpacing().max() * static_cast<float>(pattern_size_x) * defaults::WFC_MIN_GRID_SIZE;

        throw std::invalid_argument(
            "WFC grid too small (" + std::to_string(min_grid_size) +
            "x" + std::to_string(min_grid_size) +
            "). Need at least " + std::to_string(defaults::WFC_MIN_GRID_SIZE) +
            "x" + std::to_string(defaults::WFC_MIN_GRID_SIZE) +
            ". Either increase min_world_size to at least " +
            std::to_string(required_world_size) +
            "m, decrease max_spacing, or use smaller pattern_size.");
    }
}

const murosimproc::wfc::Tiles& WfcGenerator::getTiles() const
{
    return tiles_;
}

const RandomRange& WfcGenerator::getObstacleDensity() const
{
    return obstacle_density_;
}

const RandomRange& WfcGenerator::getObstacleSize() const
{
    return obstacle_size_;
}

const murosimproc::obstacle::ShapeDistribution& WfcGenerator::getShapeDistribution() const
{
    return shape_distribution_;
}

void WfcGenerator::generate(
    murosim::random::RandomGenerator::result_type seed,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    std::vector<murosim::navigation::Node> &nodes
) const
{
    murosim::random::RandomGenerator random_generator(seed);
    const float node_radius = getNodeRadius();
    const float world_size = getWorldSize()(random_generator);
    const float spacing = getSpacing()(random_generator);

    // Create PostProcessor — RandomRanges are sampled per BSP region for variety
    murosimproc::wfc::PostProcessor post_processor(
        obstacle_density_, obstacle_size_, shape_distribution_);

    // Calculate grid dimensions based on world size and spacing
    // This ensures cells are large enough for robots to navigate
    const size_t grid_size = calculateGridSize(world_size, spacing);

    // Run WFC algorithm with calculated grid dimensions
    murosimproc::wfc::WaveFunctionCollapse wfc_algorithm(tiles_);
    const murosimproc::util::Array2d<uint8_t> pattern_grid = wfc_algorithm.run(grid_size, grid_size, seed);

    // Check if WFC succeeded (non-empty result)
    if (pattern_grid.empty()) {
        // WFC failed (contradiction) - return empty world
        return;
    }

    // Calculate cell size: world_size / (grid_size × pattern_size)
    // This is the physical size of each cell in the pattern_grid output image
    const size_t pattern_size = tiles_.getPatterns()[0].sizeX();
    const float cell_size = world_size / static_cast<float>(grid_size * pattern_size);

    // Convert pattern grid to geometry using PostProcessor
    // Pattern grid contains values: 0 = empty space, 1 = wall, 2 = obstacle
    // PostProcessor will handle all three types and output in world coordinates
    post_processor.process(pattern_grid, lines, circles, nodes, random_generator, cell_size, node_radius, getMinNodeCount());
}

} // namespace murosimproc::generators
