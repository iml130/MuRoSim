// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_WFC_GENERATOR_HPP_
#define MUROSIMPROC_GENERATORS_WFC_GENERATOR_HPP_

#include "murosimproc/generators/generator.hpp"
#include "murosimproc/obstacle/shape_distribution.hpp"
#include "murosimproc/wfc/post_processor.hpp"
#include "murosimproc/wfc/preset.hpp"
#include "murosimproc/wfc/tiles.hpp"

namespace murosimproc::generators
{

namespace defaults {
    /// Minimum grid size to ensure meaningful WFC generation
    constexpr size_t WFC_MIN_GRID_SIZE = 3;
}

/// Generates worlds via Wave Function Collapse (WFC): a tile-pattern grid is collapsed into a layout of
/// empty/wall/obstacle cells (see `wfc::WaveFunctionCollapse`), then converted to obstacle geometry and
/// navigation nodes by a `wfc::PostProcessor` that also scatters extra small obstacles. Deterministic for
/// a given seed (aside from `operator()`'s own retry loop in the base `Generator`); if WFC hits a
/// contradiction, `generate()` silently returns an empty world for that attempt.
class WfcGenerator : public murosimproc::generators::Generator
{
private:
    murosimproc::wfc::Tiles tiles_;

    // Obstacle placement parameters
    RandomRange obstacle_density_;
    RandomRange obstacle_size_;
    obstacle::ShapeDistribution shape_distribution_;

    /// Validates the WFC-specific configuration, throwing std::invalid_argument if: `tiles_` has no
    /// patterns or weights; its patterns are not square or have size 0; `obstacle_density_` has a
    /// negative minimum; or the worst-case WFC grid (smallest world, largest spacing) would be smaller
    /// than `defaults::WFC_MIN_GRID_SIZE` per side.
    void validateWFCConfig() const;
    /// Returns how many WFC grid cells (each `min_spacing * pattern_size` wide) fit along one side of `world_size`.
    size_t calculateGridSize(const float world_size, const float min_spacing) const;

protected:
    void generate(
        murosim::random::RandomGenerator::result_type seed,
        std::vector<murosim::geometry::Line> &lines,
        std::vector<murosim::geometry::Circle> &circles,
        std::vector<murosim::navigation::Node> &nodes
    ) const override;

public:
    /// Constructs a WFC generator using a named built-in tile preset.
    /// @param obstacle_size Size range (metres) for the extra obstacles scattered by the post-processor.
    /// @param obstacle_density Density range controlling how many extra obstacles are scattered.
    /// @param shape_distribution Relative weights and thresholds used to choose obstacle shapes.
    /// @throws std::invalid_argument via `validateWFCConfig()` if the configuration is infeasible.
    WfcGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        murosimproc::wfc::preset::PresetType preset_type,
        RandomRange obstacle_size,
        RandomRange obstacle_density,
        obstacle::ShapeDistribution shape_distribution
    );

    /// Constructs a WFC generator with an explicit, caller-provided tile set instead of a preset.
    /// @throws std::invalid_argument via `validateWFCConfig()` if the configuration is infeasible.
    WfcGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        const murosimproc::wfc::Tiles &tiles,
        RandomRange obstacle_size,
        RandomRange obstacle_density,
        obstacle::ShapeDistribution shape_distribution
    );

    /// Returns the tile set used for WFC generation.
    const murosimproc::wfc::Tiles& getTiles() const;

    /// Returns the configured obstacle-density range.
    const RandomRange& getObstacleDensity() const;
    /// Returns the configured obstacle-size range (metres).
    const RandomRange& getObstacleSize() const;
    /// Returns the configured obstacle shape distribution.
    const obstacle::ShapeDistribution& getShapeDistribution() const;
};

} // namespace murosimproc::generators

#endif // MUROSIMPROC_GENERATORS_WFC_GENERATOR_HPP_
