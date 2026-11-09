// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_MAZE_GENERATOR_HPP_
#define MUROSIMPROC_GENERATORS_MAZE_GENERATOR_HPP_

#include "murosim/io/data_container.hpp"
#include "murosimproc/generators/generator.hpp"
#include "murosim/random/random_generator.hpp"

namespace murosimproc::generators
{

/// Generates perfect-maze worlds using Prim's algorithm on a grid, then randomly punches holes in walls
/// to create loops, converts the result to obstacle geometry and places navigation nodes, and finally
/// scales everything to the sampled world size. Deterministic for a given seed (aside from `operator()`'s
/// own retry loop in the base `Generator`).
class MazeGenerator : public murosimproc::generators::Generator
{
private:
    RandomRange wall_thickness_;
    RandomRange wall_removal_rate_;

    /// Throws std::runtime_error if the worst-case grid size (smallest world, largest spacing and wall
    /// thickness) computed by `calculateWorldSize()` is smaller than the minimum viable maze size (3).
    void checkWorldSize() const;
    /// Returns how many grid cells of size `space + wall_thickness` fit along one side of `world_size`.
    size_t calculateWorldSize(const float world_size, const float space, const float wall_thickness) const;

protected:
    void generate(
        murosim::random::RandomGenerator::result_type seed,
        std::vector<murosim::geometry::Line> &lines,
        std::vector<murosim::geometry::Circle> &circles,
        std::vector<murosim::navigation::Node> &nodes
    ) const override;

public:
    /// Constructs a maze generator.
    /// @param wall_thickness Thickness of maze walls, in metres.
    /// @param wall_removal_rate Fraction in [0, 1] of interior walls to additionally remove after maze generation, creating loops; 0 (default) yields a perfect maze (no loops).
    /// @throws std::runtime_error via `checkWorldSize()` if the world is too small to fit a viable maze grid.
    MazeGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        RandomRange wall_thickness,
        RandomRange wall_removal_rate = 0.0f
    );

    /// Returns the configured wall-thickness range (metres).
    const RandomRange& getWallThickness() const;
    /// Returns the configured wall-removal-rate range ([0, 1]).
    const RandomRange& getWallRemovalRate() const;
};

} // namespace murosimproc::generators

#endif // MUROSIMPROC_GENERATORS_MAZE_GENERATOR_HPP_
