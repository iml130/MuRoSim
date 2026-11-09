// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_GENERATOR_HPP_
#define MUROSIMPROC_GENERATORS_GENERATOR_HPP_

#include "murosim/io/data_container.hpp"
#include "murosimproc/generators/random_range.hpp"
#include <murosim/random/random_generator.hpp>
#include <murosim/geometry/line.hpp>
#include <murosim/geometry/circle.hpp>
#include <murosim/navigation/node.hpp>
#include <murosim/geometry/shape.hpp>
#include <murosimproc/util/geometry.hpp>


namespace murosimproc::generators
{

/// Abstract base class for all world generators (maze, graph, line, intersection, random-sparse, WFC, ...).
/// A generator turns a random seed into obstacle geometry (lines/circles) plus navigation nodes, retrying
/// with different seeds internally if the requested minimum node count is not reached. Derived classes
/// implement `generate()`; shared configuration (world size, node spacing/radius, minimum node count) and
/// its validation live here.
class Generator {
private:
    std::string name_;
    float node_radius_;
    RandomRange spacing_;
    RandomRange world_size_;
    size_t min_node_count_;

    /// Validates the shared configuration, throwing std::invalid_argument if: `min_node_count < 2`;
    /// `world_size.min() <= 0`; `node_radius <= 0`; `spacing.min() < 2 * node_radius` (nodes must fit
    /// within corridors); `spacing.min() > world_size.min()`; or `min_node_count` exceeds the estimated
    /// worst-case node capacity of the world (smallest world, largest spacing).
    void validateConfiguration() const;
protected:
    /// Produces one world instance for `seed`: fills `lines` (obstacle segments), `circles` (obstacle
    /// disks) and `nodes` (navigation graph). Implemented by each concrete generator.
    virtual void generate(murosim::random::RandomGenerator::result_type seed, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, std::vector<murosim::navigation::Node> &nodes) const = 0;
public:
    /// Generators are shared polymorphically (`std::shared_ptr<Generator>` in the Factory).
    virtual ~Generator() = default;
    /// Constructs a generator with default world size (10) and spacing (1), both fixed (non-random) ranges.
    /// @throws std::invalid_argument via `validateConfiguration()` if the resulting configuration is infeasible.
    Generator(const std::string &name, size_t min_node_count, float node_radius);
    /// Constructs a generator with explicit world size and node-spacing ranges.
    /// @param world_size Side length of the (square) world, in metres.
    /// @param spacing Minimum desired spacing between navigation nodes, in metres.
    /// @throws std::invalid_argument via `validateConfiguration()` if the resulting configuration is infeasible.
    Generator(const std::string &name, size_t min_node_count, float node_radius, RandomRange world_size, RandomRange spacing);

    /// Returns the generator's name (used as a prefix for generated world names).
    std::string getName() const;
    /// Returns `getName()` suffixed with `seed`, zero-padded to the width of the seed type, as used for the generated world's name.
    std::string getNameWithSeed(murosim::random::RandomGenerator::result_type seed) const;

    /// Returns the configured node-spacing range (metres).
    const RandomRange& getSpacing() const;
    /// Returns the configured world-size range (metres).
    const RandomRange& getWorldSize() const;
    /// Returns the minimum number of navigation nodes a generated world must contain.
    size_t getMinNodeCount() const;
    /// Returns the configured navigation node radius (metres).
    float getNodeRadius() const;

    /// Generates a world for `seed`. If the first attempt yields fewer than `getMinNodeCount()` nodes,
    /// retries with `seed + 1`, `seed + 2`, ... up to 100 attempts total before giving up.
    /// @return A `DataContainer` holding the generated lines, circles and nodes, named via `getNameWithSeed(seed)`.
    /// @throws std::runtime_error if no attempt reaches the minimum node count within 100 tries.
    murosim::io::DataContainer operator()(murosim::random::RandomGenerator::result_type seed) const;
    /// Same as the seed-overload, but draws a fresh seed from `random_generator` for each retry attempt
    /// (so the number of values consumed from `random_generator` depends on how many attempts are needed).
    /// @throws std::runtime_error if no attempt reaches the minimum node count within 100 tries.
    murosim::io::DataContainer operator()(murosim::random::RandomGenerator &random_generator) const;
};

} // namespace murosimproc::generators

#endif // MUROSIMPROC_GENERATORS_GENERATOR_HPP_
