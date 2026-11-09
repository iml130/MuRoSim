// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_INTERSECTION_GENERATOR_HPP_
#define MUROSIMPROC_GENERATORS_INTERSECTION_GENERATOR_HPP_

#include "murosim/io/data_container.hpp"
#include "murosimproc/generators/generator.hpp"

namespace murosimproc::generators
{

/// Generates a single four-way "+"-shaped intersection: a square world with a cross-shaped open area
/// (of side `spacing`) placed near the center, one navigation node facing inward on each of the four
/// approach corridors. Deterministic for a given seed (aside from `operator()`'s own retry loop in the
/// base `Generator`).
class IntersectionGenerator: public murosimproc::generators::Generator
{
private:
    float center_offset_;
    /// Throws std::invalid_argument if `world_size.min()` is not more than three times `spacing.max()`,
    /// i.e. the world would be too small to fit the intersection with its four approach corridors.
    void checkWorldSize() const;
protected:
    void generate(murosim::random::RandomGenerator::result_type seed, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, std::vector<murosim::navigation::Node> &nodes) const override;
public:
    /// Constructs an intersection generator.
    /// @param center_offset Clamped to [0, 1]; how far the intersection's center may be randomly displaced from the world's center, as a fraction of the available offset room.
    /// @throws std::invalid_argument via `checkWorldSize()` if the world is too small for the configured spacing.
    IntersectionGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        float center_offset
    );
};

} // namespace murosimproc::generators


#endif // MUROSIMPROC_GENERATORS_INTERSECTION_GENERATOR_HPP_
