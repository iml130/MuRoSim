// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_LINE_GENERATOR_HPP_
#define MUROSIMPROC_GENERATORS_LINE_GENERATOR_HPP_

#include "murosimproc/generators/generator.hpp"

namespace murosimproc::generators
{

/// Generates a single straight corridor: a square world with one rectangular corridor (a pair of
/// parallel walls) and a navigation node at each end, facing along the corridor. Deterministic for a
/// given seed (aside from `operator()`'s own retry loop in the base `Generator`).
class LineGenerator: public murosimproc::generators::Generator
{
private:
    float length_offset_;
    bool is_horizontal_;

    /// Throws std::invalid_argument if `world_size.min()` is not more than twice `spacing.max()`, or if
    /// it does not fit `min_node_count` node slots of that spacing along the corridor.
    void checkWorldSize() const;

protected:
    void generate(murosim::random::RandomGenerator::result_type seed, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, std::vector<murosim::navigation::Node> &nodes) const override;

public:
    /// Constructs a line-corridor generator.
    /// @param length_offset Clamped to [0, 1]; how much shorter than the maximum possible length the corridor may randomly be.
    /// @param is_horizontal If true, the corridor is always axis-aligned along x; if false (default), an arbitrarily-oriented, non-overlapping placement is attempted first and only falls back to the axis-aligned line if none is found within 100 tries.
    /// @throws std::invalid_argument via `checkWorldSize()` if the world is too small for the configured spacing and node count.
    LineGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        float length_offset,
        bool is_horizontal = false
    );
};

} // namespace murosimproc::generators


#endif // MUROSIMPROC_GENERATORS_LINE_GENERATOR_HPP_
