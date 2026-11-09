// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_WFC_POST_PROCESSOR_HPP_
#define MUROSIMPROC_WFC_POST_PROCESSOR_HPP_

#include "murosimproc/util/array2d.hpp"
#include "murosimproc/obstacle/shape_distribution.hpp"
#include "murosimproc/generators/random_range.hpp"
#include "murosim/geometry/line.hpp"
#include "murosim/geometry/circle.hpp"
#include "murosim/navigation/node.hpp"
#include "murosim/random/random_generator.hpp"

#include <Eigen/Core>
#include <random>

namespace murosimproc::wfc
{

/// Minimum BSP split ratio (constrains split to middle portion of range)
constexpr float MIN_BSP_SPLIT_RATIO = 0.2f;

/**
 * @brief Converts WFC pattern grid to geometry (lines, circles, nodes)
 *
 * Processes a pattern grid where:
 * - 0 = empty/navigable space -> navigation nodes
 * - 1 = wall -> line contours
 * - 2 = obstacle zone -> multiple circles or polygons with varying sizes
 */
class PostProcessor
{
private:
    generators::RandomRange obstacle_density_;
    generators::RandomRange obstacle_size_;
    murosimproc::obstacle::ShapeDistribution shape_distribution_;

    /// Returns a copy of `image` in which every 4-connected non-wall region (free cells and obstacle
    /// zones count as one region together) is flood-filled to 1 (wall) except the single largest one,
    /// which is left untouched. Used to discard small, likely-spurious pockets produced by WFC.
    murosimproc::util::Array2d<uint8_t> fillSmallestRegion(const murosimproc::util::Array2d<uint8_t> &image) const;
    /// Recursively splits `image` via `bspOccupied()` to find obstacle-zone (value 2) regions, and for each
    /// one calls `placeOccupiedObjects()` to scatter obstacle shapes inside its bounding box.
    void addOccupiedObjects(std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, murosim::random::RandomGenerator &random_generator, const murosimproc::util::Array2d<uint8_t> &image) const;
    /// Binary-space-partitions the cell rectangle `[min_point, max_point]` (recursing on the longer axis,
    /// split point constrained to the middle portion of the range via `MIN_BSP_SPLIT_RATIO`) down to
    /// uniform regions, and returns whether the whole rectangle is obstacle (value 2) cells.
    bool bspOccupied(const murosimproc::util::Array2d<uint8_t> &image, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, murosim::random::RandomGenerator &random_generator, std::uniform_real_distribution<float> &merge_dist, Eigen::Vector2i min_point, Eigen::Vector2i max_point) const;
    /// Scatters obstacle shapes (via `obstacle::placeObstacles`) across the rectangular cell area `[min_point, max_point]`, using this post-processor's density/size/shape configuration.
    void placeOccupiedObjects(std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, murosim::random::RandomGenerator &random_generator, const Eigen::Vector2i min_point, const Eigen::Vector2i max_point) const;

public:
    /// Constructs a post-processor with the given obstacle-scattering configuration (sampled per obstacle-zone region for variety, see `addOccupiedObjects()`).
    PostProcessor(
        generators::RandomRange obstacle_density,
        generators::RandomRange obstacle_size,
        murosimproc::obstacle::ShapeDistribution shape_distribution);

    /// Converts a WFC pattern grid (`image`, cell values 0=empty, 1=wall, 2=obstacle) into world geometry.
    /// Walls become contour lines, obstacle regions get scattered obstacle shapes, and navigation nodes
    /// are chosen from empty cells via farthest-point sampling (preferring cells where a parked robot
    /// would not block others, falling back to other empty cells if there are not enough of those).
    /// @param cell_size Physical size of one grid cell (metres).
    /// @param node_radius Radius used for placed navigation nodes (metres).
    /// @param target_node_count Desired number of navigation nodes; fewer are placed if there are not enough empty cells.
    void process(const murosimproc::util::Array2d<uint8_t> &image, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, std::vector<murosim::navigation::Node> &nodes, murosim::random::RandomGenerator &random_generator, float cell_size, float node_radius, size_t target_node_count) const;
};

}// namespace murosimproc::wfc

#endif // MUROSIMPROC_WFC_POST_PROCESSOR_HPP_
