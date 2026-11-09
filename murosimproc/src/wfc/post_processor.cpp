// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/wfc/post_processor.hpp"

#include "murosim/geometry/line.hpp"
#include "murosim/navigation/node.hpp"
#include "murosimproc/util/image.hpp"
#include "murosimproc/util/geometry.hpp"
#include "murosimproc/obstacle/place_obstacles.hpp"
#include "murosimproc/obstacle/shape_distribution.hpp"
#include "murosim/math/constants.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <vector>

// Maximum region size (in cells) at which merge decisions are made
constexpr size_t MAX_SPLIT_SIZE = 3;

namespace murosimproc::wfc
{

PostProcessor::PostProcessor(
    murosimproc::generators::RandomRange obstacle_density,
    murosimproc::generators::RandomRange obstacle_size,
    murosimproc::obstacle::ShapeDistribution shape_distribution)
    : obstacle_density_(obstacle_density)
    , obstacle_size_(obstacle_size)
    , shape_distribution_(shape_distribution)
{
}

murosimproc::util::Array2d<uint8_t> PostProcessor::fillSmallestRegion(const murosimproc::util::Array2d<uint8_t> &image) const
{
    murosimproc::util::Array2d<uint8_t> image_filled = image;

    std::vector<size_t> indices_biggest_region;

    for(size_t y = 0; y < image.sizeY(); y++)
    {
        for(size_t x = 0; x < image.sizeX(); x++)
        {
            if(image_filled(x, y) != 1)
            {
                std::vector<size_t> indices_region;
                murosimproc::util::image::fillRegion(image_filled, Eigen::Vector2i(x, y), 1, indices_region);
                if (indices_biggest_region.size() < indices_region.size())
                {
                    indices_biggest_region = std::move(indices_region);
                }
            }
        }
    }

    for (const size_t index : indices_biggest_region)
    {
        image_filled[index] = image[index];
    }

    return image_filled;
}

void PostProcessor::placeOccupiedObjects(
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    murosim::random::RandomGenerator &random_generator,
    const Eigen::Vector2i min_point,
    const Eigen::Vector2i max_point) const
{
    const Eigen::Vector2f area_min = min_point.cast<float>();
    const Eigen::Vector2f area_max = max_point.cast<float>() + Eigen::Vector2f(1.0f, 1.0f);

    const float density = obstacle_density_(random_generator);
    const float scale_min = obstacle_size_.min();
    const float scale_max = obstacle_size_.max();
    murosimproc::obstacle::placeObstacles(density, scale_min, scale_max, shape_distribution_, area_min, area_max, random_generator, lines, circles);
}

void PostProcessor::addOccupiedObjects(
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    murosim::random::RandomGenerator &random_generator,
    const murosimproc::util::Array2d<uint8_t> &image) const
{
    std::uniform_real_distribution<float> merge_dist(0.0f, 1.0f);
    const Eigen::Vector2i min_point = Eigen::Vector2i::Zero();
    const Eigen::Vector2i max_point = Eigen::Vector2i(image.sizeX() - 1, image.sizeY() - 1);

    if (bspOccupied(image, lines, circles, random_generator, merge_dist, min_point, max_point))
    {
        placeOccupiedObjects(lines, circles, random_generator, min_point, max_point);
    }
}

bool PostProcessor::bspOccupied(
    const murosimproc::util::Array2d<uint8_t> &image,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    murosim::random::RandomGenerator &random_generator,
    std::uniform_real_distribution<float> &merge_dist,
    const Eigen::Vector2i min_point,
    const Eigen::Vector2i max_point) const
{
    const Eigen::Vector2i diff = max_point - min_point;

    assert(0 <= diff.x());
    assert(0 <= diff.y());
    assert(murosimproc::util::image::isInImage(image, min_point));
    assert(murosimproc::util::image::isInImage(image, max_point));

    // Base case: single pixel
    if (diff == Eigen::Vector2i::Zero())
    {
        return murosimproc::util::image::at(image, min_point) == 2;
    }

    // Early termination: check if entire region is uniform
    bool all_obstacle = true;
    bool all_empty = true;

    for (int y = min_point.y(); y <= max_point.y() && (all_obstacle || all_empty); ++y)
    {
        for (int x = min_point.x(); x <= max_point.x() && (all_obstacle || all_empty); ++x)
        {
            const uint8_t val = image(static_cast<size_t>(x), static_cast<size_t>(y));
            if (val != 2) all_obstacle = false;
            if (val == 2) all_empty = false;
        }
    }

    if (all_obstacle) return true;
    if (all_empty)    return false;

    // Choose split direction: prefer splitting along the longer dimension
    const bool split_y = (diff.x() == diff.y())
        ? std::uniform_int_distribution<int>(0, 1)(random_generator)
        : (diff.x() < diff.y());

    // Constrain BSP split to middle 60% of range to avoid degenerate splits
    Eigen::Vector2i min_point_split = max_point;
    Eigen::Vector2i max_point_split = min_point;

    if (split_y)
    {
        const int range = max_point.y() - min_point.y();
        int split_min = min_point.y();
        int split_max = max_point.y() - 1;

        if (range >= 3)
        {
            const int constrained_min = min_point.y() + static_cast<int>(std::ceil(static_cast<float>(range) * MIN_BSP_SPLIT_RATIO));
            const int constrained_max = max_point.y() - 1 - static_cast<int>(std::ceil(static_cast<float>(range) * MIN_BSP_SPLIT_RATIO));
            if (constrained_min <= constrained_max)
            {
                split_min = constrained_min;
                split_max = constrained_max;
            }
        }

        const int split = std::uniform_int_distribution<int>(split_min, split_max)(random_generator);
        min_point_split.y() = split;
        max_point_split.y() = split + 1;
    }
    else
    {
        const int range = max_point.x() - min_point.x();
        int split_min = min_point.x();
        int split_max = max_point.x() - 1;

        if (range >= 3)
        {
            const int constrained_min = min_point.x() + static_cast<int>(std::ceil(static_cast<float>(range) * MIN_BSP_SPLIT_RATIO));
            const int constrained_max = max_point.x() - 1 - static_cast<int>(std::ceil(static_cast<float>(range) * MIN_BSP_SPLIT_RATIO));
            if (constrained_min <= constrained_max)
            {
                split_min = constrained_min;
                split_max = constrained_max;
            }
        }

        const int split = std::uniform_int_distribution<int>(split_min, split_max)(random_generator);
        min_point_split.x() = split;
        max_point_split.x() = split + 1;
    }

    const bool can_place1 = bspOccupied(image, lines, circles, random_generator, merge_dist, min_point, min_point_split);
    const bool can_place2 = bspOccupied(image, lines, circles, random_generator, merge_dist, max_point_split, max_point);

    if (can_place1 && can_place2)
    {
        const int area_size = (diff.x() + 1) * (diff.y() + 1);
        const float merge_probability = std::min(1.0f,
            static_cast<float>(area_size) / static_cast<float>(MAX_SPLIT_SIZE * MAX_SPLIT_SIZE));

        if (merge_dist(random_generator) < merge_probability)
        {
            return true;
        }
    }

    if (can_place1)
    {
        placeOccupiedObjects(lines, circles, random_generator, min_point, min_point_split);
    }

    if (can_place2)
    {
        placeOccupiedObjects(lines, circles, random_generator, max_point_split, max_point);
    }

    return false;
}


void PostProcessor::process(
    const murosimproc::util::Array2d<uint8_t> &image,
    std::vector<murosim::geometry::Line> &lines,
    std::vector<murosim::geometry::Circle> &circles,
    std::vector<murosim::navigation::Node> &nodes,
    murosim::random::RandomGenerator &random_generator,
    float cell_size,
    float node_radius,
    size_t target_node_count) const
{
    // Step 1: Fill smallest region to remove noise
    murosimproc::util::Array2d<uint8_t> image_filled = fillSmallestRegion(image);

    // Step 2: Collect free cells, candidates first: cells where a parked
    // robot does not block others (dead ends, open areas). Corridor and
    // intersection cells follow as fallback and are only used when there
    // are fewer candidates than requested nodes.
    std::vector<Eigen::Vector2f> free_cells;
    std::vector<Eigen::Vector2f> fallback_cells;
    for (size_t y = 0; y < image_filled.sizeY(); ++y) {
        for (size_t x = 0; x < image_filled.sizeX(); ++x) {
            if (image_filled(x, y) == 0) {
                const float center_x = (static_cast<float>(x) + 0.5f) * cell_size;
                const float center_y = (static_cast<float>(y) + 0.5f) * cell_size;
                if (murosimproc::util::image::isNodeCandidate(image_filled, static_cast<int>(x), static_cast<int>(y))) {
                    free_cells.emplace_back(center_x, center_y);
                } else {
                    fallback_cells.emplace_back(center_x, center_y);
                }
            }
        }
    }

    const size_t candidate_count = free_cells.size();
    free_cells.insert(free_cells.end(), fallback_cells.begin(), fallback_cells.end());

    // Select target_node_count nodes using farthest-point sampling,
    // restricted to the candidate prefix while unselected candidates remain
    const size_t select_count = std::min(target_node_count, free_cells.size());

    if (select_count > 0)
    {
        std::vector<bool> selected(free_cells.size(), false);
        std::vector<float> min_dist(free_cells.size(), std::numeric_limits<float>::max());

        for (size_t n = 0; n < select_count; ++n)
        {
            const size_t eligible_count = (n < candidate_count) ? candidate_count : free_cells.size();

            size_t best_idx = 0;
            if (n == 0)
            {
                // Pick first node randomly
                best_idx = std::uniform_int_distribution<size_t>(0, eligible_count - 1)(random_generator);
            }
            else
            {
                // Greedily select the farthest cell from all selected nodes
                float best_dist = -1.0f;
                for (size_t i = 0; i < eligible_count; ++i)
                {
                    if (!selected[i] && min_dist[i] > best_dist)
                    {
                        best_dist = min_dist[i];
                        best_idx = i;
                    }
                }
            }

            selected[best_idx] = true;
            nodes.emplace_back(murosim::geometry::Circle(free_cells[best_idx].x(), free_cells[best_idx].y(), node_radius), 0.0f, murosim::math::PI);

            // Update min distances with the newly selected node
            for (size_t i = 0; i < free_cells.size(); ++i)
            {
                if (!selected[i])
                {
                    const float dx = free_cells[i].x() - free_cells[best_idx].x();
                    const float dy = free_cells[i].y() - free_cells[best_idx].y();
                    const float d2 = dx * dx + dy * dy;
                    if (d2 < min_dist[i])
                    {
                        min_dist[i] = d2;
                    }
                }
            }
        }
    }

    // Step 3: Find wall contours (value 1) using contour detection
    murosimproc::util::Array2d<uint8_t> wall_image(image_filled.sizeX(), image_filled.sizeY(), 0);
    for (size_t i = 0; i < image_filled.size(); ++i) {
        if (image_filled[i] == 1) {
            wall_image[i] = 1;
        }
    }

    // Find contour lines in grid coordinates
    std::vector<murosim::geometry::Line> contour_lines = murosimproc::util::image::findContourLines(wall_image);

    // Scale contour lines to world coordinates
    for (auto &line : contour_lines) { line.scale(cell_size); }
    lines.insert(lines.end(),
        std::make_move_iterator(contour_lines.begin()),
        std::make_move_iterator(contour_lines.end()));

    // Step 4: Add border in world coordinates
    const float world_size = static_cast<float>(std::max(image.sizeX(), image.sizeY())) * cell_size;
    murosimproc::util::geometry::addBorderLines(lines, world_size);

    // Step 5: Use BSP to place objects in obstacle zones (value 2)
    std::vector<murosim::geometry::Line> bsp_lines;
    std::vector<murosim::geometry::Circle> bsp_circles;
    addOccupiedObjects(bsp_lines, bsp_circles, random_generator, image_filled);

    // Scale BSP output to world coordinates
    for (auto &line : bsp_lines) { line.scale(cell_size); }
    lines.insert(lines.end(),
        std::make_move_iterator(bsp_lines.begin()),
        std::make_move_iterator(bsp_lines.end()));

    for (auto &circle : bsp_circles) { circle.scale(cell_size); }
    circles.insert(circles.end(),
        std::make_move_iterator(bsp_circles.begin()),
        std::make_move_iterator(bsp_circles.end()));
}

} // namespace murosimproc::wfc
