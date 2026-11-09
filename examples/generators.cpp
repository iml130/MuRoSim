// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Shows every procedural world generator in murosimproc. Each screen holds a
// 3x3 grid of worlds drawn from the same generator with different random
// draws, which is the quickest way to see what a parameter set produces.
//
// Needs a display: this example opens a GLFW window. Press any key in the
// window to advance to the next generator, close the window to quit. The
// random draws come from std::random_device unless a seed is given, in which
// case every run shows the same worlds.
//
//   cmake -S . -B build -DMUROSIM_BUILD_EXAMPLES=ON
//   cmake --build build
//   ./build/examples/murosim_example_generators [seed]

#include "murosim/io/data_container.hpp"
#include "murosim/random/random_generator.hpp"
#include "murosim/visualization/renderer_glfw.hpp"

#include "murosimproc/generators/generator.hpp"
#include "murosimproc/generators/graph_generator.hpp"
#include "murosimproc/generators/intersection_generator.hpp"
#include "murosimproc/generators/line_generator.hpp"
#include "murosimproc/generators/maze_generator.hpp"
#include "murosimproc/generators/random_range.hpp"
#include "murosimproc/generators/random_sparse_generator.hpp"
#include "murosimproc/generators/wfc_generator.hpp"
#include "murosimproc/obstacle/shape_distribution.hpp"
#include "murosimproc/wfc/preset.hpp"

#include <charconv>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <system_error>
#include <vector>

namespace generators = murosimproc::generators;
namespace preset = murosimproc::wfc::preset;

namespace
{
using murosim::io::DataContainer;
using murosim::random::RandomGenerator;
using murosim::visualization::RendererGlfw;

constexpr size_t GRID_COUNT = 9; // 3x3 worlds per screen
constexpr float NODE_RADIUS = 0.25f;
// Shared by every generator below; the same values the Python examples use.
const generators::RandomRange WORLD_SIZE(12.0f, 16.0f);
const generators::RandomRange SPACING(0.8f, 1.25f);

std::vector<DataContainer> generateGrid(generators::Generator &generator, RandomGenerator &random_generator)
{
    std::vector<DataContainer> containers;
    containers.reserve(GRID_COUNT);
    for (size_t i = 0; i < GRID_COUNT; i++)
    {
        containers.push_back(generator(random_generator));
    }
    return containers;
}

/// Draws the grid and blocks until a key is pressed in the window.
/// @return false if the window was closed instead.
bool showAndWait(RendererGlfw &renderer, const std::vector<DataContainer> &containers, const std::string &title)
{
    std::cout << "=== " << title << " (press a key to continue, close the window to quit)" << std::endl;
    renderer.draw(containers);
    renderer.show();
    return renderer.waitKey() != -1;
}

/// Parses the optional seed argument. Returns false if it is not a whole
/// non-negative number that fits the generator's seed type.
bool parseSeed(const char *text, RandomGenerator::result_type &seed)
{
    const char *const end = text + std::strlen(text);
    const auto [rest, error] = std::from_chars(text, end, seed);
    return error == std::errc() && rest == end;
}

/// Shows every generator; returns early once the window is closed.
void showGenerators(RandomGenerator &random_generator)
{
    RendererGlfw renderer(1000);

    {
        generators::GraphGenerator generator(
            /* min_node_count     */ 5,
            /* node_radius        */ NODE_RADIUS,
            /* world_size         */ WORLD_SIZE,
            /* spacing            */ SPACING,
            /* edge_length        */ {1.5f, 4.0f},
            /* point_removal_rate */ {0.05f, 0.75f},
            /* edge_removal_rate  */ {0.05f, 0.75f}
        );
        if (!showAndWait(renderer, generateGrid(generator, random_generator), "GraphGenerator"))
        {
            return;
        }
    }

    {
        generators::IntersectionGenerator generator(
            /* min_node_count */ 4,
            /* node_radius    */ NODE_RADIUS,
            /* world_size     */ WORLD_SIZE,
            /* spacing        */ SPACING,
            /* center_offset  */ 0.5f
        );
        if (!showAndWait(renderer, generateGrid(generator, random_generator), "IntersectionGenerator"))
        {
            return;
        }
    }

    {
        generators::LineGenerator generator(
            /* min_node_count */ 2,
            /* node_radius    */ NODE_RADIUS,
            /* world_size     */ WORLD_SIZE,
            /* spacing        */ SPACING,
            /* length_offset  */ 0.5f
        );
        if (!showAndWait(renderer, generateGrid(generator, random_generator), "LineGenerator"))
        {
            return;
        }
    }

    {
        generators::MazeGenerator generator(
            /* min_node_count    */ 5,
            /* node_radius       */ NODE_RADIUS,
            /* world_size        */ WORLD_SIZE,
            /* spacing           */ SPACING,
            /* wall_thickness    */ {0.1f, 0.3f},
            /* wall_removal_rate */ {0.0f, 0.3f}
        );
        if (!showAndWait(renderer, generateGrid(generator, random_generator), "MazeGenerator"))
        {
            return;
        }
    }

    {
        generators::RandomSparseGenerator generator(
            /* min_node_count         */ 5,
            /* node_radius            */ NODE_RADIUS,
            /* world_size             */ WORLD_SIZE,
            /* spacing                */ SPACING,
            /* obstacle_size          */ {0.3f, 2.0f},
            /* placement_density_rate */ 1.0f
        );
        if (!showAndWait(renderer, generateGrid(generator, random_generator), "RandomSparseGenerator"))
        {
            return;
        }
    }

    // One screen per WFC preset. The lower world_size bound is not free:
    // world_size.min >= spacing.max * pattern_size * WFC_MIN_GRID_SIZE
    //                 = 1.25 * 3 * 3 = 11.25, which WORLD_SIZE satisfies.
    for (size_t i = 0; i < preset::PRESET_COUNT; i++)
    {
        const auto preset_type = static_cast<preset::PresetType>(i);

        generators::WfcGenerator generator(
            /* min_node_count     */ 5,
            /* node_radius        */ NODE_RADIUS,
            /* world_size         */ WORLD_SIZE,
            /* spacing            */ SPACING,
            /* preset_type        */ preset_type,
            /* obstacle_size      */ {0.3f, 1.5f},
            /* obstacle_density   */ {0.3f, 0.7f},
            /* shape_distribution */ murosimproc::obstacle::ShapeDistribution{}
        );

        if (!showAndWait(renderer, generateGrid(generator, random_generator),
                    "WfcGenerator - " + std::string(preset::getPresetName(preset_type))))
        {
            return;
        }
    }

}
} // namespace

int main(int argc, char **argv)
{
    RandomGenerator::result_type seed = 0;
    if (argc > 2 || (argc == 2 && !parseSeed(argv[1], seed)))
    {
        std::cerr << "usage: " << argv[0] << " [seed]  (seed: a non-negative integer)" << std::endl;
        return EXIT_FAILURE;
    }
    // Without an argument the generator seeds itself from std::random_device.
    RandomGenerator random_generator = argc == 2 ? RandomGenerator(seed) : RandomGenerator();
    std::cout << "seed " << random_generator.seed()
              << " (pass it as the first argument to see these worlds again)" << std::endl;

    try
    {
        showGenerators(random_generator);
    }
    catch (const std::exception &error)
    {
        std::cerr << "generation failed: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    return 0;
}
