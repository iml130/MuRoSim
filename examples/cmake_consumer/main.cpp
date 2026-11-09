// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Touches one symbol from each library so a successful build and run proves
// that headers, static libraries and their dependencies were all found.

#include <murosim/io/util.hpp>
#include <murosim/visualization/renderer_egl.hpp>
#include <murosimproc/util/image.hpp>
#include <murosimrl/util/grid.hpp>

#include <iostream>

int main(int argc, char ** /*argv*/)
{
    // Linked but only constructed on request: it needs a GPU context, which a
    // CI machine may lack. Linking it proves EGL and GLES resolve.
    if (argc > 1)
    {
        murosim::visualization::RendererEgl renderer(64);
        std::cout << "offscreen renderer with " << renderer.samples() << " samples" << std::endl;
    }

    const auto parts = murosim::io::util::splitString("a,b,c", ',');
    const float *lut = murosimrl::util::grid::gridValueLut(true);
    const murosimproc::util::Array2d<uint8_t> image(3, 3);
    const bool inside = murosimproc::util::image::isInImage(image, Eigen::Vector2i(1, 1));

    const bool ok = parts.size() == 3 && lut != nullptr && inside;
    std::cout << (ok ? "MuRoSim consumer: ok" : "MuRoSim consumer: FAILED") << std::endl;
    return ok ? 0 : 1;
}
