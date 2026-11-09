// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/visualization/renderer_egl.hpp"
#include "murosim/visualization/frame_buffer.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>

namespace murosimpy {
namespace {

// Keep this helper - FrameBuffer is a custom type, not handled by pybind11/eigen.h
pybind11::array_t<uint8_t> convertFrameBufferToNumpyArray(const murosim::visualization::FrameBuffer &frame_buffer)
{
    const ssize_t width = static_cast<ssize_t>(frame_buffer.width());
    const ssize_t height = static_cast<ssize_t>(frame_buffer.height());
    const ssize_t channels = 3; // RGB format
    const size_t size = static_cast<size_t>(width * height * channels);

    pybind11::array_t<uint8_t> array = pybind11::array_t<uint8_t>({height, width, channels}, {});
    std::copy(frame_buffer.data(), frame_buffer.data() + size, array.mutable_data());
    return array;
}

} // anonymous namespace

void initRenderingOfflineRenderer(pybind11::module &m)
{
    // === OfflineRenderer (EGL-based headless rendering) ===
    auto pyOfflineRenderer = pybind11::class_<murosim::visualization::RendererEgl>(m, "OfflineRenderer",
        R"doc(Headless renderer using an EGL pbuffer surface and OpenGL ES 3, with no
visible window.

Prefers the surfaceless EGL platform where available (typical of CI and
server environments without a display), falling back to the default EGL
display otherwise; needs no X11/Wayland/GLFW window. Use OnlineRenderer
instead for an interactive on-screen window. A renderer instance owns one GL
context and must only be driven from one thread at a time.)doc");

    pyOfflineRenderer.def(pybind11::init<const int, const int>(),
        "Create an offline renderer and its EGL context.\n\n"
        "Args:\n"
        "    resolution: Square output image resolution in pixels (default: 600)\n"
        "    samples: Multisampling sample count for anti-aliasing, 0 disables it\n"
        "        (default: 8), clamped to the driver's maximum (Mesa's CPU renderer\n"
        "        llvmpipe supports 4). See the samples property for the count in use.\n\n"
        "Raises:\n"
        "    ValueError: if resolution is not positive or samples is negative.\n"
        "    RuntimeError: if EGL display/context/surface creation fails.",
        pybind11::arg("resolution") = 600, pybind11::arg("samples") = 8);
    pyOfflineRenderer.def_property_readonly("samples", &murosim::visualization::RendererEgl::samples,
        "Multisampling sample count actually in use (0 = no anti-aliasing).");

    // Render methods
    pyOfflineRenderer.def("render", [](murosim::visualization::RendererEgl &r,
                                        const std::shared_ptr<murosim::simulation::Simulation> sim) {
        r.draw(sim);
        return convertFrameBufferToNumpyArray(r.getFrameBuffer());
    }, "Render a simulation and return the image as a numpy array.\n\n"
       "Args:\n"
       "    simulation: Simulation to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("simulation"));
    pyOfflineRenderer.def("render", [](murosim::visualization::RendererEgl &r,
                                        const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        r.draw(simulations);
        return convertFrameBufferToNumpyArray(r.getFrameBuffer());
    }, "Render multiple simulations in a grid and return the image as a numpy array.\n\n"
       "Args:\n"
       "    simulations: List of simulations to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("simulations"));
    pyOfflineRenderer.def("render", [](murosim::visualization::RendererEgl &r,
                                        const std::shared_ptr<murosim::robot::Robot> robot) {
        r.draw(robot);
        return convertFrameBufferToNumpyArray(r.getFrameBuffer());
    }, "Render a robot and return the image as a numpy array.\n\n"
       "Args:\n"
       "    robot: Robot to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("robot"));
    pyOfflineRenderer.def("render", [](murosim::visualization::RendererEgl &r,
                                        const murosim::geometry::Shape &shape) {
        r.draw(shape);
        return convertFrameBufferToNumpyArray(r.getFrameBuffer());
    }, "Render a shape and return the image as a numpy array.\n\n"
       "Args:\n"
       "    shape: Shape to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("shape"));
    pyOfflineRenderer.def("render", [](murosim::visualization::RendererEgl &r,
                                        const murosim::geometry::World &world) {
        r.draw(world);
        return convertFrameBufferToNumpyArray(r.getFrameBuffer());
    }, "Render a world and return the image as a numpy array.\n\n"
       "Args:\n"
       "    world: World to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("world"));
    pyOfflineRenderer.def("render", [](murosim::visualization::RendererEgl &r,
                                        const murosim::geometry::World &,
                                        const murosim::io::DataContainer &data) {
        r.draw(data);
        return convertFrameBufferToNumpyArray(r.getFrameBuffer());
    }, "Render a DataContainer and return the image as a numpy array.\n\n"
       "Args:\n"
       "    world: World (unused, for API compatibility)\n"
       "    data: DataContainer to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("world"), pybind11::arg("data"));

    // Settings (write-only properties)
    pyOfflineRenderer.def_property("lidar_skip_steps",
        nullptr,
        [](murosim::visualization::RendererEgl &r, long steps) {
            r.setLaserSkipSteps(static_cast<size_t>(std::max(0L, steps)));
        },
        "Draw every Nth lidar ray: 1 = all rays, 0 = none (the default). Write-only.");
    pyOfflineRenderer.def_property("robot_trajectory_length",
        nullptr,
        [](murosim::visualization::RendererEgl &r, long len) {
            r.setRobotTrajectoryLength(static_cast<size_t>(std::max(0L, len)));
        },
        "Length of robot trajectory trail to render (0 = no trail, write-only).");
    pyOfflineRenderer.def_property("draw_robot_path",
        nullptr,
        [](murosim::visualization::RendererEgl &r, bool draw) {
            r.setDrawRobotPath(draw);
        },
        "Whether to render the planner's path for each robot (write-only).");
    pyOfflineRenderer.def_property("draw_grid_map",
        nullptr,
        [](murosim::visualization::RendererEgl &r, bool draw) {
            r.setDrawGridMap(draw);
        },
        "Whether to render the grid map (free/occupied/buffer cells, write-only).");

    // Python special methods
    pyOfflineRenderer.def("__repr__", [](const murosim::visualization::RendererEgl &) {
        return "<OfflineRenderer (EGL)>";
    }, "Return a debug string representation.");
}

} // namespace murosimpy
