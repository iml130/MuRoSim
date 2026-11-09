// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/visualization/renderer_glfw.hpp"
#include "murosim/visualization/frame_buffer.hpp"
#include "murosim/simulation/simulation.hpp"
#include "murosim/io/data_container.hpp"

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

void initRenderingOnlineRenderer(pybind11::module &m)
{
    // === OnlineRenderer (GLFW-based windowed rendering) ===
    auto pyOnlineRenderer = pybind11::class_<murosim::visualization::RendererGlfw>(m, "OnlineRenderer",
        R"doc(On-screen renderer backed by a GLFW window and OpenGL ES 3, for interactive
real-time display.

Requires a display/windowing system to be available (unlike OfflineRenderer).
GLFW is initialized process-wide on first use; vsync is disabled. Each
render() call redraws and immediately presents the frame (swaps buffers and
polls window/input events) before returning the image. A renderer instance
owns one GL context/window and must only be driven from one thread at a
time.)doc");

    pyOnlineRenderer.def(pybind11::init<const int, const int>(),
        "Create an online renderer, its window, and its GL context.\n\n"
        "Args:\n"
        "    resolution: Square window resolution in pixels (default: 600)\n"
        "    samples: Multisampling sample count for anti-aliasing, 0 disables it\n"
        "        (default: 8), clamped to the driver's maximum. See the samples property.\n\n"
        "Raises:\n"
        "    ValueError: if resolution is not positive or samples is negative.\n"
        "    RuntimeError: if the GLFW window cannot be created.",
        pybind11::arg("resolution") = 600, pybind11::arg("samples") = 8);
    pyOnlineRenderer.def_property_readonly("samples", &murosim::visualization::RendererGlfw::samples,
        "Multisampling sample count actually in use (0 = no anti-aliasing).");

    // Render methods
    pyOnlineRenderer.def("render", [](murosim::visualization::RendererGlfw &r,
                                       const std::shared_ptr<murosim::simulation::Simulation> sim) {
        r.draw(sim);
        // read the frame before the swap; post-swap back-buffer contents are undefined
        auto frame = convertFrameBufferToNumpyArray(r.getFrameBuffer());
        r.show();
        return frame;
    }, "Render a simulation to window and return the image.\n\n"
       "Args:\n"
       "    simulation: Simulation to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("simulation"));
    pyOnlineRenderer.def("render", [](murosim::visualization::RendererGlfw &r,
                                       const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        r.draw(simulations);
        // read the frame before the swap; post-swap back-buffer contents are undefined
        auto frame = convertFrameBufferToNumpyArray(r.getFrameBuffer());
        r.show();
        return frame;
    }, "Render multiple simulations in a grid layout to window and return the image.\n\n"
       "Args:\n"
       "    simulations: List of simulations to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("simulations"));
    pyOnlineRenderer.def("render", [](murosim::visualization::RendererGlfw &r,
                                       const std::shared_ptr<murosim::robot::Robot> robot) {
        r.draw(robot);
        // read the frame before the swap; post-swap back-buffer contents are undefined
        auto frame = convertFrameBufferToNumpyArray(r.getFrameBuffer());
        r.show();
        return frame;
    }, "Render a robot to window and return the image.\n\n"
       "Args:\n"
       "    robot: Robot to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("robot"));
    pyOnlineRenderer.def("render", [](murosim::visualization::RendererGlfw &r,
                                       const murosim::geometry::Shape &shape) {
        r.draw(shape);
        // read the frame before the swap; post-swap back-buffer contents are undefined
        auto frame = convertFrameBufferToNumpyArray(r.getFrameBuffer());
        r.show();
        return frame;
    }, "Render a shape to window and return the image.\n\n"
       "Args:\n"
       "    shape: Shape to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("shape"));
    pyOnlineRenderer.def("render", [](murosim::visualization::RendererGlfw &r,
                                       const murosim::geometry::World &world) {
        r.draw(world);
        // read the frame before the swap; post-swap back-buffer contents are undefined
        auto frame = convertFrameBufferToNumpyArray(r.getFrameBuffer());
        r.show();
        return frame;
    }, "Render a world to window and return the image.\n\n"
       "Args:\n"
       "    world: World to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("world"));
    pyOnlineRenderer.def("render", [](murosim::visualization::RendererGlfw &r,
                                       const murosim::geometry::World &,
                                       const murosim::io::DataContainer &data) {
        r.draw(data);
        // read the frame before the swap; post-swap back-buffer contents are undefined
        auto frame = convertFrameBufferToNumpyArray(r.getFrameBuffer());
        r.show();
        return frame;
    }, "Render a DataContainer to window and return the image.\n\n"
       "Args:\n"
       "    world: World (unused, for API compatibility)\n"
       "    data: DataContainer to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("world"), pybind11::arg("data"));
    pyOnlineRenderer.def("render", [](murosim::visualization::RendererGlfw &r,
                                       const std::vector<murosim::io::DataContainer> &containers) {
        r.draw(containers);
        // read the frame before the swap; post-swap back-buffer contents are undefined
        auto frame = convertFrameBufferToNumpyArray(r.getFrameBuffer());
        r.show();
        return frame;
    }, "Render multiple DataContainers in a grid layout to window and return the image.\n\n"
       "Args:\n"
       "    containers: Vector of DataContainers to render\n\n"
       "Returns:\n"
       "    numpy.ndarray of shape (height, width, 3), dtype uint8, channels in RGB order, top row first.",
       pybind11::arg("containers"));

    // Wait for key input
    pyOnlineRenderer.def("wait_key", &murosim::visualization::RendererGlfw::waitKey,
        "Wait for a key press in the renderer window (like cv2.waitKey).\n\n"
        "Args:\n"
        "    timeout_ms: Timeout in milliseconds. 0 = wait forever (default).\n\n"
        "Returns:\n"
        "    GLFW key code of the pressed key, or -1 on timeout/window close.",
        pybind11::arg("timeout_ms") = 0);

    pyOnlineRenderer.def_property_readonly("is_open", &murosim::visualization::RendererGlfw::isOpen,
        "False once the user has closed the window.");

    // Settings (write-only properties)
    pyOnlineRenderer.def_property("lidar_skip_steps",
        nullptr,
        [](murosim::visualization::RendererGlfw &r, long steps) {
            r.setLaserSkipSteps(static_cast<size_t>(std::max(0L, steps)));
        },
        "Draw every Nth lidar ray: 1 = all rays, 0 = none (the default). Write-only.");
    pyOnlineRenderer.def_property("robot_trajectory_length",
        nullptr,
        [](murosim::visualization::RendererGlfw &r, long len) {
            r.setRobotTrajectoryLength(static_cast<size_t>(std::max(0L, len)));
        },
        "Length of robot trajectory trail to render (0 = no trail, write-only).");
    pyOnlineRenderer.def_property("draw_robot_path",
        nullptr,
        [](murosim::visualization::RendererGlfw &r, bool draw) {
            r.setDrawRobotPath(draw);
        },
        "Whether to render the planner's path for each robot (write-only).");
    pyOnlineRenderer.def_property("draw_grid_map",
        nullptr,
        [](murosim::visualization::RendererGlfw &r, bool draw) {
            r.setDrawGridMap(draw);
        },
        "Whether to render the grid map (free/occupied/buffer cells, write-only).");

    // Python special methods
    pyOnlineRenderer.def("__repr__", [](const murosim::visualization::RendererGlfw &) {
        return "<OnlineRenderer (GLFW)>";
    }, "Return a debug string representation.");
}

} // namespace murosimpy
