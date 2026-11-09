// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_RENDERER_GLFW_HPP_
#define MUROSIM_VISUALIZATION_RENDERER_GLFW_HPP_

#include "murosim/visualization/renderer.hpp"
#include "murosim/visualization/glfw_guard.hpp"

namespace murosim::visualization
{

/// On-screen Renderer backed by a GLFW window and OpenGL ES 3 context.
/// Requires a display/windowing system to be available (unlike RendererEgl);
/// use this for interactive visualization/debugging, e.g. from
/// murosim_demo. GLFW itself is initialized process-wide via GlfwGuard on
/// first use. Vsync is disabled. The viewport is re-queried from the live
/// framebuffer size on every makeContextCurrent() call (before each draw) to
/// track HiDPI/fractional display scaling, which some window managers only
/// apply after the window is mapped.
class RendererGlfw : public Renderer
{
private:
    GLFWwindow* window_;
    int last_key_ = -1;

    /// GLFW key callback (registered via glfwSetKeyCallback); records the
    /// last key pressed (GLFW_PRESS only) on the RendererGlfw instance
    /// retrieved from the window's user pointer, for waitKey() to consume.
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

public:
    /// Creates a non-resizable resolution x resolution window (with
    /// multisampling per Renderer::MSAA_FACTOR) and its GL ES 3 context.
    /// @throws std::invalid_argument if resolution is not positive.
    /// @throws std::runtime_error if the GLFW window cannot be created.
    /// @param resolution Square window size in pixels, must be positive.
    /// @param samples Requested multisampling sample count (0 = off), clamped
    /// to the driver's maximum; see Renderer::samples().
    RendererGlfw(const EGLint resolution, const EGLint samples = Renderer::MSAA_FACTOR);
    /// Destroys the shaders while the window's context is still current,
    /// then destroys the window. Does not terminate GLFW itself (owned by
    /// GlfwGuard).
    ~RendererGlfw();

    /// Swaps the front/back buffers to present the last drawn frame and
    /// polls window/input events. Call after a draw() to make it visible.
    void show();

    /// Wait for a key press. Returns the GLFW key code, or -1 on timeout/window close.
    /// @param timeout_ms Timeout in milliseconds. 0 = wait forever.
    int waitKey(int timeout_ms = 0);

    /// False once the user has closed the window.
    bool isOpen() const;

    /// Makes this renderer's window's GL context current on the calling
    /// thread and re-syncs the GL viewport to the window's live framebuffer
    /// size (see class documentation).
    virtual void makeContextCurrent() override;

protected:
    /// The live window framebuffer, which HiDPI scaling makes larger than requested.
    void getDefaultFramebufferSize(GLint &width, GLint &height) const override;

};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_RENDERER_GLFW_HPP_
