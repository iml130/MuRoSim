// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/renderer_glfw.hpp"

#include <chrono>

namespace murosim::visualization
{

RendererGlfw::RendererGlfw(const EGLint resolution, const EGLint samples)
    : Renderer(resolution, resolution, samples),
      window_(nullptr)
{
    if (width_ <= 0 || height_ <= 0)
    {
        throw std::invalid_argument("[RendererGlfw]: Width and height must be positive integers.");
    }

    // Initialize GLFW
    GlfwGuard::instance(); // Ensure GLFW is initialized

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    glfwWindowHint(GLFW_SAMPLES, 0); // anti-aliasing happens in the renderer's own framebuffer
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    window_ = glfwCreateWindow(width_, height_, "MuRoSim", nullptr, nullptr);
    if (!window_) {
        // GLFW init/termination is owned by the GlfwGuard singleton; terminating
        // it here would leave the guard thinking GLFW is still live, breaking any
        // later renderer.
        throw std::runtime_error("[RendererGlfw]: Failed to create GLFW window.");
    }

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, keyCallback);

    makeContextCurrent();
    glfwSwapInterval( 0 ); // Disable vsync
    initializeRenderer();
}

RendererGlfw::~RendererGlfw()
{
    if (window_) {
        // Delete the GL objects while this window's context is current —
        // after glfwDestroyWindow the shader destructors would run against
        // no context (or another renderer's).
        glfwMakeContextCurrent(window_);
        releaseShaders();
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
}

void RendererGlfw::show()
{
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void RendererGlfw::keyCallback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/)
{
    if (action == GLFW_PRESS)
    {
        auto* self = static_cast<RendererGlfw*>(glfwGetWindowUserPointer(window));
        self->last_key_ = key;
    }
}

int RendererGlfw::waitKey(int timeout_ms)
{
    last_key_ = -1;
    auto start = std::chrono::steady_clock::now();

    while (last_key_ == -1 && !glfwWindowShouldClose(window_))
    {
        if (timeout_ms > 0)
        {
            glfwWaitEventsTimeout(timeout_ms / 1000.0);
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed >= timeout_ms) break;
        }
        else
        {
            glfwWaitEvents(); // block until any event
        }
    }
    return last_key_;
}

bool RendererGlfw::isOpen() const
{
    return !glfwWindowShouldClose(window_);
}

void RendererGlfw::makeContextCurrent()
{
    glfwMakeContextCurrent(window_);
}

void RendererGlfw::getDefaultFramebufferSize(GLint &width, GLint &height) const
{
    // On HiDPI/fractionally-scaled displays the framebuffer is larger than the
    // requested window (e.g. 1450x1450 for a 1000x1000 window at 145%), and
    // the window manager only applies that scaling after the window is
    // mapped, so it is queried for every frame rather than once at
    // construction. The frame is drawn at the requested size and scaled into
    // this framebuffer when presented.
    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window_, &framebuffer_width, &framebuffer_height);
    width = framebuffer_width;
    height = framebuffer_height;
}

} // namespace murosim::visualization
