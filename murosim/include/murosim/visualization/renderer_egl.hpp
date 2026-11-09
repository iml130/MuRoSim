// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_RENDERER_EGL_HPP_
#define MUROSIM_VISUALIZATION_RENDERER_EGL_HPP_

#include "murosim/visualization/renderer.hpp"
#include <EGL/egl.h>

namespace murosim::visualization
{

/// Headless/offscreen Renderer backed by an EGL pbuffer surface and an
/// OpenGL ES 3 context, with no visible window. Prefers the
/// EGL_MESA_platform_surfaceless extension where available (typical of CI and
/// server environments without a display), falling back to the default EGL
/// display otherwise. Intended for rendering to a FrameBuffer via
/// Renderer::getFrameBuffer() rather than on-screen display; use RendererGlfw
/// for an interactive window instead.
class RendererEgl : public Renderer
{
private:
    EGLDisplay display_;
    EGLSurface surface_;
    EGLContext context_;
public:
    /// Creates a square resolution x resolution pbuffer surface and EGL
    /// context and initializes the shared renderer state.
    /// @throws std::runtime_error if any EGL step (display, init, config,
    ///         surface or context creation) fails.
    /// @param resolution Square framebuffer size in pixels, must be positive.
    /// @param samples Requested multisampling sample count (0 = off), clamped
    /// to the driver's maximum; see Renderer::samples().
    RendererEgl(const EGLint resolution, const EGLint samples = Renderer::MSAA_FACTOR);
    /// Destroys the shaders while the context is still current, then tears
    /// down the EGL context, surface, display connection in that order.
    ~RendererEgl();

    /// Makes this renderer's EGL context current on the calling thread.
    /// @throws std::runtime_error if eglMakeCurrent fails.
    virtual void makeContextCurrent() override;

protected:
    /// The pbuffer has exactly the requested size.
    void getDefaultFramebufferSize(GLint &width, GLint &height) const override;
};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_RENDERER_EGL_HPP_
