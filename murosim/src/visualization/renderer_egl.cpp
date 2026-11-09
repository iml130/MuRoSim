// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/renderer_egl.hpp"

#include <EGL/eglext.h>
#include <cstring>

namespace murosim::visualization
{

static EGLDisplay getSurfacelessDisplay()
{
    const char* extensions = eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS);
    if (extensions && std::strstr(extensions, "EGL_MESA_platform_surfaceless"))
    {
        auto eglGetPlatformDisplayEXT =
            reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
        if (eglGetPlatformDisplayEXT)
        {
            EGLDisplay display = eglGetPlatformDisplayEXT(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
            if (display != EGL_NO_DISPLAY)
            {
                return display;
            }
        }
    }
    return eglGetDisplay(EGL_DEFAULT_DISPLAY);
}

/// First single-sample OpenGL ES 3 pbuffer config, or nullptr if the driver
/// has none. Anti-aliasing is done in the renderer's own multisampled
/// framebuffer, so the surface itself never needs multisampling (which
/// Mesa's CPU renderer cannot provide for pbuffers anyway).
static EGLConfig chooseSingleSampleConfig(EGLDisplay display)
{
    const EGLint attribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_BLUE_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_RED_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 8,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_SAMPLE_BUFFERS, 0,
        EGL_NONE
    };

    // EGL_SAMPLE_BUFFERS matches "at least", so pick a config without samples
    constexpr EGLint MAX_CONFIGS = 64;
    EGLConfig configs[MAX_CONFIGS];
    EGLint num_configs = 0;
    if (!eglChooseConfig(display, attribs, configs, MAX_CONFIGS, &num_configs))
    {
        return nullptr;
    }
    for (EGLint i = 0; i < num_configs; i++)
    {
        EGLint config_samples = 0;
        eglGetConfigAttrib(display, configs[i], EGL_SAMPLES, &config_samples);
        if (config_samples == 0)
        {
            return configs[i];
        }
    }
    return nullptr;
}

RendererEgl::RendererEgl(const EGLint resolution, const EGLint samples)
    : Renderer(resolution, resolution, samples),
      display_(EGL_NO_DISPLAY),
      surface_(EGL_NO_SURFACE),
      context_(EGL_NO_CONTEXT)
{
    // Get the display (prefer surfaceless for headless environments)
    display_ = getSurfacelessDisplay();
    if (display_ == EGL_NO_DISPLAY) {
        throw std::runtime_error("[RendererEgl]: Failed to get EGL display.");
    }

    // Initialize EGL
    if (!eglInitialize(display_, nullptr, nullptr)) {
        throw std::runtime_error("[RendererEgl]: Failed to initialize EGL.");
    }

    const EGLConfig config = chooseSingleSampleConfig(display_);
    if (config == nullptr) {
        throw std::runtime_error("[RendererEgl]: No matching EGL config found (OpenGL ES 3 pbuffer unsupported?).");
    }

    // Create an EGL surface
    EGLint surface_attribs[] = {
        EGL_WIDTH, width_,
        EGL_HEIGHT, height_,
        EGL_NONE
    };
    surface_ = eglCreatePbufferSurface(display_, config, surface_attribs);
    if (surface_ == EGL_NO_SURFACE) {
        throw std::runtime_error("[RendererEgl]: Failed to create EGL surface.");
    }

    // Create an EGL context
    EGLint contextAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
    context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, contextAttribs);
    if (context_ == EGL_NO_CONTEXT) {
        throw std::runtime_error("[RendererEgl]: Failed to create EGL context.");
    }

    makeContextCurrent();
    initializeRenderer();
}

RendererEgl::~RendererEgl()
{
    // Delete the GL objects while the context is still alive — the base
    // class destroys the shaders only after this body, which would be after
    // eglTerminate.
    if (display_ != EGL_NO_DISPLAY && context_ != EGL_NO_CONTEXT) {
        eglMakeCurrent(display_, surface_, surface_, context_);
    }
    releaseShaders();

    if (display_ != EGL_NO_DISPLAY) {
        eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
    if (context_ != EGL_NO_CONTEXT) {
        eglDestroyContext(display_, context_);
    }
    if (surface_ != EGL_NO_SURFACE) {
        eglDestroySurface(display_, surface_);
    }
    if (display_ != EGL_NO_DISPLAY) {
        eglTerminate(display_);
    }
}

void RendererEgl::makeContextCurrent()
{
    if (!eglMakeCurrent(display_, surface_, surface_, context_)) {
        throw std::runtime_error("[RendererEgl]: Failed to make EGL context current.");
    }
}

void RendererEgl::getDefaultFramebufferSize(GLint &width, GLint &height) const
{
    width = width_;
    height = height_;
}

} // namespace murosim::visualization
