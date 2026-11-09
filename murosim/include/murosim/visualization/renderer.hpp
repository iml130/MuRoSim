// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_RENDERER_HPP_
#define MUROSIM_VISUALIZATION_RENDERER_HPP_

#include "murosim/visualization/canvas.hpp"
#include "murosim/visualization/scenery.hpp"
#include "murosim/simulation/simulation.hpp"
#include "murosim/visualization/frame_buffer.hpp"
#include <GLES3/gl3.h>
#include <vector>
#include <EGL/egl.h>

namespace murosim::visualization
{

/// Abstract base for the two concrete OpenGL ES renderers (RendererEgl for
/// offscreen/headless rendering, RendererGlfw for an on-screen window). Owns
/// the shared drawing pipeline: a GlShaderManager, a Scenery describing style,
/// and the draw()/getFrameBuffer() entry points used to render murosim
/// worlds, shapes, robots and simulations and read the result back as an RGB
/// FrameBuffer. Subclasses only provide context management: creating/owning
/// the GL context/surface and implementing makeContextCurrent(). Every public
/// method makes the renderer's context current on the calling thread before
/// touching GL state, so a single Renderer must only be driven from one
/// thread at a time (concurrent renderers on different threads are fine, one
/// per thread/context). Not copyable.
class Renderer
{
private:
    std::shared_ptr<murosim::visualization::shader::GlShaderManager> shader_manager_;
    murosim::visualization::Scenery scenery_;

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    /// Creates a Canvas over the current shader manager with thickness sized
    /// to width_/height_ and the identity draw area (caller must still set a
    /// draw area for non-default framing).
    murosim::visualization::Canvas createCanvas();
    /// @copydoc createCanvas()
    /// Also sets the draw area to world's bounding box (with a small margin).
    murosim::visualization::Canvas createCanvas(const murosim::geometry::World &world);
    /// @copydoc createCanvas()
    /// Also sets the draw area to shape's bounding box (with a small margin).
    murosim::visualization::Canvas createCanvas(const murosim::geometry::Shape &shape);

    // Every frame is drawn into renderer-owned framebuffer objects of exactly
    // width_ x height_ pixels: a multisampled one when samples_ > 0, resolved
    // into a single-sample one that getFrameBuffer() reads and endFrame()
    // presents into the backend's default framebuffer. This keeps
    // anti-aliasing independent of the EGL/GLFW surface (Mesa's CPU renderer
    // has no usable multisampled pbuffer) and the read-back independent of
    // the window size (HiDPI windows are larger than requested).
    GLuint framebuffer_multisample_ = 0;
    GLuint color_multisample_ = 0;
    GLuint depth_multisample_ = 0;
    GLuint framebuffer_resolve_ = 0;
    GLuint color_resolve_ = 0;
    GLuint depth_resolve_ = 0;
    /// Creates the framebuffer objects; clamps samples_ to GL_MAX_SAMPLES.
    void createFramebuffers();
    void deleteFramebuffers();
    /// Makes the context current and binds the frame's draw target.
    void beginFrame();
    /// Resolves the multisampled frame and presents it into the default framebuffer.
    void endFrame();

protected:
    EGLint width_;
    EGLint height_;
    /// Multisampling sample count in use (0 = none); the requested count
    /// clamped to the driver's GL_MAX_SAMPLES by createFramebuffers().
    EGLint samples_;

    /// Makes the context current, sets the default GL state (viewport,
    /// clear color, depth test, alpha blending) and constructs the shader
    /// manager. Must be called by each subclass constructor once its context
    /// and surface exist.
    void initializeRenderer();
    // Derived destructors must call this while their GL context is still
    // current: the shader destructors issue glDelete* calls, and the base
    // class would otherwise destroy them after the context is gone.
    /// Deletes the shader manager and the framebuffer objects; the
    /// renderer's context must be current.
    void releaseShaders();
    /// Makes this renderer's GL context current on the calling thread.
    /// Implemented by each backend (EGL pbuffer surface, or GLFW window);
    /// called before every GL operation in the base class.
    virtual void makeContextCurrent() = 0;
    /// Size in pixels of the backend's default framebuffer, the target of the
    /// final presentation blit (the requested size for a pbuffer, the live
    /// window framebuffer for GLFW).
    virtual void getDefaultFramebufferSize(GLint &width, GLint &height) const = 0;

public:
    /// Default multisampling sample count for anti-aliasing. The count is
    /// clamped to the driver's maximum (Mesa's CPU renderer llvmpipe offers
    /// 4), so check samples() when the exact count matters.
    static constexpr EGLint MSAA_FACTOR = 8;

    /// @param width, height requested framebuffer resolution in pixels; both must be positive.
    /// @param samples requested multisampling sample count; 0 disables anti-aliasing.
    /// @throws std::invalid_argument if width or height is not positive or samples is negative.
    Renderer(const EGLint width, const EGLint height, const EGLint samples = MSAA_FACTOR);
    virtual ~Renderer() = default;

    /// Reads back the current framebuffer contents as a tightly-packed RGB
    /// FrameBuffer, converting from whatever format/type
    /// GL_IMPLEMENTATION_COLOR_READ_FORMAT/_TYPE report (RGB, RGBA, BGR or
    /// BGRA; only GL_UNSIGNED_BYTE component type is supported).
    /// @throws std::runtime_error if the reported format/type is unsupported.
    /// Reads the last drawn frame back as tightly packed RGB of exactly the
    /// requested width x height, from the renderer's resolved framebuffer.
    murosim::visualization::FrameBuffer getFrameBuffer();
    /// Multisampling sample count in use: the requested count clamped to the
    /// driver's maximum; 0 means no anti-aliasing.
    EGLint samples() const;
    /// Renders data's world and navigation nodes, filling the whole frame.
    void draw(const murosim::io::DataContainer &data);
    /// Renders each container's world/nodes into its own cell of a grid
    /// layout spanning the whole frame.
    void draw(const std::vector<murosim::io::DataContainer> &containers);
    /// Renders world's geometry, filling the whole frame.
    void draw(const murosim::geometry::World &world);
    /// Renders a single shape, filling the whole frame.
    void draw(const murosim::geometry::Shape &shape);
    /// Renders one simulation's world, grid map and robots, filling the whole frame.
    void draw(const std::shared_ptr<murosim::simulation::Simulation> &simulation);
    /// Renders a single robot (with its own pose applied), filling the whole frame.
    void draw(const std::shared_ptr<murosim::robot::Robot> &robot);
    /// Renders each simulation into its own cell of a grid layout spanning
    /// the whole frame.
    void draw(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations);

    /// @copydoc Scenery::setRobotTrajectoryLength
    void setRobotTrajectoryLength(const size_t trajectory_length);
    /// @copydoc Scenery::setLaserSkipSteps
    void setLaserSkipSteps(const size_t skip_steps);
    /// @copydoc Scenery::setDrawRobotPath
    void setDrawRobotPath(const bool draw_path);
    /// @copydoc Scenery::setDrawGridMap
    void setDrawGridMap(const bool draw_grid_map);
};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_RENDERER_HPP_
