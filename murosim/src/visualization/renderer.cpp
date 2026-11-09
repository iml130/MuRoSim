// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/renderer.hpp"
#include "murosim/visualization/frame_buffer.hpp"
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <utility>

constexpr float DEFAULT_MARGIN = 0.05f;

namespace
{
/// glReadPixels returns the bottom row first; images are stored top row first.
murosim::visualization::FrameBuffer flipRows(murosim::visualization::FrameBuffer frame_buffer)
{
    const size_t row_bytes = frame_buffer.width() * 3;
    uint8_t* const data = frame_buffer.data();
    for (size_t top = 0, bottom = frame_buffer.height() - 1; top < bottom; top++, bottom--)
    {
        std::swap_ranges(data + top * row_bytes, data + (top + 1) * row_bytes, data + bottom * row_bytes);
    }
    return frame_buffer;
}
} // namespace

namespace murosim::visualization
{

Renderer::Renderer(const EGLint width, const EGLint height, const EGLint samples)
    : shader_manager_(nullptr),
      scenery_(),
      width_(width),
      height_(height),
      samples_(samples)
{
    if (width_ <= 0 || height_ <= 0)
    {
        throw std::invalid_argument("[Renderer]: Width and height must be positive integers.");
    }
    if (samples_ < 0)
    {
        throw std::invalid_argument("[Renderer]: The sample count must not be negative.");
    }
}

EGLint Renderer::samples() const
{
    return samples_;
}

void Renderer::initializeRenderer()
{
    makeContextCurrent();
    // Initialize OpenGL settings

    // Default viewport matches the requested resolution. Backends whose real
    // framebuffer differs (e.g. GLFW on HiDPI/scaled displays) override this.
    glViewport(0, 0, width_, height_);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    shader_manager_ = std::make_shared<murosim::visualization::shader::GlShaderManager>();
    createFramebuffers();
}

void Renderer::createFramebuffers()
{
    GLint max_samples = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &max_samples);
    samples_ = std::min(samples_, max_samples);

    // Single-sample target: drawn into directly without multisampling, the
    // resolve destination otherwise; always what getFrameBuffer() reads
    glGenFramebuffers(1, &framebuffer_resolve_);
    glGenRenderbuffers(1, &color_resolve_);
    glGenRenderbuffers(1, &depth_resolve_);
    glBindRenderbuffer(GL_RENDERBUFFER, color_resolve_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width_, height_);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_resolve_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width_, height_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_resolve_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_resolve_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_resolve_);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        deleteFramebuffers();
        throw std::runtime_error("[Renderer]: Failed to create the render framebuffer.");
    }

    if (samples_ > 0)
    {
        glGenFramebuffers(1, &framebuffer_multisample_);
        glGenRenderbuffers(1, &color_multisample_);
        glGenRenderbuffers(1, &depth_multisample_);
        glBindRenderbuffer(GL_RENDERBUFFER, color_multisample_);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples_, GL_RGBA8, width_, height_);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_multisample_);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples_, GL_DEPTH_COMPONENT16, width_, height_);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_multisample_);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_multisample_);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_multisample_);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            deleteFramebuffers();
            throw std::runtime_error("[Renderer]: Failed to create the multisampled framebuffer.");
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::deleteFramebuffers()
{
    glDeleteFramebuffers(1, &framebuffer_multisample_);
    glDeleteRenderbuffers(1, &color_multisample_);
    glDeleteRenderbuffers(1, &depth_multisample_);
    glDeleteFramebuffers(1, &framebuffer_resolve_);
    glDeleteRenderbuffers(1, &color_resolve_);
    glDeleteRenderbuffers(1, &depth_resolve_);
    framebuffer_multisample_ = color_multisample_ = depth_multisample_ = 0;
    framebuffer_resolve_ = color_resolve_ = depth_resolve_ = 0;
}

void Renderer::beginFrame()
{
    makeContextCurrent();
    glBindFramebuffer(GL_FRAMEBUFFER, (samples_ > 0) ? framebuffer_multisample_ : framebuffer_resolve_);
    glViewport(0, 0, width_, height_);
}

void Renderer::endFrame()
{
    if (samples_ > 0)
    {
        // a multisampled source must be resolved 1:1
        glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer_multisample_);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer_resolve_);
        glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }

    // present into the default framebuffer, scaled if the window is larger
    GLint default_width = 0;
    GLint default_height = 0;
    getDefaultFramebufferSize(default_width, default_height);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer_resolve_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, default_width, default_height, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::releaseShaders()
{
    deleteFramebuffers();
    shader_manager_.reset();
}

murosim::visualization::Canvas Renderer::createCanvas()
{
    assert(shader_manager_ != nullptr);

    murosim::visualization::Canvas canvas = murosim::visualization::Canvas(shader_manager_);
    canvas.setDrawThickness(width_, height_);
    return canvas;
}

murosim::visualization::Canvas Renderer::createCanvas(const murosim::geometry::World &world)
{
    murosim::visualization::Canvas canvas = createCanvas();
    canvas.setDrawArea(world.getAabbMin(), world.getAabbMax(), DEFAULT_MARGIN);

    return canvas;
}

murosim::visualization::Canvas Renderer::createCanvas(const murosim::geometry::Shape &shape)
{
    murosim::visualization::Canvas canvas = createCanvas();
    canvas.setDrawArea(shape.getAabbMin(), shape.getAabbMax(), DEFAULT_MARGIN);
    return canvas;
}

void Renderer::draw(const murosim::io::DataContainer &data)
{
    murosim::geometry::World world = data.createWorld();
    murosim::navigation::Container node_container = data.createNodeContainer();

    beginFrame();
    murosim::visualization::Canvas canvas = createCanvas(world);

    scenery_.drawSceneryWorldWithNodes(canvas, world, node_container);
    endFrame();
}

void Renderer::draw(const std::vector<murosim::io::DataContainer> &containers)
{
    beginFrame();
    murosim::visualization::Canvas canvas = createCanvas();

    scenery_.drawSceneryDataContainers(canvas, containers);
    endFrame();
}

void Renderer::draw(const murosim::geometry::World &world)
{
    beginFrame();
    murosim::visualization::Canvas canvas = createCanvas(world);

    scenery_.drawSceneryWorld(canvas, world);
    endFrame();
}

void Renderer::draw(const murosim::geometry::Shape &shape)
{
    beginFrame();
    murosim::visualization::Canvas canvas = createCanvas(shape);

    scenery_.drawSceneryShape(canvas, shape);
    endFrame();
}

void Renderer::draw(const std::shared_ptr<murosim::simulation::Simulation> &simulation)
{
    assert(simulation != nullptr);

    beginFrame();
    murosim::visualization::Canvas canvas = createCanvas(simulation->getWorld());

    scenery_.drawScenerySimulation(canvas, *simulation);
    endFrame();
}

void Renderer::draw(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    assert(robot != nullptr);

    beginFrame();
    murosim::visualization::Canvas canvas = createCanvas(robot->getShapeRobot());

    scenery_.drawSceneryRobot(canvas, *robot);
    endFrame();
}

void Renderer::draw(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations)
{
    beginFrame();
    murosim::visualization::Canvas canvas = createCanvas();

    scenery_.drawScenerySimulations(canvas, simulations);
    endFrame();
}

murosim::visualization::FrameBuffer Renderer::getFrameBuffer()
{
    makeContextCurrent();

    assert(width_ > 0);
    assert(height_ > 0);

    if (width_ <= 0 || height_ <= 0) {
        throw std::runtime_error("[Renderer]: Width and height must be positive integers for FrameBuffer.");
    }

    // Read the resolved frame: exactly width_ x height_, single-sample
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer_resolve_);

    GLint format = 0, type = 0;
    glGetIntegerv(GL_IMPLEMENTATION_COLOR_READ_FORMAT, &format);
    glGetIntegerv(GL_IMPLEMENTATION_COLOR_READ_TYPE, &type);

    assert(format != 0);
    assert(type != 0);

    if (format == 0 || type == 0) {
        throw std::runtime_error(
            "[Renderer]: Failed to get GL_IMPLEMENTATION_COLOR_READ_FORMAT or GL_IMPLEMENTATION_COLOR_READ_TYPE.");
    }

    if (type != GL_UNSIGNED_BYTE)
    {
        throw std::runtime_error("[Renderer]: Only GL_UNSIGNED_BYTE is supported.");
    }

    // glReadPixels honours GL_PACK_ALIGNMENT (default 4). For the 3-byte RGB/BGR
    // formats, a width whose row stride (width*3) is not 4-aligned would be read
    // back row-padded, which both corrupts the tightly packed FrameBuffer and
    // overruns its buffer. Force tight packing.
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    const size_t width = static_cast<size_t>(width_);
    const size_t height = static_cast<size_t>(height_);
    const size_t number_of_pixels = width * height;

    // Fast path: direct RGB read
    if (format == GL_RGB) {
        murosim::visualization::FrameBuffer frame_buffer(width, height);
        glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, frame_buffer.data());
        return flipRows(std::move(frame_buffer));
    }
    else if(format == GL_RGBA)
    {
        murosim::visualization::FrameBuffer frame_buffer(width, height);
        std::vector<uint8_t> pixels(number_of_pixels * 4);
        glReadPixels(0, 0, width_, height_, static_cast<GLenum>(format), GL_UNSIGNED_BYTE, pixels.data());

        uint8_t* dst = frame_buffer.data();
        const uint8_t* src = pixels.data();
        const uint8_t* const src_end = src + pixels.size();

        while (src < src_end) {
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst += 3;
            src += 4;
        }

        return flipRows(std::move(frame_buffer));
    }
    else if (format == GL_BGR_EXT)
    {
        murosim::visualization::FrameBuffer frame_buffer(width, height);
        std::vector<uint8_t> pixels(number_of_pixels * 3);
        glReadPixels(0, 0, width_, height_, static_cast<GLenum>(format), GL_UNSIGNED_BYTE, pixels.data());

        uint8_t* dst = frame_buffer.data();
        const uint8_t* src = pixels.data();
        const uint8_t* const src_end = src + pixels.size();

        while (src < src_end) {
            dst[0] = src[2];
            dst[1] = src[1];
            dst[2] = src[0];
            dst += 3;
            src += 3;
        }

        return flipRows(std::move(frame_buffer));
    }
    else if (format == GL_BGRA_EXT)
    {
        murosim::visualization::FrameBuffer frame_buffer(width, height);
        std::vector<uint8_t> pixels(number_of_pixels * 4);
        glReadPixels(0, 0, width_, height_, static_cast<GLenum>(format), GL_UNSIGNED_BYTE, pixels.data());

        uint8_t* dst = frame_buffer.data();
        const uint8_t* src = pixels.data();
        const uint8_t* const src_end = src + pixels.size();

        while (src < src_end) {
            dst[0] = src[2];
            dst[1] = src[1];
            dst[2] = src[0];
            dst += 3;
            src += 4;
        }

        return flipRows(std::move(frame_buffer));
    }
    else
    {
        throw std::runtime_error("[Renderer]: Unsupported format for FrameBuffer: " + std::to_string(format));
    }
}

void Renderer::setRobotTrajectoryLength(const size_t trajectory_length)
{
    scenery_.setRobotTrajectoryLength(trajectory_length);
}

void Renderer::setLaserSkipSteps(const size_t skip_steps)
{
    scenery_.setLaserSkipSteps(skip_steps);
}

void Renderer::setDrawRobotPath(const bool draw_path)
{
    scenery_.setDrawRobotPath(draw_path);
}

void Renderer::setDrawGridMap(const bool draw_grid_map)
{
    scenery_.setDrawGridMap(draw_grid_map);
}

} // namespace murosim::visualization
