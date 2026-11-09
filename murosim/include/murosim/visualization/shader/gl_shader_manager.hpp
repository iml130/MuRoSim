// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_GL_SHADER_MANAGER_HPP_
#define MUROSIM_VISUALIZATION_SHADER_GL_SHADER_MANAGER_HPP_

#include "murosim/visualization/shader/shader_circle.hpp"
#include "murosim/visualization/shader/shader_line.hpp"
#include "murosim/visualization/shader/shader_path.hpp"
#include "murosim/visualization/shader/shader_quad.hpp"
#include "murosim/visualization/shader/shader_ring.hpp"

#include <mutex>

namespace murosim::visualization::shader
{

/// Owns one instance of every murosim shader (circle, line, path, quad,
/// ring) and exposes a single draw*() entry point per primitive type, used
/// by Canvas to issue its batched/immediate draws. Guards all drawing (and
/// background clearing) with an internal mutex so calls from different
/// threads are serialized rather than racing on shared GL state; this does
/// not make it safe to call from a thread without the owning GL context
/// current, only safe with respect to other GlShaderManager calls. One
/// instance is created per Renderer in Renderer::initializeRenderer() and
/// shared with every Canvas the renderer creates.
class GlShaderManager
{
private:
    std::mutex mutex_;
    murosim::visualization::shader::ShaderCircle shader_circle_;
    murosim::visualization::shader::ShaderLine shader_line_;
    murosim::visualization::shader::ShaderPath shader_path_;
    murosim::visualization::shader::ShaderQuad shader_quad_;
    murosim::visualization::shader::ShaderRing shader_ring_;
public:
    /// Compiles and initializes all five shaders. Requires a current GL context.
    GlShaderManager();
    /// Draws filled circles in one instanced call.
    void drawCircle(const std::vector<murosim::visualization::shader::ShaderCircle::CircleData> &circles);
    /// Draws line segments in one instanced call.
    void drawLine(const std::vector<murosim::visualization::shader::ShaderLine::LineData> &lines);
    /// Draws a single path as a connected ribbon.
    void drawPath(const std::vector<murosim::visualization::shader::ShaderPath::PathData> &path);
    /// Draws multiple independent paths, each its own ribbon, from one upload.
    void drawPath(const std::vector<std::vector<murosim::visualization::shader::ShaderPath::PathData>> &paths);
    /// Draws axis-aligned quads in one instanced call.
    void drawQuad(const std::vector<murosim::visualization::shader::ShaderQuad::QuadData> &quads);
    /// Draws unfilled circle outlines (rings) in one instanced call.
    void drawRing(const std::vector<murosim::visualization::shader::ShaderRing::RingData> &rings);
    /// Clears the color and depth buffers to the given RGBA color.
    void drawAndClearBackground(const GLfloat red, const GLfloat green, const GLfloat blue, const GLfloat alpha);
};

} // namespace murosim::visualization::shader

#endif // MUROSIM_VISUALIZATION_SHADER_GL_SHADER_MANAGER_HPP_
