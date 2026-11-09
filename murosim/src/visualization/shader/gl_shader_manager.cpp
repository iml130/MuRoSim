// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/gl_shader_manager.hpp"

namespace murosim::visualization::shader
{
GlShaderManager::GlShaderManager()
    : shader_circle_(),
      shader_line_(),
      shader_path_(),
      shader_quad_(),
      shader_ring_()
{
}

void GlShaderManager::drawCircle(const std::vector<murosim::visualization::shader::ShaderCircle::CircleData> &circles)
{
    std::lock_guard<std::mutex> lock(mutex_);
    shader_circle_.draw(circles);
}

void GlShaderManager::drawLine(const std::vector<murosim::visualization::shader::ShaderLine::LineData> &lines)
{
    std::lock_guard<std::mutex> lock(mutex_);
    shader_line_.draw(lines);
}

void GlShaderManager::drawPath(const std::vector<murosim::visualization::shader::ShaderPath::PathData> &path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    shader_path_.draw(path);
}

void GlShaderManager::drawPath(const std::vector<std::vector<murosim::visualization::shader::ShaderPath::PathData>> &paths)
{
    std::lock_guard<std::mutex> lock(mutex_);
    shader_path_.draw(paths);
}

void GlShaderManager::drawQuad(const std::vector<murosim::visualization::shader::ShaderQuad::QuadData> &quads)
{
    std::lock_guard<std::mutex> lock(mutex_);
    shader_quad_.draw(quads);
}

void GlShaderManager::drawRing(const std::vector<murosim::visualization::shader::ShaderRing::RingData> &rings)
{
    std::lock_guard<std::mutex> lock(mutex_);
    shader_ring_.draw(rings);
}

void GlShaderManager::drawAndClearBackground(const GLfloat red, const GLfloat green, const GLfloat blue, const GLfloat alpha)
{
    std::lock_guard<std::mutex> lock(mutex_);
    glClearColor(red, green, blue, alpha);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


} // namespace murosim::visualization::shader
