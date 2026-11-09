// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/shader_quad.hpp"
#include "murosim/visualization/shader/util.hpp"
#include <array>

namespace {

static const char *VERTEX_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision highp float;

    layout(location = 0) in vec2 a_QuadInstance;   // Vertex position of the quad (-1 or 1 for x, -1 or 1 for y)
    layout(location = 1) in vec4 a_QuadPosSize;    // Position (xy) and size (zw) of the quad in screen space
    layout(location = 2) in float a_QuadDepth;     // Depth value for the quad
    layout(location = 3) in vec4 a_QuadColor;      // Color of the quad

    out vec4 v_Color; // Color to be passed to the fragment shader

    void main() {
        // We now have separate size_x and size_y components.
        // We'll use a_QuadPosSize.zw as a vec2 for size.
        vec2 size = a_QuadPosSize.zw;
        vec2 position = a_QuadPosSize.xy + size * 0.5f * a_QuadInstance;
        gl_Position = vec4(position, a_QuadDepth, 1.0);

        v_Color = a_QuadColor;
    }
)glsl";

static const char *FRAGMENT_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision mediump float;

    in vec4 v_Color; // Color of the quad
    out vec4 o_FragColor;

    void main() {
        o_FragColor = v_Color;
    }
)glsl";

} // namespace

namespace murosim::visualization::shader
{

ShaderQuad::ShaderQuad():Shader("ShaderQuad")
{
    shader_program_ = murosim::visualization::shader::util::compileShaderProgram(VERTEX_SHADER_SOURCE, FRAGMENT_SHADER_SOURCE);

    // QuadData layout: x, y, size_x, size_y, depth, r, g, b, a (9 floats)
    setupInstancedStrip(VAO_quad_, VBO_quad_, VBO_instance_, "a_QuadInstance", sizeof(QuadData),
                        {{"a_QuadPosSize", 4, 0}, {"a_QuadDepth", 1, 4}, {"a_QuadColor", 4, 5}});
}

ShaderQuad::~ShaderQuad()
{
    glDeleteVertexArrays(1, &VAO_quad_);
    glDeleteBuffers(1, &VBO_quad_);
    glDeleteBuffers(1, &VBO_instance_);
}

void ShaderQuad::draw(const std::vector<ShaderQuad::QuadData>& quads) {
    if (quads.empty()) {
        return;
    }

    drawInstancedStrip(VAO_quad_, VBO_instance_,
                       quads.data(), quads.size() * sizeof(ShaderQuad::QuadData), static_cast<GLsizei>(quads.size()));
}

} // namespace murosim::visualization::shader
