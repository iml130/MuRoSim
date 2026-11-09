// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/shader_circle.hpp"
#include "murosim/visualization/shader/util.hpp"
#include <Eigen/Core>
#include <vector>
#include <cmath>

constexpr size_t CIRCLE_RESOLUTION = 36;

namespace
{

const char *VERTEX_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision highp float;

    layout(location = 0) in vec2 a_CircleInstance; // Vertex for a single circle (unit circle)
    layout(location = 1) in vec3 a_CircleData;     // Instance data: x, y, radius for each circle
    layout(location = 2) in float a_CircleDepth;   // Instance data: depth for each circle
    layout(location = 3) in vec4 a_CircleColor;    // Color of the circle

    out vec4 v_Color; // Color to be passed to the fragment shader

    void main()
    {
        vec2 position = a_CircleData.xy + a_CircleData.z * a_CircleInstance;
        gl_Position = vec4(position, a_CircleDepth, 1.0);
        v_Color = a_CircleColor;
    }
)glsl";

const char *FRAGMENT_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision mediump float;

    in vec4 v_Color; // Color of the circle
    out vec4 o_FragColor;

    void main() {
        o_FragColor = v_Color;
    }
)glsl";

} // namespace


namespace murosim::visualization::shader
{

ShaderCircle::ShaderCircle(): Shader("ShaderCircle")
{
    shader_program_ = murosim::visualization::shader::util::compileShaderProgram(VERTEX_SHADER_SOURCE, FRAGMENT_SHADER_SOURCE);

    // CircleData struct layout: x, y, radius, depth, r, g, b, a (total 8 floats)
    setupInstancedFan(VAO_circle_, VBO_circle_, VBO_instance_, CIRCLE_RESOLUTION, "a_CircleInstance", sizeof(CircleData),
                      {{"a_CircleData", 3, 0}, {"a_CircleDepth", 1, 3}, {"a_CircleColor", 4, 4}});
}

ShaderCircle::~ShaderCircle()
{
    glDeleteVertexArrays(1, &VAO_circle_);
    glDeleteBuffers(1, &VBO_circle_);
    glDeleteBuffers(1, &VBO_instance_);
}

void ShaderCircle::draw(const std::vector<CircleData> &circles)
{
    if (circles.empty()) return;

    drawInstancedFan(VAO_circle_, VBO_instance_, CIRCLE_RESOLUTION,
                     circles.data(), circles.size() * sizeof(CircleData), static_cast<GLsizei>(circles.size()));
}

} // namespace murosim::visualization::shader
