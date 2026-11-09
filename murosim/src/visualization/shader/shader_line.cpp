// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/shader_line.hpp"
#include "murosim/visualization/shader/util.hpp"

namespace {

static const char *VERTEX_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision highp float;

    layout(location = 0) in vec2 a_LineInstance;   // Vertex position of the line segment (-1 or 1 for along line, -1 or 1 for perpendicular)
    layout(location = 1) in vec4 a_LineData;       // Start (xy) and end (zw) points of the line segment in screen space
    layout(location = 2) in float a_LineThickness; // Thickness of the line segment
    layout(location = 3) in float a_LineDepth;     // Depth value for the line segment
    layout(location = 4) in vec4 a_LineColor;      // Color of the line segment

    out vec4 v_Color; // Color to be passed to the fragment shader

    void main() {
        // Transform start and end points from screen space to clip space
        vec2 p1 = a_LineData.xy;
        vec2 p2 = a_LineData.zw;

        // Calculate direction and length
        vec2 direction = p2 - p1;
        float len = length(direction);

        // Handle zero-length lines to avoid division by zero and flickering
        if (len == 0.0) {
            gl_Position = vec4(0.0, 0.0, a_LineDepth, 1.0); // Make it a point at the specified depth
            return;
        }

        direction = normalize(direction);
        vec2 perpendicular = vec2(-direction.y, direction.x); // Perpendicular vector

        // Calculate the offset for thickness in clip space
        vec2 thicknessOffset = perpendicular * a_LineThickness * 0.5;

        // Calculate endpoint extension for thickness
        // This will extend the endpoints by half the thickness along the line direction
        vec2 endpointExtension = direction * a_LineThickness * 0.5;

        // Determine the position along the line segment in clip space
        // a_LineInstance.x will be -1 for the start point, 1 for the end point
        // Extend endpoints by thickness
        vec2 positionAlongLine = mix(p1 - endpointExtension, p2 + endpointExtension, (a_LineInstance.x + 1.0) * 0.5);

        // Apply thickness offset based on a_LineInstance.y
        // a_LineInstance.y will be -1 for one side of the line, 1 for the other side
        vec2 finalPosition = positionAlongLine + thicknessOffset * a_LineInstance.y;

        gl_Position = vec4(finalPosition, a_LineDepth, 1.0);

        // Pass the color to the fragment shader
        v_Color = a_LineColor;
    }
)glsl";


static const char *FRAGMENT_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision mediump float;

    in vec4 v_Color; // Color of the line segment
    out vec4 o_FragColor;

    void main() {
        o_FragColor = v_Color;
    }
)glsl";

} // namespace

namespace murosim::visualization::shader
{

ShaderLine::ShaderLine():Shader("ShaderLine")
{
    shader_program_ = murosim::visualization::shader::util::compileShaderProgram(VERTEX_SHADER_SOURCE, FRAGMENT_SHADER_SOURCE);

    // LineData layout: p1_x, p1_y, p2_x, p2_y, thickness, depth, r, g, b, a (10 floats)
    setupInstancedStrip(VAO_line_, VBO_line_, VBO_instance_, "a_LineInstance", sizeof(LineData),
                        {{"a_LineData", 4, 0}, {"a_LineThickness", 1, 4}, {"a_LineDepth", 1, 5}, {"a_LineColor", 4, 6}});
}

ShaderLine::~ShaderLine()
{
    glDeleteVertexArrays(1, &VAO_line_);
    glDeleteBuffers(1, &VBO_line_);
    glDeleteBuffers(1, &VBO_instance_);
}

void ShaderLine::draw(const std::vector<ShaderLine::LineData>& lines) {
    if (lines.empty()) {
        return;
    }

    drawInstancedStrip(VAO_line_, VBO_instance_,
                       lines.data(), lines.size() * sizeof(ShaderLine::LineData), static_cast<GLsizei>(lines.size()));
}

} // namespace murosim::visualization::shader
