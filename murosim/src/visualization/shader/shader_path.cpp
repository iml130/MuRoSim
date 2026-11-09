// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/shader_path.hpp"

#include "murosim/visualization/shader/util.hpp"

#include <utility>
#include <vector>

namespace
{

const char* VERTEX_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision highp float;

    // Per-vertex ribbon data (2 vertices per path point)
    layout(location = 0) in float a_Side;       // -1 = left, +1 = right
    layout(location = 1) in vec2  a_Position;   // Current point position
    layout(location = 2) in vec2  a_Direction;  // Tangent direction (normalized)
    layout(location = 3) in float a_Thickness;  // Line thickness
    layout(location = 4) in float a_Depth;      // Z depth
    layout(location = 5) in vec4  a_Color;      // RGBA color

    out vec4 v_Color;

    void main()
    {
        // Compute normal from tangent direction (perpendicular)
        vec2 normal = vec2(-a_Direction.y, a_Direction.x);

        // Offset position by half thickness in normal direction
        vec2 offset = normal * a_Side * a_Thickness * 0.5;
        vec2 position = a_Position + offset;

        gl_Position = vec4(position, a_Depth, 1.0);
        v_Color = a_Color;
    }
)glsl";


const char* FRAGMENT_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision mediump float;

    in vec4 v_Color;
    out vec4 FragColor;

    void main()
    {
        FragColor = v_Color;
    }
)glsl";

} // namespace


namespace murosim::visualization::shader
{

ShaderPath::ShaderPath(): murosim::visualization::shader::Shader("ShaderPath")
{
    shader_program_ = murosim::visualization::shader::util::compileShaderProgram(VERTEX_SHADER_SOURCE, FRAGMENT_SHADER_SOURCE);

    glGenVertexArrays(1, &VAO_path_);
    glGenBuffers(1, &VBO_vertex_);

    glBindVertexArray(VAO_path_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_vertex_);

    // All attributes are per-vertex (not instanced) and interleaved in PathVertex.
    const GLsizei stride = sizeof(PathVertex);

    const GLuint attrib_side = getAttribLocation("a_Side");
    glVertexAttribPointer(attrib_side, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(attrib_side);

    const GLuint attrib_position = getAttribLocation("a_Position");
    glVertexAttribPointer(attrib_position, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(1 * sizeof(GLfloat)));
    glEnableVertexAttribArray(attrib_position);

    const GLuint attrib_direction = getAttribLocation("a_Direction");
    glVertexAttribPointer(attrib_direction, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(attrib_direction);

    const GLuint attrib_thickness = getAttribLocation("a_Thickness");
    glVertexAttribPointer(attrib_thickness, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(5 * sizeof(GLfloat)));
    glEnableVertexAttribArray(attrib_thickness);

    const GLuint attrib_depth = getAttribLocation("a_Depth");
    glVertexAttribPointer(attrib_depth, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(GLfloat)));
    glEnableVertexAttribArray(attrib_depth);

    const GLuint attrib_color = getAttribLocation("a_Color");
    glVertexAttribPointer(attrib_color, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(7 * sizeof(GLfloat)));
    glEnableVertexAttribArray(attrib_color);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

ShaderPath::~ShaderPath()
{
    glDeleteVertexArrays(1, &VAO_path_);
    glDeleteBuffers(1, &VBO_vertex_);
}

GLsizei ShaderPath::expandPath(const std::vector<PathData>& path, std::vector<PathVertex>& out)
{
    // Each point becomes a left/right vertex pair; consecutive pairs form the
    // quads of a GL_TRIANGLE_STRIP ribbon: L0-R0-L1-R1-...
    for (const PathData& p : path)
    {
        const PathVertex left{ -1.0f, p.x, p.y, p.dir_x, p.dir_y, p.thickness, p.depth, p.red, p.green, p.blue, p.alpha };
        PathVertex right = left;
        right.side = 1.0f;
        out.push_back(left);
        out.push_back(right);
    }
    return static_cast<GLsizei>(path.size() * 2);
}

void ShaderPath::draw(const std::vector<PathData>& path)
{
    if (path.size() < 2)
    {
        return;
    }

    std::vector<PathVertex> vertices;
    vertices.reserve(path.size() * 2);
    const GLsizei count = expandPath(path, vertices);

    glUseProgram(shader_program_);
    glBindVertexArray(VAO_path_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_vertex_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(PathVertex)), vertices.data(), GL_DYNAMIC_DRAW);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, count);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);
}

void ShaderPath::draw(const std::vector<std::vector<PathData>>& paths)
{
    if (paths.empty())
    {
        return;
    }

    // Expand every path into one combined vertex buffer, recording each path's
    // (first vertex, vertex count) so it can be drawn as its own strip.
    std::vector<PathVertex> vertices;
    std::vector<std::pair<GLint, GLsizei>> ranges;

    size_t total_points = 0;
    for (const auto& path : paths)
    {
        if (path.size() >= 2)
        {
            total_points += path.size();
        }
    }

    if (total_points == 0)
    {
        return;
    }

    vertices.reserve(total_points * 2);
    ranges.reserve(paths.size());

    for (const auto& path : paths)
    {
        if (path.size() < 2)
        {
            continue;
        }

        const GLint first = static_cast<GLint>(vertices.size());
        const GLsizei count = expandPath(path, vertices);
        ranges.emplace_back(first, count);
    }

    glUseProgram(shader_program_);
    glBindVertexArray(VAO_path_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_vertex_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(PathVertex)), vertices.data(), GL_DYNAMIC_DRAW);

    // Each path is a separate strip, so draw them individually from the shared
    // buffer (no primitive restart needed).
    for (const auto& range : ranges)
    {
        glDrawArrays(GL_TRIANGLE_STRIP, range.first, range.second);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);
}

} // namespace murosim::visualization::shader
