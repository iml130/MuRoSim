// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_SHADER_LINE_HPP_
#define MUROSIM_VISUALIZATION_SHADER_SHADER_LINE_HPP_

#include "murosim/visualization/shader/shader.hpp"
#include <vector>

namespace murosim::visualization::shader
{

/// Draws solid-colored line segments as instanced triangle strips (one draw
/// call for any number of lines), each expanded to a thin quad in the vertex
/// shader from its two endpoints and thickness. Coordinates in LineData are
/// already in normalized device / clip space.
class ShaderLine : public murosim::visualization::shader::Shader
{
private:
    GLuint VAO_line_;
    GLuint VBO_line_;
    GLuint VBO_instance_;
public:
    /// Compiles the line shader program and sets up its instanced strip geometry.
    ShaderLine();
    ~ShaderLine();

    /// Per-line instance data: NDC/clip-space endpoints, thickness, depth, RGBA color.
    struct LineData {
        GLfloat x1;
        GLfloat y1;
        GLfloat x2;
        GLfloat y2;
        GLfloat thickness;
        GLfloat depth;
        GLfloat red;
        GLfloat green;
        GLfloat blue;
        GLfloat alpha;
    };

    /// Draws lines in one instanced call; a no-op for an empty vector.
    void draw(const std::vector<LineData>& lines);
};

} // namespace murosim::visualization::shader

#endif // MUROSIM_VISUALIZATION_SHADER_SHADER_LINE_HPP_
