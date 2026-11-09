// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_SHADER_QUAD_HPP_
#define MUROSIM_VISUALIZATION_SHADER_SHADER_QUAD_HPP_

#include "murosim/visualization/shader/shader.hpp"
#include <vector>

namespace murosim::visualization::shader
{

/// Draws axis-aligned, solid-colored quads as instanced triangle strips (one
/// draw call for any number of quads); used by Canvas to rasterize occupancy
/// grid cells. Coordinates in QuadData are already in normalized device /
/// clip space.
class ShaderQuad : public murosim::visualization::shader::Shader
{
private:
    GLuint VAO_quad_;
    GLuint VBO_quad_;
    GLuint VBO_instance_;
public:
    /// Compiles the quad shader program and sets up its instanced strip geometry.
    ShaderQuad();
    ~ShaderQuad();

    /// Per-quad instance data: NDC/clip-space center and full (width, height) size, depth, RGBA color.
    struct QuadData {
        GLfloat x;
        GLfloat y;
        GLfloat size_x;
        GLfloat size_y;
        GLfloat depth;
        GLfloat red;
        GLfloat green;
        GLfloat blue;
        GLfloat alpha;
    };

    /// Draws quads in one instanced call; a no-op for an empty vector.
    void draw(const std::vector<QuadData>& quads);
};

} // namespace murosim::visualization::shader

#endif // MUROSIM_VISUALIZATION_SHADER_SHADER_QUAD_HPP_
