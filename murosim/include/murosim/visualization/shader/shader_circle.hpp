// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_SHADER_CIRCLE_HPP_
#define MUROSIM_VISUALIZATION_SHADER_SHADER_CIRCLE_HPP_

#include "murosim/visualization/shader/shader.hpp"
#include <vector>

namespace murosim::visualization::shader
{

/// Draws filled, solid-colored circles as instanced triangle fans (one draw
/// call for any number of circles). All coordinates in CircleData are
/// already in normalized device / clip space, as produced by Canvas's
/// world-to-NDC transform; the vertex shader just scales the unit fan by
/// radius and translates by (x, y).
class ShaderCircle : public murosim::visualization::shader::Shader
{
private:
    GLuint VAO_circle_;
    GLuint VBO_circle_;
    GLuint VBO_instance_;
public:
    /// Compiles the circle shader program and sets up its instanced fan geometry.
    ShaderCircle();
    ~ShaderCircle();

    /// Per-circle instance data (NDC/clip-space center and radius, depth, RGBA color).
    struct CircleData {
        GLfloat x;
        GLfloat y;
        GLfloat radius;
        GLfloat depth;
        GLfloat red;
        GLfloat green;
        GLfloat blue;
        GLfloat alpha;
    };

    /// Draws circles in one instanced call; a no-op for an empty vector.
    void draw(const std::vector<CircleData> &circles);
};

} // namespace murosim::visualization::shader

#endif // MUROSIM_VISUALIZATION_SHADER_SHADER_CIRCLE_HPP_
