// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_SHADER_RING_HPP_
#define MUROSIM_VISUALIZATION_SHADER_SHADER_RING_HPP_

#include "murosim/visualization/shader/shader.hpp"
#include <vector>

namespace murosim::visualization::shader
{

/// Draws unfilled, solid-colored circle outlines ("rings") as instanced
/// triangle fans: the fragment shader discards fragments outside the
/// [radius - thickness/2, radius + thickness/2] band, so the visible outer
/// edge sits at radius + thickness/2. Coordinates in RingData are already in
/// normalized device / clip space.
class ShaderRing : public murosim::visualization::shader::Shader
{
private:
    GLuint VAO_ring_;
    GLuint VBO_ring_;
    GLuint VBO_instance_;
public:
    /// Compiles the ring shader program and sets up its instanced fan geometry.
    ShaderRing();
    ~ShaderRing();

    /// Per-ring instance data: NDC/clip-space center, ring (center-line)
    /// radius, band thickness, depth, RGBA color.
    struct RingData {
        GLfloat x;
        GLfloat y;
        GLfloat radius;
        GLfloat thickness;
        GLfloat depth;
        GLfloat red;
        GLfloat green;
        GLfloat blue;
        GLfloat alpha;
    };

    /// Draws rings in one instanced call; a no-op for an empty vector.
    void draw(const std::vector<RingData> &rings);
};

} // namespace murosim::visualization::shader

#endif // MUROSIM_VISUALIZATION_SHADER_SHADER_RING_HPP_
