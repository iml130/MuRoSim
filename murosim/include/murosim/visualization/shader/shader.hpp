// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_SHADER_HPP_
#define MUROSIM_VISUALIZATION_SHADER_SHADER_HPP_

#include <GLES3/gl3.h>
#include <string>
#include <vector>

namespace murosim::visualization::shader
{

/// Common base for the murosim OpenGL ES shaders (ShaderCircle, ShaderLine,
/// ShaderQuad, ShaderRing, ShaderPath). Compiles and owns one shader program,
/// and factors out the two instanced-rendering setups they share: an
/// instanced triangle fan (used for filled circles/rings) and an instanced
/// triangle strip over a unit quad (used for lines/quads/rings). In both
/// cases per-primitive data (position, size, color, ...) is uploaded once per
/// draw call as a per-instance vertex attribute (divisor 1) sourced from a
/// shared instance VBO, so an arbitrary number of primitives is drawn with a
/// single glDrawArraysInstanced call. Requires the owning renderer's GL
/// context to be current for construction, destruction and every draw call.
/// Not copyable or movable: derived shaders own raw GL handles (program,
/// VAO/VBO) released in their destructors, so copying would double-delete.
class Shader
{
protected:
    std::string shader_name_;
    GLuint shader_program_;

    /// One per-instance vertex attribute inside an interleaved float struct.
    struct InstanceAttribute
    {
        const char* name;
        GLint components;
        GLsizei offset_floats;
    };

    /// Looks up the attribute location of name in this shader's program.
    /// @throws std::runtime_error if the attribute is not found.
    GLuint getAttribLocation(const char* name);

    /// Create a unit triangle-fan VBO (center + resolution + 1 rim vertices)
    /// and a VAO binding it to vertex_attribute plus per-instance attributes
    /// sourced from a shared instance VBO.
    /// @param resolution number of rim segments; must be at least 8.
    /// @throws std::runtime_error if resolution < 8.
    void setupInstancedFan(GLuint &vao, GLuint &vbo_fan, GLuint &vbo_instance,
                           const size_t resolution, const char* vertex_attribute,
                           const GLsizei instance_stride, const std::vector<InstanceAttribute> &instance_attributes);

    /// Upload instance data and draw the fan instanced.
    void drawInstancedFan(const GLuint vao, const GLuint vbo_instance, const size_t resolution,
                          const void* instance_data, const size_t instance_bytes, const GLsizei instance_count);

    /// Create a unit-quad VBO (4-vertex triangle strip, corners at +/-1)
    /// and a VAO binding it to vertex_attribute plus per-instance attributes
    /// sourced from a shared instance VBO.
    void setupInstancedStrip(GLuint &vao, GLuint &vbo_strip, GLuint &vbo_instance,
                             const char* vertex_attribute,
                             const GLsizei instance_stride, const std::vector<InstanceAttribute> &instance_attributes);

    /// Upload instance data and draw the unit-quad strip instanced.
    void drawInstancedStrip(const GLuint vao, const GLuint vbo_instance,
                            const void* instance_data, const size_t instance_bytes, const GLsizei instance_count);

private:
    /// Bind the per-instance (divisor 1) attributes from a shared instance VBO.
    void bindInstanceAttributes(const GLuint vbo_instance, const GLsizei instance_stride,
                                const std::vector<InstanceAttribute> &instance_attributes);

protected:

    /// Constructs an unlinked shader; derived constructors set shader_program_
    /// via murosim::visualization::shader::util::compileShaderProgram.
    /// @param name used only in error messages (e.g. "ShaderCircle").
    Shader(const std::string &name);
    /// Deletes shader_program_ if it was set. The GL context under which the
    /// program was created must still be current.
    ~Shader();

    // Shaders own raw GL handles and delete them in their destructors; copying
    // would double-delete. Non-copyable/non-movable (inherited by all shaders).
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
};

} // namespace murosim::visualization::shader

#endif // MUROSIM_VISUALIZATION_SHADER_SHADER_HPP_
