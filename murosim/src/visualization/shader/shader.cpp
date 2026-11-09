// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/shader.hpp"
#include <numbers>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace murosim::visualization::shader
{

Shader::Shader(const std::string &name) : shader_name_(name), shader_program_(0)
{

}

Shader::~Shader()
{
    if (shader_program_ != 0)
    {
        glDeleteProgram(shader_program_);
    }
}

GLuint Shader::getAttribLocation(const char* name)
{
    GLuint current_program;
    glGetIntegerv(GL_CURRENT_PROGRAM, reinterpret_cast<GLint *>(&current_program));
    glUseProgram(shader_program_);

    GLint location = glGetAttribLocation(shader_program_, name);
    if (location == -1) {
        throw std::runtime_error("[Shader::" + shader_name_ + "]: Failed to get the location of " + std::string(name));
    }

    glUseProgram(current_program);

    return static_cast<GLuint>(location);
}

void Shader::setupInstancedFan(GLuint &vao, GLuint &vbo_fan, GLuint &vbo_instance,
                               const size_t resolution, const char* vertex_attribute,
                               const GLsizei instance_stride, const std::vector<InstanceAttribute> &instance_attributes)
{
    if (resolution < 8)
    {
        throw std::runtime_error("[Shader::" + shader_name_ + "]: fan resolution must be at least 8.");
    }

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo_fan);
    glGenBuffers(1, &vbo_instance);

    glBindVertexArray(vao);

    // unit fan geometry: center plus resolution + 1 rim vertices
    std::vector<GLfloat> vertices;
    vertices.reserve((resolution + 2) * 2);
    vertices.push_back(0.0f); // x
    vertices.push_back(0.0f); // y
    for (size_t i = 0; i <= resolution; ++i)
    {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(resolution);
        vertices.push_back(static_cast<GLfloat>(cos(angle))); // x
        vertices.push_back(static_cast<GLfloat>(sin(angle))); // y
    }
    glBindBuffer(GL_ARRAY_BUFFER, vbo_fan);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(GLfloat)), vertices.data(), GL_STATIC_DRAW);

    const GLuint location_vertex = getAttribLocation(vertex_attribute);
    glVertexAttribPointer(location_vertex, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(location_vertex);
    glVertexAttribDivisor(location_vertex, 0);

    bindInstanceAttributes(vbo_instance, instance_stride, instance_attributes);

    // Unbind for safety
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Shader::setupInstancedStrip(GLuint &vao, GLuint &vbo_strip, GLuint &vbo_instance,
                                 const char* vertex_attribute,
                                 const GLsizei instance_stride, const std::vector<InstanceAttribute> &instance_attributes)
{
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo_strip);
    glGenBuffers(1, &vbo_instance);

    glBindVertexArray(vao);

    // Unit quad as a 4-vertex triangle strip: corners at +/-1.
    const GLfloat vertices[] = {
        -1.0f, -1.0f, // bottom-left
         1.0f, -1.0f, // bottom-right
        -1.0f,  1.0f, // top-left
         1.0f,  1.0f, // top-right
    };
    glBindBuffer(GL_ARRAY_BUFFER, vbo_strip);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    const GLuint location_vertex = getAttribLocation(vertex_attribute);
    glVertexAttribPointer(location_vertex, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(location_vertex);
    glVertexAttribDivisor(location_vertex, 0);

    bindInstanceAttributes(vbo_instance, instance_stride, instance_attributes);

    // Unbind for safety
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Shader::bindInstanceAttributes(const GLuint vbo_instance, const GLsizei instance_stride,
                                    const std::vector<InstanceAttribute> &instance_attributes)
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo_instance);

    for (const InstanceAttribute &attribute : instance_attributes)
    {
        const GLuint location = getAttribLocation(attribute.name);
        glVertexAttribPointer(location, attribute.components, GL_FLOAT, GL_FALSE, instance_stride, reinterpret_cast<void*>(static_cast<size_t>(attribute.offset_floats) * sizeof(GLfloat)));
        glEnableVertexAttribArray(location);
        glVertexAttribDivisor(location, 1);
    }
}

void Shader::drawInstancedFan(const GLuint vao, const GLuint vbo_instance, const size_t resolution,
                              const void* instance_data, const size_t instance_bytes, const GLsizei instance_count)
{
    glUseProgram(shader_program_);
    glBindVertexArray(vao);

    // Update the instance VBO with the new data
    glBindBuffer(GL_ARRAY_BUFFER, vbo_instance);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(instance_bytes), instance_data, GL_DYNAMIC_DRAW);

    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(resolution + 2), instance_count);

    // Clean up
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);
}

void Shader::drawInstancedStrip(const GLuint vao, const GLuint vbo_instance,
                                const void* instance_data, const size_t instance_bytes, const GLsizei instance_count)
{
    glUseProgram(shader_program_);
    glBindVertexArray(vao);

    // Update the instance VBO with the new data
    glBindBuffer(GL_ARRAY_BUFFER, vbo_instance);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(instance_bytes), instance_data, GL_DYNAMIC_DRAW);

    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, instance_count);

    // Clean up
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);
}

} // namespace murosim::visualization::shader
