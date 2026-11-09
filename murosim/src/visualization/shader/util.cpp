// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/util.hpp"

#include <vector>
#include <iostream>

// Holds information about a shader, including its type, source code, and compiled shader ID.
struct ShaderData {
    GLenum      type;    // Shader type (e.g., GL_VERTEX_SHADER)
    const char* source;  // Pointer to shader source code
    GLuint      id;      // Compiled shader object ID
};

static GLuint compileShader(GLenum type, const char* source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        // Build the message first, then delete the shader before throwing so the
        // GL object is not leaked on the error path.
        GLint log_length = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
        std::string message = "[compileShader]:Error compiling shader: ";
        if (log_length > 1) {
            std::vector<char> log(static_cast<size_t>(log_length));
            glGetShaderInfoLog(shader, log_length, nullptr, log.data());
            message += log.data();
        } else {
            message += "(no log available)";
        }

        glDeleteShader(shader);
        throw std::runtime_error(message);
    }

    return shader;
}

inline static GLuint compileShaderProgramFromShaderData(std::vector<ShaderData>& shaders)
{
    // compileShader throws on failure, so reaching the link step means every
    // shader compiled successfully.
    for (ShaderData& shader : shaders) {
        shader.id = compileShader(shader.type, shader.source);
    }

    GLuint program = glCreateProgram();
    for (const ShaderData& shader : shaders)
    {
        glAttachShader(program, shader.id);
    }

    glLinkProgram(program);

    GLint linked = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);

    if (!linked)
    {
        GLint log_length = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_length);
        std::string message = "[linkProgram]: Error linking program: ";
        if (log_length > 1) {
            std::vector<char> log(static_cast<size_t>(log_length));
            glGetProgramInfoLog(program, log_length, nullptr, log.data());
            message += log.data();
        } else {
            message += "(no log available)";
        }

        // Delete the GL objects before throwing so they are not leaked.
        for (const ShaderData& shader : shaders)
        {
            glDeleteShader(shader.id);
        }
        glDeleteProgram(program);
        throw std::runtime_error(message);
    }

    for (const ShaderData& shader : shaders)
    {
        glDetachShader(program, shader.id);
        glDeleteShader(shader.id);
    }

    return program;
}


namespace murosim::visualization::shader::util
{

GLuint compileShaderProgram(const char* vertex_source, const char* fragment_source)
{
    std::vector<ShaderData> shaders = {
        { GL_VERTEX_SHADER,   vertex_source,   0 },
        { GL_FRAGMENT_SHADER, fragment_source, 0 }
    };

    return compileShaderProgramFromShaderData(shaders);
}

} // namespace murosim::visualization::shader::util
