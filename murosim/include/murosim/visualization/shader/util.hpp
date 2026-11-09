// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_UTIL_HPP_
#define MUROSIM_VISUALIZATION_SHADER_UTIL_HPP_


#include <GLES3/gl3.h>
#include <iostream>
#include <string>

/// Drains the GL error queue (glGetError) and logs each pending error, with
/// its symbolic name and the call site (__FILE__:__LINE__), to stderr. A
/// debugging aid only: it does not throw or otherwise affect control flow.
/// Place after suspect GL calls; has no effect if there is no pending error.
#define MUROSIM_GL_CHECK_ERROR() \
    do { \
        GLenum err; \
        while ((err = glGetError()) != GL_NO_ERROR) { \
            std::string error_str; \
            switch (err) { \
                case GL_INVALID_ENUM: error_str = "GL_INVALID_ENUM"; break; \
                case GL_INVALID_VALUE: error_str = "GL_INVALID_VALUE"; break; \
                case GL_INVALID_OPERATION: error_str = "GL_INVALID_OPERATION"; break; \
                case GL_OUT_OF_MEMORY: error_str = "GL_OUT_OF_MEMORY"; break; \
                case GL_INVALID_FRAMEBUFFER_OPERATION: error_str = "GL_INVALID_FRAMEBUFFER_OPERATION"; break; \
                default: error_str = "UNKNOWN_ERROR (" + std::to_string(err) + ")"; break; \
            } \
            std::cerr << "OpenGL Error: " << error_str \
                      << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
        } \
    } while (0)

namespace murosim::visualization::shader::util
{

/// Compiles a vertex and fragment shader from GLSL source, links them into a
/// program, and cleans up the intermediate shader objects. Requires a
/// current GL context.
/// @return the linked program's GL handle; ownership passes to the caller
///         (glDeleteProgram when done, as Shader's destructor does).
/// @throws std::runtime_error with the compiler/linker log if compilation or
///         linking fails; any GL objects created before the failure are
///         deleted before throwing.
GLuint compileShaderProgram(const char* vertex_shader_source, const char* fragment_shader_source);

} // namespace murosim::visualization::shader::util

#endif // MUROSIM_VISUALIZATION_SHADER_UTIL_HPP_
