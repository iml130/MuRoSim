// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SHADER_SHADER_PATH_HPP_
#define MUROSIM_VISUALIZATION_SHADER_SHADER_PATH_HPP_

#include "murosim/visualization/shader/shader.hpp"
#include <vector>

namespace murosim::visualization::shader
{

/// Draws one or more polylines as solid-colored ribbons (variable thickness,
/// per-point color) rendered as connected triangle strips. Unlike the other
/// shaders here, path vertices are expanded on the CPU (expandPath()) into
/// explicit left/right ribbon vertices rather than drawn via GL instancing,
/// since each path needs a variable-length strip rather than a fixed
/// per-instance shape. Coordinates in PathData are already in normalized
/// device / clip space.
class ShaderPath : public murosim::visualization::shader::Shader
{
public:
    /// Per path-point data (10 floats = 40 bytes).
    /// Each point is expanded into 2 ribbon vertices (left/right) on upload.
    struct PathData
    {
        GLfloat x;          // Position x
        GLfloat y;          // Position y
        GLfloat dir_x;      // Tangent direction x (normalized)
        GLfloat dir_y;      // Tangent direction y (normalized)
        GLfloat thickness;  // Line thickness
        GLfloat depth;      // Z depth
        GLfloat red;        // Color
        GLfloat green;
        GLfloat blue;
        GLfloat alpha;
    };

    /// Compiles the path shader program and sets up its VAO/vertex buffer.
    ShaderPath();
    ~ShaderPath();

    /// Draw a single path as a connected triangle strip; a no-op for fewer
    /// than 2 points.
    void draw(const std::vector<PathData> &path);

    /// Draw multiple paths, each as its own triangle strip, from one upload;
    /// a no-op for an empty vector. Paths with fewer than 2 points are
    /// skipped individually.
    void draw(const std::vector<std::vector<PathData>> &paths);

private:
    /// One vertex of the ribbon: a path point offset to the left/right side.
    struct PathVertex
    {
        GLfloat side;       // -1 = left, +1 = right
        GLfloat x;
        GLfloat y;
        GLfloat dir_x;
        GLfloat dir_y;
        GLfloat thickness;
        GLfloat depth;
        GLfloat red;
        GLfloat green;
        GLfloat blue;
        GLfloat alpha;
    };

    GLuint VAO_path_;
    GLuint VBO_vertex_;     // Expanded ribbon vertices (2 per path point)

    /// Expand a path's points into the left/right vertex pairs of a triangle
    /// strip, appending them to out. Returns the number of vertices appended.
    static GLsizei expandPath(const std::vector<PathData> &path, std::vector<PathVertex> &out);
};

} // namespace murosim::visualization::shader

#endif // MUROSIM_VISUALIZATION_SHADER_SHADER_PATH_HPP_
