// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/shader/shader_ring.hpp"
#include "murosim/visualization/shader/util.hpp"
#include <Eigen/Core>
#include <vector>
#include <cmath>

constexpr size_t RING_RESOLUTION = 36;

namespace
{

const char *VERTEX_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision highp float;

    layout(location = 0) in vec2 a_RingInstance; // Vertex for a single ring (unit ring)
    layout(location = 1) in vec4 a_RingData;     // Instance data: x, y, radius, thickness for each ring
    layout(location = 2) in float a_RingDepth;   // Instance data: depth for each ring
    layout(location = 3) in vec4 a_RingColor;    // Color of the ring

    out vec4 v_Color;      // Color to be passed to the fragment shader
    out vec2  v_LocalPos;  // unit-ring coords passed to FS
    out float v_Thickness; // thickness as fraction of radius

    void main()
    {
        float radius = a_RingData.z + a_RingData.w * 0.5f;
        vec2 position = a_RingData.xy + radius * a_RingInstance;
        gl_Position = vec4(position, a_RingDepth, 1.0);

        v_LocalPos   = a_RingInstance;
        // Guard against a zero-radius ring producing a NaN thickness fraction.
        v_Thickness  = radius > 0.0 ? a_RingData.w / radius : 0.0;
        v_Color = a_RingColor;
    }
)glsl";

const char *FRAGMENT_SHADER_SOURCE = R"glsl(
    #version 300 es
    precision mediump float;

    in vec4 v_Color;       // Color of the ring
    in vec2 v_LocalPos;    // unit-ring coordinates
    in float v_Thickness;  // 0..1
    out vec4 o_FragColor;

    void main() {
        float dist = length(v_LocalPos);

        // Keep only the ring:  (1 - t) <= dist <= 1
        if (dist < 1.0 - v_Thickness || dist > 1.0)
            discard;

        o_FragColor = v_Color;
    }
)glsl";

}

namespace murosim::visualization::shader
{

ShaderRing::ShaderRing(): Shader("ShaderRing")
{
    shader_program_ = murosim::visualization::shader::util::compileShaderProgram(VERTEX_SHADER_SOURCE, FRAGMENT_SHADER_SOURCE);

    // RingData struct layout: x, y, radius, thickness, depth, r, g, b, a (total 9 floats)
    setupInstancedFan(VAO_ring_, VBO_ring_, VBO_instance_, RING_RESOLUTION, "a_RingInstance", sizeof(RingData),
                      {{"a_RingData", 4, 0}, {"a_RingDepth", 1, 4}, {"a_RingColor", 4, 5}});
}

ShaderRing::~ShaderRing()
{
    glDeleteVertexArrays(1, &VAO_ring_);
    glDeleteBuffers(1, &VBO_ring_);
    glDeleteBuffers(1, &VBO_instance_);
}

void ShaderRing::draw(const std::vector<RingData> &rings)
{
    if (rings.empty()) return;

    drawInstancedFan(VAO_ring_, VBO_instance_, RING_RESOLUTION,
                     rings.data(), rings.size() * sizeof(RingData), static_cast<GLsizei>(rings.size()));
}

} // namespace murosim::visualization::shader
