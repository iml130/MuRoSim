// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_CANVAS_HPP_
#define MUROSIM_VISUALIZATION_CANVAS_HPP_

#include "murosim/visualization/color.hpp"
#include "murosim/geometry/shape.hpp"
#include "murosim/grid/map.hpp"
#include "murosim/visualization/shader/gl_shader_manager.hpp"
#include <memory>



namespace murosim::visualization
{

/// Per-frame drawing surface backed by a GlShaderManager. Two usage modes:
/// the addXxx() methods batch shapes into internal buffers with the current
/// transform/thickness baked in, later flushed together by drawAll() (one
/// draw call per shader, used by Scenery so a whole world/simulation is
/// issued efficiently); the drawXxx() methods render a single item
/// immediately without batching, for one-off use. Both apply the transform
/// set by setDrawArea() (world coordinates -> normalized device coordinates)
/// and the thickness scale set by setDrawThickness(). Requires an OpenGL
/// context current on the calling thread (see Renderer); not thread-safe.
class Canvas
{
private:
    std::shared_ptr<murosim::visualization::shader::GlShaderManager> shader_manager_;
    std::vector<murosim::visualization::shader::ShaderCircle::CircleData> data_circles_;
    std::vector<murosim::visualization::shader::ShaderLine::LineData> data_lines_;
    std::vector<murosim::visualization::shader::ShaderQuad::QuadData> data_quads_;
    std::vector<murosim::visualization::shader::ShaderRing::RingData> data_rings_;
    std::vector<std::vector<murosim::visualization::shader::ShaderPath::PathData>> data_paths_;
    Eigen::Affine2f transform_;
    float thickness_;

    /// Rasterize the grid into one quad per cell whose FIELD has a
    /// color (nullptr = skip that FIELD); shared by every addGridMap/drawGridMap
    /// overload.
    void appendGridMapQuads(const murosim::grid::Map &grid_map, const murosim::visualization::Color *color_free, const murosim::visualization::Color *color_occupied, const murosim::visualization::Color *color_buffer, const murosim::visualization::Color *color_unreachable, const float depth, std::vector<murosim::visualization::shader::ShaderQuad::QuadData> &quads) const;

    /// Tessellate a path (tangents + interpolated color) into renderable
    /// segments; shared by the addPath/drawPath overloads. Returns an empty
    /// vector for degenerate input (max_length == 0 or fewer than 2 points).
    std::vector<murosim::visualization::shader::ShaderPath::PathData> buildPathData(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color_start, const murosim::visualization::Color &color_end, const float thickness, const float depth, const size_t max_length) const;
public:
    /// @param shader_manager must be non-null; throws std::runtime_error otherwise.
    Canvas(std::shared_ptr<murosim::visualization::shader::GlShaderManager> shader_manager);

    /// Queues a circle for the next drawAll(), in the current transform.
    void addCircle(const murosim::geometry::Circle &circle, const murosim::visualization::Color &color, const float depth);
    /// @copydoc addCircle(const murosim::geometry::Circle&, const murosim::visualization::Color&, const float)
    void addCircle(const std::vector<murosim::geometry::Circle> &circles, const murosim::visualization::Color &color, const float depth);
    /// Queues a line segment for the next drawAll(); thickness is scaled by
    /// the canvas's current draw thickness (see setDrawThickness()).
    void addLine(const murosim::geometry::Line &line, const murosim::visualization::Color &color, const float thickness, const float depth);
    /// @copydoc addLine(const murosim::geometry::Line&, const murosim::visualization::Color&, const float, const float)
    void addLine(const std::vector<murosim::geometry::Line> &lines, const murosim::visualization::Color &color, const float thickness, const float depth);
    /// Queues a path with a single color, tessellated into up to path.size()
    /// segments.
    void addPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color, const float thickness, const float depth);
    /// Queues a path with per-segment color interpolated linearly from
    /// color_start to color_end, tessellated into at most max_length points.
    /// No-op for max_length == 0 or paths shorter than 2 points.
    void addPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color_start, const murosim::visualization::Color &color_end, const float thickness, const float depth, const size_t max_length);
    /// Queues every line and circle of shape at the identity pose.
    /// @param draw_ring_instead_of_circle if true, circles are queued as
    ///        unfilled rings (outline only) instead of filled circles.
    void addShape(const murosim::geometry::Shape &shape, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle = false);
    /// @copydoc addShape(const murosim::geometry::Shape&, const murosim::visualization::Color&, const float, const float, const bool)
    /// @param pose additional transform applied to the shape before the canvas's own transform.
    void addShape(const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle = false);
    /// Queues one quad per occupancy-grid cell that is FREE or OCCUPIED;
    /// BUFFER cells reuse color_free.
    void addGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const float depth);
    /// @copydoc addGridMap(const murosim::grid::Map&, const murosim::visualization::Color&, const murosim::visualization::Color&, const float)
    /// BUFFER cells use color_buffer; UNREACHABLE cells are skipped.
    void addGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const float depth);
    /// @copydoc addGridMap(const murosim::grid::Map&, const murosim::visualization::Color&, const murosim::visualization::Color&, const murosim::visualization::Color&, const float)
    /// UNREACHABLE cells use color_unreachable instead of being skipped.
    void addGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const murosim::visualization::Color &color_unreachable, const float depth);
    /// Queues an unfilled circle outline (ring).
    void addRing(const murosim::geometry::Circle &ring, const murosim::visualization::Color &color, const float thickness, const float depth);
    /// @copydoc addRing(const murosim::geometry::Circle&, const murosim::visualization::Color&, const float, const float)
    void addRing(const std::vector<murosim::geometry::Circle> &ring, const murosim::visualization::Color &color, const float thickness, const float depth);
    /// Draws a circle immediately (not batched).
    void drawCircle(const murosim::geometry::Circle &circle, const murosim::visualization::Color &color, const float depth) const;
    /// @copydoc drawCircle(const murosim::geometry::Circle&, const murosim::visualization::Color&, const float) const
    void drawCircle(const std::vector<murosim::geometry::Circle> &circles, const murosim::visualization::Color &color, const float depth) const;
    /// Draws a line immediately (not batched).
    void drawLine(const murosim::geometry::Line &line, const murosim::visualization::Color &color, const float thickness, const float depth) const;
    /// @copydoc drawLine(const murosim::geometry::Line&, const murosim::visualization::Color&, const float, const float) const
    void drawLine(const std::vector<murosim::geometry::Line> &lines, const murosim::visualization::Color &color, const float thickness, const float depth) const;
    /// Draws a single-colored path immediately (not batched).
    void drawPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color, const float thickness, const float depth) const;
    /// Draws a path with interpolated color immediately (not batched); see
    /// the batched overload for the color/tessellation semantics.
    void drawPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color_start, const murosim::visualization::Color &color_end, const float thickness, const float depth, const size_t max_length) const;
    /// Draws a shape immediately (not batched).
    void drawShape(const murosim::geometry::Shape &shape, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle = false) const;
    /// @copydoc drawShape(const murosim::geometry::Shape&, const murosim::visualization::Color&, const float, const float, const bool) const
    void drawShape(const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle = false) const;
    /// Draws an occupancy grid immediately (not batched).
    void drawGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const float depth) const;
    /// @copydoc drawGridMap(const murosim::grid::Map&, const murosim::visualization::Color&, const murosim::visualization::Color&, const float) const
    void drawGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const float depth) const;
    /// @copydoc drawGridMap(const murosim::grid::Map&, const murosim::visualization::Color&, const murosim::visualization::Color&, const murosim::visualization::Color&, const float) const
    void drawGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const murosim::visualization::Color &color_unreachable, const float depth) const;
    /// Flushes every batched primitive queued via addXxx() to the GL shaders
    /// (one draw call per shape type) and clears the internal buffers.
    void drawAll();
    /// Clears the framebuffer to color (see GlShaderManager::drawAndClearBackground).
    void drawBackground(const murosim::visualization::Color &color) const;
    /// Sets the world-to-NDC transform so [area_min, area_max] is centered
    /// and uniformly scaled to fill the [-1,1] NDC square (aspect-preserving,
    /// using the larger extent).
    void setDrawArea(const Eigen::Vector2f area_min, const Eigen::Vector2f area_max);
    /// @copydoc setDrawArea(const Eigen::Vector2f, const Eigen::Vector2f)
    /// @param margin fraction of the NDC square left empty around the content, in [0, 0.5).
    void setDrawArea(const Eigen::Vector2f area_min, const Eigen::Vector2f area_max, const float margin);
    /// Sets the world-to-NDC transform so [area_min, area_max] is scaled
    /// (aspect-preserving, using the smaller resulting scale) to fit inside
    /// an ndc_size NDC region centered at ndc_origin + ndc_size/2, instead of
    /// filling the whole [-1,1] square. Used to lay out multiple worlds in a
    /// grid within one frame (see Scenery::drawSceneryDataContainers/
    /// drawScenerySimulations).
    void setDrawArea(const Eigen::Vector2f area_min, const Eigen::Vector2f area_max, const Eigen::Vector2f ndc_origin, const Eigen::Vector2f ndc_size);
    /// Sets the base line/ring thickness multiplier applied to subsequent
    /// addLine/addRing/addShape/addPath/drawXxx calls.
    void setDrawThickness(const float thickness);
    /// Derives the base thickness from the output resolution (width, height
    /// in pixels), so line thickness scales with image size; clamped to a
    /// minimum.
    void setDrawThickness(const GLint width, const GLint height);
    /// Current base thickness multiplier set by setDrawThickness().
    float getDrawThickness() const;
};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_CANVAS_HPP_
