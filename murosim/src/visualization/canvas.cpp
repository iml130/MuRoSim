// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/canvas.hpp"
#include <vector>

// constexpr float DEFAULT_MARGIN = 0.05f;
constexpr float DEFAULT_THICKNESS_RATIO = 0.000001f;
constexpr float DEFAULT_THICKNESS_MIN = 0.004f;
constexpr float GRID_MAP_QUAD_SIZE_FACTOR = 1.005f; // to fill the gaps between the quads

namespace murosim::visualization
{

Canvas::Canvas(std::shared_ptr<murosim::visualization::shader::GlShaderManager> shader_manager)
    : shader_manager_(std::move(shader_manager)), transform_(Eigen::Affine2f::Identity()), thickness_(DEFAULT_THICKNESS_MIN)
{
    assert(shader_manager_ != nullptr);

    if(shader_manager_ == nullptr)
    {
        throw std::runtime_error("[Canvas]: Shader manager is nullptr!");
    }
}

void Canvas::addCircle(const murosim::geometry::Circle &circle, const murosim::visualization::Color &color, const float depth)
{
    const murosim::geometry::Circle transformed_circle = transform_ * circle;
    data_circles_.emplace_back(murosim::visualization::shader::ShaderCircle::CircleData{transformed_circle.getX(), transformed_circle.getY(), transformed_circle.getRadius(), depth, color.r(), color.g(), color.b(), color.a()});
}

void Canvas::addCircle(const std::vector<murosim::geometry::Circle> &circles, const murosim::visualization::Color &color, const float depth)
{
    for (const auto &circle : circles)
    {
        const murosim::geometry::Circle transformed_circle = transform_ * circle;
        data_circles_.emplace_back(murosim::visualization::shader::ShaderCircle::CircleData{transformed_circle.getX(), transformed_circle.getY(), transformed_circle.getRadius(), depth, color.r(), color.g(), color.b(), color.a()});
    }
}

void Canvas::addLine(const murosim::geometry::Line &line, const murosim::visualization::Color &color, const float thickness, const float depth)
{
    const murosim::geometry::Line transformed_line = transform_ * line;
    data_lines_.emplace_back(murosim::visualization::shader::ShaderLine::LineData{transformed_line.getPoint1X(), transformed_line.getPoint1Y(), transformed_line.getPoint2X(), transformed_line.getPoint2Y(), thickness_ * thickness, depth, color.r(), color.g(), color.b(), color.a()});
}

void Canvas::addLine(const std::vector<murosim::geometry::Line> &lines, const murosim::visualization::Color &color, const float thickness, const float depth)
{
    for (const auto &line : lines)
    {
        const murosim::geometry::Line transformed_line = transform_ * line;
        data_lines_.emplace_back(murosim::visualization::shader::ShaderLine::LineData{transformed_line.getPoint1X(), transformed_line.getPoint1Y(), transformed_line.getPoint2X(), transformed_line.getPoint2Y(), thickness_ * thickness, depth, color.r(), color.g(), color.b(), color.a()});
    }
}

void Canvas::addShape(const murosim::geometry::Shape &shape, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle)
{
    addShape(shape, Eigen::Affine2f::Identity(), color, thickness, depth, draw_ring_instead_of_circle);
}

void Canvas::addShape(const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle)
{
    const Eigen::Affine2f transform = transform_ * pose;
    for (const auto& shape_line : shape.getLines())
    {
        const murosim::geometry::Line line = transform * shape_line;
        data_lines_.emplace_back(murosim::visualization::shader::ShaderLine::LineData{line.getPoint1X(), line.getPoint1Y(), line.getPoint2X(), line.getPoint2Y(), thickness_ * thickness, depth, color.r(), color.g(), color.b(), color.a()});
    }

    if (draw_ring_instead_of_circle)
    {
        for (const auto& shape_circle : shape.getCircles())
        {
            const murosim::geometry::Circle circle = transform * shape_circle;
            data_rings_.emplace_back(murosim::visualization::shader::ShaderRing::RingData{circle.getX(), circle.getY(), circle.getRadius(), thickness_ * thickness, depth, color.r(), color.g(), color.b(), color.a()});
        }
    }
    else
    {
        for (const auto& shape_circle : shape.getCircles())
        {
            const murosim::geometry::Circle circle = transform * shape_circle;
            data_circles_.emplace_back(murosim::visualization::shader::ShaderCircle::CircleData{circle.getX(), circle.getY(), circle.getRadius(), depth, color.r(), color.g(), color.b(), color.a()});
        }
    }
}

void Canvas::appendGridMapQuads(const murosim::grid::Map &grid_map, const murosim::visualization::Color *color_free, const murosim::visualization::Color *color_occupied, const murosim::visualization::Color *color_buffer, const murosim::visualization::Color *color_unreachable, const float depth, std::vector<murosim::visualization::shader::ShaderQuad::QuadData> &quads) const
{
    const float quad_size = (transform_.linear() * Eigen::Vector2f(grid_map.getGridScale(), 0.0f)).norm() * GRID_MAP_QUAD_SIZE_FACTOR;
    Eigen::Vector2f point;

    for (size_t x = 0; x < grid_map.sizeX(); x++)
    {
        for (size_t y = 0; y < grid_map.sizeY(); y++)
        {
            const murosim::visualization::Color *color = nullptr;
            switch (grid_map(x, y))
            {
            case murosim::grid::Map::FIELD::FREE:
                color = color_free;
                break;

            case murosim::grid::Map::FIELD::OCCUPIED:
                color = color_occupied;
                break;

            case murosim::grid::Map::FIELD::BUFFER:
                color = color_buffer;
                break;

            case murosim::grid::Map::FIELD::UNREACHABLE:
                color = color_unreachable;
                break;

            default:
                break;
            }

            if (color != nullptr)
            {
                grid_map.fromMapIntoWorld(static_cast<int>(x), static_cast<int>(y), point);
                point = transform_ * point;
                quads.emplace_back(murosim::visualization::shader::ShaderQuad::QuadData{point.x(), point.y(), quad_size, quad_size, depth, color->r(), color->g(), color->b(), color->a()});
            }
        }
    }
}

void Canvas::addGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const float depth)
{
    appendGridMapQuads(grid_map, &color_free, &color_occupied, &color_free, nullptr, depth, data_quads_);
}

void Canvas::addGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const float depth)
{
    appendGridMapQuads(grid_map, &color_free, &color_occupied, &color_buffer, nullptr, depth, data_quads_);
}

void Canvas::addGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const murosim::visualization::Color &color_unreachable, const float depth)
{
    appendGridMapQuads(grid_map, &color_free, &color_occupied, &color_buffer, &color_unreachable, depth, data_quads_);
}

void Canvas::addRing(const murosim::geometry::Circle &ring, const murosim::visualization::Color &color, const float thickness, const float depth)
{
    const murosim::geometry::Circle transformed_ring = transform_ * ring;
    data_rings_.emplace_back(murosim::visualization::shader::ShaderRing::RingData{transformed_ring.getX(), transformed_ring.getY(), transformed_ring.getRadius(), thickness_ * thickness, depth, color.r(), color.g(), color.b(), color.a()});
}

void Canvas::addRing(const std::vector<murosim::geometry::Circle> &ring, const murosim::visualization::Color &color, const float thickness, const float depth)
{
    for (const auto &circle : ring)
    {
        const murosim::geometry::Circle transformed_ring = transform_ * circle;
        data_rings_.emplace_back(murosim::visualization::shader::ShaderRing::RingData{transformed_ring.getX(), transformed_ring.getY(), transformed_ring.getRadius(), thickness_ * thickness, depth, color.r(), color.g(), color.b(), color.a()});
    }
}

void Canvas::addPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color, const float thickness, const float depth)
{
    addPath(path, color, color, thickness, depth, path.size());
}

void Canvas::addPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color_start, const murosim::visualization::Color &color_end, const float thickness, const float depth, const size_t max_length)
{
    std::vector<murosim::visualization::shader::ShaderPath::PathData> path_data = buildPathData(path, color_start, color_end, thickness, depth, max_length);
    if (!path_data.empty())
    {
        data_paths_.push_back(std::move(path_data));
    }
}

std::vector<murosim::visualization::shader::ShaderPath::PathData> Canvas::buildPathData(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color_start, const murosim::visualization::Color &color_end, const float thickness, const float depth, const size_t max_length) const
{
    std::vector<murosim::visualization::shader::ShaderPath::PathData> path_data;

    if (max_length == 0 || path.size() < 2)
    {
        return path_data;
    }

    const size_t size = std::min(path.size(), max_length);
    path_data.reserve(size);

    const float line_thickness = thickness_ * thickness;
    const float color_steps = static_cast<float>(size - 1);

    for (size_t i = 0; i < size; ++i)
    {
        const Eigen::Vector2f pos = transform_ * path[i];

        // Compute tangent direction from neighboring points
        const Eigen::Vector2f prev = transform_ * path[(i == 0) ? 0 : i - 1];
        const Eigen::Vector2f next = transform_ * path[(i >= size - 1) ? size - 1 : i + 1];

        Eigen::Vector2f dir = next - prev;
        const float len = dir.norm();
        if (len > 0.0f)
        {
            dir /= len;
        }
        else
        {
            dir = Eigen::Vector2f(1.0f, 0.0f);
        }

        // Interpolate color
        const float t = static_cast<float>(i) / color_steps;
        const GLfloat r = color_start.r() + t * (color_end.r() - color_start.r());
        const GLfloat g = color_start.g() + t * (color_end.g() - color_start.g());
        const GLfloat b = color_start.b() + t * (color_end.b() - color_start.b());
        const GLfloat a = color_start.a() + t * (color_end.a() - color_start.a());

        path_data.emplace_back(murosim::visualization::shader::ShaderPath::PathData{
            pos.x(), pos.y(), dir.x(), dir.y(), line_thickness, depth, r, g, b, a
        });
    }

    return path_data;
}

void Canvas::drawCircle(const murosim::geometry::Circle &circle, const murosim::visualization::Color &color, const float depth) const
{
    const murosim::geometry::Circle transformed_circle = transform_ * circle;
    shader_manager_->drawCircle({murosim::visualization::shader::ShaderCircle::CircleData{transformed_circle.getX(), transformed_circle.getY(), transformed_circle.getRadius(), depth, color.r(), color.g(), color.b(), color.a()}});
}

void Canvas::drawCircle(const std::vector<murosim::geometry::Circle> &circles, const murosim::visualization::Color &color, const float depth) const
{
    std::vector<murosim::visualization::shader::ShaderCircle::CircleData> circle_data(circles.size());
    for (size_t i = 0; i < circles.size(); ++i)
    {
        const murosim::geometry::Circle transformed_circle = transform_ * circles[i];
        murosim::visualization::shader::ShaderCircle::CircleData &data = circle_data[i];
        data.x = transformed_circle.getX();
        data.y = transformed_circle.getY();
        data.radius = transformed_circle.getRadius();
        data.depth = depth;
        data.red = color.r();
        data.green = color.g();
        data.blue = color.b();
        data.alpha = color.a();
    }
    shader_manager_->drawCircle(circle_data);
}

void Canvas::drawLine(const murosim::geometry::Line &line, const murosim::visualization::Color &color, const float thickness, const float depth) const
{
    const murosim::geometry::Line transformed_line = transform_ * line;
    shader_manager_->drawLine({murosim::visualization::shader::ShaderLine::LineData{transformed_line.getPoint1X(), transformed_line.getPoint1Y(), transformed_line.getPoint2X(), transformed_line.getPoint2Y(), thickness_ * thickness, depth, color.r(), color.g(), color.b(), color.a()}});
}

void Canvas::drawLine(const std::vector<murosim::geometry::Line> &lines, const murosim::visualization::Color &color, const float thickness, const float depth) const
{
    std::vector<murosim::visualization::shader::ShaderLine::LineData> line_data(lines.size());
    for (size_t i = 0; i < lines.size(); ++i)
    {
        const murosim::geometry::Line transformed_line = transform_ * lines[i];
        murosim::visualization::shader::ShaderLine::LineData &data = line_data[i];
        data.x1 = transformed_line.getPoint1X();
        data.y1 = transformed_line.getPoint1Y();
        data.x2 = transformed_line.getPoint2X();
        data.y2 = transformed_line.getPoint2Y();
        data.thickness = thickness_ * thickness;
        data.depth = depth;
        data.red = color.r();
        data.green = color.g();
        data.blue = color.b();
        data.alpha = color.a();
    }
    shader_manager_->drawLine(line_data);
}

void Canvas::drawPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color, const float thickness, const float depth) const
{
    drawPath(path, color, color, thickness, depth, path.size());
}

void Canvas::drawPath(const std::vector<Eigen::Vector2f> &path, const murosim::visualization::Color &color_start, const murosim::visualization::Color &color_end, const float thickness, const float depth, const size_t max_length) const
{
    const std::vector<murosim::visualization::shader::ShaderPath::PathData> path_data = buildPathData(path, color_start, color_end, thickness, depth, max_length);
    if (!path_data.empty())
    {
        shader_manager_->drawPath(path_data);
    }
}

void Canvas::drawShape(const murosim::geometry::Shape &shape, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle) const
{
    drawShape(shape, Eigen::Affine2f::Identity(), color, thickness, depth, draw_ring_instead_of_circle);
}

void Canvas::drawShape(const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose, const murosim::visualization::Color &color, const float thickness, const float depth, const bool draw_ring_instead_of_circle) const
{
    const Eigen::Affine2f transform = transform_ * pose;

    const auto& lines = shape.getLines();
    std::vector<murosim::visualization::shader::ShaderLine::LineData> line_data(lines.size());
    for (size_t i = 0; i < lines.size(); i++)
    {
        const murosim::geometry::Line line = transform * lines[i];
        murosim::visualization::shader::ShaderLine::LineData &data = line_data[i];
        data.x1 = line.getPoint1X();
        data.y1 = line.getPoint1Y();
        data.x2 = line.getPoint2X();
        data.y2 = line.getPoint2Y();
        data.thickness = thickness_ * thickness;
        data.depth = depth;
        data.red = color.r();
        data.green = color.g();
        data.blue = color.b();
        data.alpha = color.a();
    }
    shader_manager_->drawLine(line_data);

    const auto& circles = shape.getCircles();
    if (draw_ring_instead_of_circle)
    {
        std::vector<murosim::visualization::shader::ShaderRing::RingData> ring_data(circles.size());
        for (size_t i = 0; i < circles.size(); i++)
        {
            const murosim::geometry::Circle circle = transform * circles[i];
            murosim::visualization::shader::ShaderRing::RingData &data = ring_data[i];
            data.x = circle.getX();
            data.y = circle.getY();
            data.radius = circle.getRadius();
            data.thickness = thickness_ * thickness;
            data.depth = depth;
            data.red = color.r();
            data.green = color.g();
            data.blue = color.b();
            data.alpha = color.a();
        }
        shader_manager_->drawRing(ring_data);
    }
    else
    {
        std::vector<murosim::visualization::shader::ShaderCircle::CircleData> circle_data(circles.size());
        for (size_t i = 0; i < circles.size(); i++)
        {
            const murosim::geometry::Circle circle = transform * circles[i];
            murosim::visualization::shader::ShaderCircle::CircleData &data = circle_data[i];
            data.x = circle.getX();
            data.y = circle.getY();
            data.radius = circle.getRadius();
            data.depth = depth;
            data.red = color.r();
            data.green = color.g();
            data.blue = color.b();
            data.alpha = color.a();
        }
        shader_manager_->drawCircle(circle_data);
    }
}

void Canvas::drawGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const float depth) const
{
    std::vector<murosim::visualization::shader::ShaderQuad::QuadData> quads;
    appendGridMapQuads(grid_map, &color_free, &color_occupied, &color_free, nullptr, depth, quads);
    shader_manager_->drawQuad(quads);
}

void Canvas::drawGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const float depth) const
{
    std::vector<murosim::visualization::shader::ShaderQuad::QuadData> quads;
    appendGridMapQuads(grid_map, &color_free, &color_occupied, &color_buffer, nullptr, depth, quads);
    shader_manager_->drawQuad(quads);
}

void Canvas::drawGridMap(const murosim::grid::Map &grid_map, const murosim::visualization::Color &color_free, const murosim::visualization::Color &color_occupied, const murosim::visualization::Color &color_buffer, const murosim::visualization::Color &color_unreachable, const float depth) const
{
    std::vector<murosim::visualization::shader::ShaderQuad::QuadData> quads;
    appendGridMapQuads(grid_map, &color_free, &color_occupied, &color_buffer, &color_unreachable, depth, quads);
    shader_manager_->drawQuad(quads);
}

void Canvas::drawAll()
{
    shader_manager_->drawCircle(data_circles_);
    shader_manager_->drawLine(data_lines_);
    shader_manager_->drawRing(data_rings_);
    shader_manager_->drawQuad(data_quads_);
    shader_manager_->drawPath(data_paths_);
    data_circles_.clear();
    data_lines_.clear();
    data_rings_.clear();
    data_quads_.clear();
    data_paths_.clear();
}

void Canvas::drawBackground(const murosim::visualization::Color &color) const
{
    shader_manager_->drawAndClearBackground(color.r(), color.g(), color.b(), color.a());
}

void Canvas::setDrawArea(const Eigen::Vector2f area_min, const Eigen::Vector2f area_max)
{
    // world geometry
    const Eigen::Vector2f world_size   = area_max - area_min;
    const Eigen::Vector2f world_center = (area_max + area_min) * 0.5f;

    // unit scale for the ndc translation
    const float s = 2.0f / std::max(world_size.x(), world_size.y());

    transform_.setIdentity();
    transform_.scale(Eigen::Vector2f::Constant(s));
    transform_.translate(-world_center);
}

void Canvas::setDrawArea(Eigen::Vector2f area_min, Eigen::Vector2f area_max, float margin)
{
    assert(area_min.x() < area_max.x());
    assert(area_min.y() < area_max.y());
    assert(0.0f <= margin && margin < 0.5f);

    // world geometry
    const Eigen::Vector2f world_size   = area_max - area_min;
    const Eigen::Vector2f world_center = (area_max + area_min) * 0.5f;

    // unit scale for the ndc translation
    const float scale = (1.0f - margin) * 2.0f / std::max(world_size.x(), world_size.y());

    transform_.setIdentity();
    transform_.scale(Eigen::Vector2f::Constant(scale));
    transform_.translate(-world_center);
}

void Canvas::setDrawArea(const Eigen::Vector2f area_min, const Eigen::Vector2f area_max, const Eigen::Vector2f ndc_origin, const Eigen::Vector2f ndc_size)
{
    assert(area_min.x() < area_max.x());
    assert(area_min.y() < area_max.y());

    // 1. world geometry
    const Eigen::Vector2f world_size   = area_max - area_min;
    const Eigen::Vector2f world_center = (area_max + area_min) * 0.5f;

    // 2. unit scale to fit in ndc size
    const float scale = std::min(ndc_size.x() / world_size.x(), ndc_size.y() / world_size.y());

    transform_.setIdentity();
    transform_.translate(ndc_origin + 0.5f * ndc_size);
    transform_.scale(Eigen::Vector2f::Constant(scale));
    transform_.translate(-world_center);
}

float Canvas::getDrawThickness() const
{
    return thickness_;
}

void Canvas::setDrawThickness(const float thickness)
{
    thickness_ = thickness;
}

void Canvas::setDrawThickness(GLint width, GLint height)
{
    assert(width > 0 && height > 0);

    const float thickness = static_cast<float>(std::min(width, height)) * DEFAULT_THICKNESS_RATIO;
    thickness_ = std::max(DEFAULT_THICKNESS_MIN, thickness);
}

} // namespace murosim::visualization
