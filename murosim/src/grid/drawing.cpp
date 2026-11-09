// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/grid/drawing.hpp"
#include "murosim/grid/bresenham.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stack>

namespace
{

bool isInCircle(const Eigen::Vector2i p, const Eigen::Vector2i cp, const int cr)
{
    const Eigen::Vector2i diff = cp - p;
    // Use 64-bit math so cr*cr / the squared distance can't overflow int.
    const std::int64_t dist_squared =
        static_cast<std::int64_t>(diff.x()) * diff.x() + static_cast<std::int64_t>(diff.y()) * diff.y();
    return dist_squared <= static_cast<std::int64_t>(cr) * cr;
}

bool isInCircle(const Eigen::Vector2i p, const Eigen::Vector2f cp, const float cr)
{
    const Eigen::Vector2f diff = cp - p.cast<float>();
    return diff.squaredNorm() <= cr * cr;
}

/// Cell at signed coordinates the caller has checked with isInMap().
uint8_t &cell(murosim::grid::Map &map, const int x, const int y)
{
    return map(static_cast<size_t>(x), static_cast<size_t>(y));
}

struct FillLine
{
    int x1;
    int x2;
    int y;
    int dy;

    FillLine(const int start_x, const int end_x, const int row, const int step_y)
        : x1(start_x), x2(end_x), y(row), dy(step_y)
    {
    }
};

void fillImpl(murosim::grid::Map &map, const uint8_t symbol_old, const uint8_t symbol_new,
              const int px, const int py, std::vector<size_t> *indices)
{
    if (indices)
    {
        indices->clear();
    }

    // Cells are marked visited by overwriting them with symbol_new; if it equals
    // symbol_old nothing is ever marked and the fill loops forever, so bail early.
    if (symbol_old == symbol_new)
    {
        return;
    }

    if (!map.isInMap(px, py) || cell(map, px, py) != symbol_old)
    {
        return;
    }

    std::stack<FillLine> stack;
    stack.emplace(px, px, py, 1);
    stack.emplace(px, px, py - 1, -1);

    while (!stack.empty())
    {
        int x1 = stack.top().x1;
        int x2 = stack.top().x2;
        int y = stack.top().y;
        int dy = stack.top().dy;
        int x = x1;
        stack.pop();

        if (map.isInMap(x, y) && cell(map, x, y) == symbol_old)
        {
            while (map.isInMap(x - 1, y) && cell(map, x - 1, y) == symbol_old)
            {
                if (indices)
                {
                    indices->emplace_back(map.pointToIndex(x - 1, y));
                }
                cell(map, x - 1, y) = symbol_new;
                x = x - 1;
            }
            if (x < x1)
            {
                stack.emplace(x, x1 - 1, y - dy, -dy);
            }
        }

        while (x1 <= x2)
        {
            while (map.isInMap(x1, y) && cell(map, x1, y) == symbol_old)
            {
                if (indices)
                {
                    indices->emplace_back(map.pointToIndex(x1, y));
                }
                cell(map, x1, y) = symbol_new;
                x1 = x1 + 1;
            }
            if (x1 > x)
            {
                stack.emplace(x, x1 - 1, y + dy, dy);
            }
            if (x1 - 1 > x2)
            {
                stack.emplace(x2 + 1, x1 - 1, y - dy, -dy);
            }
            x1 = x1 + 1;
            while (x1 < x2 && map.isInMap(x1, y) && cell(map, x1, y) != symbol_old)
            {
                x1 = x1 + 1;
            }
            x = x1;
        }
    }
}

} // namespace

namespace murosim::grid::drawing
{

void line(Map &map, const uint8_t symbol, const int px1, const int py1, const int px2, const int py2)
{
    // streaming rasterization: map construction draws one line per world
    // line, so avoid materializing a point vector per call
    murosim::grid::bresenham::lineCells(px1, py1, px2, py2, [&map, symbol](const Eigen::Vector2i point) {
        map.set(symbol, point);
    });
}

void line(Map &map, const uint8_t symbol, const Eigen::Vector2i p1, const Eigen::Vector2i p2)
{
    line(map, symbol, p1.x(), p1.y(), p2.x(), p2.y());
}

void line(Map &map, const uint8_t symbol, const float p1x, const float p1y, const float p2x, const float p2y)
{
    int x1, y1, x2, y2;

    map.fromWorldIntoMap(p1x, p1y, x1, y1);
    map.fromWorldIntoMap(p2x, p2y, x2, y2);

    line(map, symbol, x1, y1, x2, y2);
}

void line(Map &map, const uint8_t symbol, const Eigen::Vector2f p1, const Eigen::Vector2f p2)
{
    int x1, y1, x2, y2;

    map.fromWorldIntoMap(p1, x1, y1);
    map.fromWorldIntoMap(p2, x2, y2);

    line(map, symbol, x1, y1, x2, y2);
}

void line(Map &map, const uint8_t symbol, const murosim::geometry::Line l)
{
    int x1, y1, x2, y2;

    map.fromWorldIntoMap(l.getPoint1(), x1, y1);
    map.fromWorldIntoMap(l.getPoint2(), x2, y2);

    line(map, symbol, x1, y1, x2, y2);
}

void circle(Map &map, const uint8_t symbol, const int cx, const int cy, const int cr)
{
    circle(map, symbol, Eigen::Vector2i(cx, cy), cr);
}

void circle(Map &map, const uint8_t symbol, const Eigen::Vector2i cp, const int cr)
{
    const int x_begin = std::max<int>(0, cp.x() - cr);
    const int x_end = std::min<int>(static_cast<int>(map.sizeX()) - 1, cp.x() + cr);
    const int y_begin = std::max<int>(0, cp.y() - cr);
    const int y_end = std::min<int>(static_cast<int>(map.sizeY()) - 1, cp.y() + cr);

    Eigen::Vector2i pixel(x_begin, y_begin);

    for (pixel.x() = x_begin; pixel.x() <= x_end; pixel.x()++)
    {
        for (pixel.y() = y_begin; pixel.y() <= y_end; pixel.y()++)
        {
            if (isInCircle(pixel, cp, cr))
            {
                map.set(symbol, pixel);
            }
        }
    }
}

void circle(Map &map, const uint8_t symbol, const float x, const float y, const float r)
{
    circle(map, symbol, Eigen::Vector2f(x, y), r);
}

void circle(Map &map, const uint8_t symbol, const Eigen::Vector2f cp, const float cr)
{
    circleCells(map, cp, cr, [&map, symbol](const Eigen::Vector2i pixel) {
        map.set(symbol, pixel);
    });
}

void circleCells(const Map &map, const Eigen::Vector2f cp, const float cr, const std::function<void(Eigen::Vector2i)> &visit)
{
    // always visit the center cell, so even a circle far smaller than one cell
    // occupies at least one pixel
    Eigen::Vector2i pixel_center;
    map.fromWorldIntoMap(cp, pixel_center);
    if (map.isInMap(pixel_center))
    {
        visit(pixel_center);
    }

    Eigen::Vector2f p;
    map.fromWorldIntoMap(cp, p);

    // + 0.5 accounts for the half-width of the cells themselves, so every cell
    // the circle physically overlaps is visited
    const float r = cr * map.getGridScaleInv() + 0.5f;

    const int x_begin = std::max(0, static_cast<int>(std::ceil(p.x() - r)));
    const int x_end = std::min(static_cast<int>(map.sizeX()) - 1, static_cast<int>(std::floor(p.x() + r)));
    const int y_begin = std::max(0, static_cast<int>(std::ceil(p.y() - r)));
    const int y_end = std::min(static_cast<int>(map.sizeY()) - 1, static_cast<int>(std::floor(p.y() + r)));

    Eigen::Vector2i pixel(x_begin, y_begin);

    for (pixel.x() = x_begin; pixel.x() <= x_end; pixel.x()++)
    {
        for (pixel.y() = y_begin; pixel.y() <= y_end; pixel.y()++)
        {
            const bool is_center = (pixel.x() == pixel_center.x()) && (pixel.y() == pixel_center.y());
            if (!is_center && isInCircle(pixel, p, r))
            {
                visit(pixel);
            }
        }
    }
}

void circle(Map &map, const uint8_t symbol, const murosim::geometry::Circle c)
{
    circle(map, symbol, c.getX(), c.getY(), c.getRadius());
}

void fill(Map &map, const uint8_t symbol_old, const uint8_t symbol_new, const int px, const int py)
{
    fillImpl(map, symbol_old, symbol_new, px, py, nullptr);
}

void fill(Map &map, const uint8_t symbol_old, const uint8_t symbol_new, const Eigen::Vector2i p)
{
    fillImpl(map, symbol_old, symbol_new, p.x(), p.y(), nullptr);
}

void fill(Map &map, const uint8_t symbol_old, const uint8_t symbol_new, const int px, const int py, std::vector<size_t> &indices)
{
    fillImpl(map, symbol_old, symbol_new, px, py, &indices);
}

void fill(Map &map, const uint8_t symbol_old, const uint8_t symbol_new, const Eigen::Vector2i p, std::vector<size_t> &indices)
{
    fillImpl(map, symbol_old, symbol_new, p.x(), p.y(), &indices);
}

} // namespace murosim::grid::drawing
