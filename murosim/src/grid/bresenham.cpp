// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/grid/bresenham.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace murosim::grid::bresenham
{

void lineCells(const int x1, const int y1, const int x2, const int y2, const std::function<void(Eigen::Vector2i)> &visit)
{
    // Bresenham's line algorithm: step one cell at a time along the driving axis
    // (the one with the larger extent) and let the other axis catch up whenever
    // the ideal line has drifted more than half a cell away from the current one.
    const int run = std::max(std::abs(x2 - x1), std::abs(y2 - y1));
    const bool x_drives = std::abs(x2 - x1) >= std::abs(y2 - y1);
    const int rise = x_drives ? std::abs(y2 - y1) : std::abs(x2 - x1);
    const int drive_step = x_drives ? (x2 > x1) - (x2 < x1) : (y2 > y1) - (y2 < y1);
    const int follow_step = x_drives ? (y2 > y1) - (y2 < y1) : (x2 > x1) - (x2 < x1);

    Eigen::Vector2i cell{x1, y1};
    int &drive = x_drives ? cell.x() : cell.y();
    int &follow = x_drives ? cell.y() : cell.x();

    visit(cell);

    // Drift of the ideal line from the cell centre along the following axis, in
    // units of 1 / (2 * run) cells so it stays an integer.
    int drift = 0;
    for (int step = 0; step < run; ++step)
    {
        drive += drive_step;
        drift += 2 * rise;

        // Exactly half a cell is resolved towards the positive driving direction,
        // so a line and its reverse cover the same cells.
        if (drift > run || (drift == run && drive_step > 0))
        {
            follow += follow_step;
            drift -= 2 * run;
        }

        visit(cell);
    }
}

std::vector<Eigen::Vector2i> createLine(const int x1, const int y1, const int x2, const int y2)
{
    std::vector<Eigen::Vector2i> points;
    lineCells(x1, y1, x2, y2, [&points](const Eigen::Vector2i point) {
        points.push_back(point);
    });
    return points;
}

std::vector<Eigen::Vector2i> createCircle(const int cx, const int cy, const int r)
{
    // Midpoint circle algorithm: walk the octant from (r, 0) up to the diagonal,
    // one row at a time, and mirror each cell into the other seven octants.
    std::vector<Eigen::Vector2i> points;

    if (r <= 0)
    {
        points.push_back(Eigen::Vector2i{cx, cy});
        return points;
    }

    // Adds (x, y) in all octants; cells on an axis or on the diagonal are shared
    // by two octants and added once.
    const auto mirror = [&points, cx, cy](const int x, const int y) {
        for (const auto &[a, b] : {std::pair{x, y}, std::pair{y, x}})
        {
            points.push_back(Eigen::Vector2i{cx + a, cy + b});
            if (a != 0)
            {
                points.push_back(Eigen::Vector2i{cx - a, cy + b});
            }
            if (b != 0)
            {
                points.push_back(Eigen::Vector2i{cx + a, cy - b});
            }
            if (a != 0 && b != 0)
            {
                points.push_back(Eigen::Vector2i{cx - a, cy - b});
            }
            if (x == y)
            {
                break;
            }
        }
    };

    const long long r_squared = static_cast<long long>(r) * r;
    int x = r;
    mirror(x, 0);

    for (int y = 1; x >= y; ++y)
    {
        // Step inwards once the midpoint between x and x - 1 lies outside the
        // circle: y^2 + (x - 1/2)^2 > r^2, with the constant 1/4 dropped so the
        // test stays in integers.
        if (static_cast<long long>(x) * x - x + static_cast<long long>(y) * y > r_squared)
        {
            --x;
        }

        if (x < y)
        {
            break;
        }

        mirror(x, y);
    }

    return points;
}

std::vector<Eigen::Vector2i> createCircleFilled(const int cx, const int cy, const int r)
{
    const std::vector<Eigen::Vector2i> circle = createCircle(cx, cy, r);

    if (circle.size() <= 1)
    {
        return circle;
    }

    int y_min = std::numeric_limits<int>::max();
    int y_max = std::numeric_limits<int>::min();

    for (const auto &point : circle)
    {
        y_min = std::min(y_min, point.y());
        y_max = std::max(y_max, point.y());
    }

    std::vector<Eigen::Vector2i> boundaries(static_cast<size_t>(y_max - y_min + 1), Eigen::Vector2i{std::numeric_limits<int>::max(), std::numeric_limits<int>::min()});

    for (const auto &point : circle)
    {
        Eigen::Vector2i &boundary = boundaries[static_cast<size_t>(point.y() - y_min)];
        boundary.x() = std::min(boundary.x(), point.x());
        boundary.y() = std::max(boundary.y(), point.x());
    }

    std::vector<Eigen::Vector2i> circle_filled;

    for (size_t y = 1; y < boundaries.size() - 1; y++)
    {
        const int x_min = boundaries[y].x();
        const int x_max = boundaries[y].y();

        // Skip rows with no outline point (x_min/x_max still at their sentinels),
        // otherwise x_min + 1 would overflow.
        if (x_min > x_max)
        {
            continue;
        }

        for (int x = x_min + 1; x < x_max; x++)
        {
            circle_filled.emplace_back(Eigen::Vector2i{x, static_cast<int>(y) + y_min});
        }
    }

    return circle_filled;
}

std::vector<std::pair<int, int>> createKernelCircle(const int radius)
{
    std::vector<std::pair<int, int>> kernel;

    if (radius < 2)
    {
        kernel.emplace_back(0, 0);
        return kernel;
    }

    std::vector<Eigen::Vector2i> circle = createCircle(radius, radius, radius);

    // The kernel stores a (min_y, max_y) span per x-column, so it must be sized
    // by the x extent (not y). createCircle(r, r, r) is symmetric so x_max == y_max,
    // but deriving the size from x keeps it correct regardless and the assert below
    // guards against any future asymmetry.
    int x_max = std::numeric_limits<int>::min();

    for (const auto &point : circle)
    {
        x_max = std::max(x_max, point.x());
    }

    for (int i = 0; i <= x_max; ++i)
    {
        kernel.emplace_back(std::numeric_limits<int>::max(), std::numeric_limits<int>::min());
    }

    for (const auto &point : circle)
    {
        const size_t x = static_cast<size_t>(point.x());
        const int y = point.y();

        assert(x < kernel.size());
        kernel[x].first = std::min(kernel[x].first, y);
        kernel[x].second = std::max(kernel[x].second, y);
    }

    return kernel;
}

std::vector<std::pair<int, int>> createKernelRectangle(const int length)
{
    std::vector<std::pair<int, int>> kernel;

    if (length < 2)
    {
        kernel.emplace_back(0, 0);
        return kernel;
    }

    const int kernel_length = length * 2 + 1;

    // Each column spans rows 0..kernel_length-1 inclusive, matching the
    // kernel_length columns so the kernel is square (and consistent with
    // createKernelCircle, where .second is an inclusive max row index).
    for (int i = 0; i < kernel_length; ++i)
    {
        kernel.emplace_back(0, kernel_length - 1);
    }

    return kernel;
}

} // namespace murosim::grid::bresenham
