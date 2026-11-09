// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/grid/bresenham.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
// The midpoint circle algorithm picks the cell closest to the ideal circle,
// so every outline cell lies within half a cell of the radius (measured
// maximum 0.44 for r = 10).
constexpr float RASTER_TOLERANCE = 0.5f;

float distanceFrom(const Eigen::Vector2i &point, const int cx, const int cy)
{
    return std::hypot(static_cast<float>(point.x() - cx), static_cast<float>(point.y() - cy));
}
} // namespace

// ============================================================================
// Line Tests
// ============================================================================

TEST(BresenhamLine, HorizontalLine)
{
    const auto line = murosim::grid::bresenham::createLine(0, 0, 5, 0);

    ASSERT_EQ(line.size(), static_cast<size_t>(6));
    for (size_t i = 0; i < line.size(); i++)
    {
        EXPECT_EQ(line[i].x(), static_cast<int>(i));
        EXPECT_EQ(line[i].y(), 0);
    }
}

TEST(BresenhamLine, VerticalLine)
{
    const auto line = murosim::grid::bresenham::createLine(0, 0, 0, 5);

    ASSERT_EQ(line.size(), static_cast<size_t>(6));
    for (size_t i = 0; i < line.size(); i++)
    {
        EXPECT_EQ(line[i].x(), 0);
        EXPECT_EQ(line[i].y(), static_cast<int>(i));
    }
}

TEST(BresenhamLine, DiagonalLine)
{
    const auto line = murosim::grid::bresenham::createLine(0, 0, 5, 5);

    ASSERT_EQ(line.size(), static_cast<size_t>(6));
    for (size_t i = 0; i < line.size(); i++)
    {
        EXPECT_EQ(line[i].x(), static_cast<int>(i));
        EXPECT_EQ(line[i].y(), static_cast<int>(i));
    }
}

TEST(BresenhamLine, ReverseLineVisitsTheSameCells)
{
    const auto line_forward = murosim::grid::bresenham::createLine(0, 0, 5, 0);
    const auto line_backward = murosim::grid::bresenham::createLine(5, 0, 0, 0);

    ASSERT_EQ(line_forward.size(), line_backward.size());
    for (size_t i = 0; i < line_forward.size(); i++)
    {
        const Eigen::Vector2i &mirrored = line_backward[line_backward.size() - 1 - i];
        EXPECT_EQ(line_forward[i].x(), mirrored.x());
        EXPECT_EQ(line_forward[i].y(), mirrored.y());
    }
}

TEST(BresenhamLine, SinglePoint)
{
    const auto line = murosim::grid::bresenham::createLine(5, 5, 5, 5);

    ASSERT_EQ(line.size(), static_cast<size_t>(1));
    EXPECT_EQ(line[0].x(), 5);
    EXPECT_EQ(line[0].y(), 5);
}

TEST(BresenhamLine, NegativeCoordinates)
{
    const auto line = murosim::grid::bresenham::createLine(-5, -5, 5, 5);

    // One cell per step along the longer axis, both end points included
    ASSERT_EQ(line.size(), static_cast<size_t>(11));
    EXPECT_EQ(line.front().x(), -5);
    EXPECT_EQ(line.front().y(), -5);
    EXPECT_EQ(line.back().x(), 5);
    EXPECT_EQ(line.back().y(), 5);
}

// ============================================================================
// Circle Tests
// ============================================================================

TEST(BresenhamCircle, SmallCircleLiesOnTheRadius)
{
    const auto circle = murosim::grid::bresenham::createCircle(0, 0, 3);

    ASSERT_GT(circle.size(), static_cast<size_t>(0));
    for (const Eigen::Vector2i &point : circle)
    {
        EXPECT_NEAR(distanceFrom(point, 0, 0), 3.0f, RASTER_TOLERANCE) << "cell " << point.transpose();
    }
}

TEST(BresenhamCircle, ZeroRadiusIsTheCenter)
{
    const auto circle = murosim::grid::bresenham::createCircle(0, 0, 0);

    ASSERT_EQ(circle.size(), static_cast<size_t>(1));
    EXPECT_EQ(circle[0].x(), 0);
    EXPECT_EQ(circle[0].y(), 0);
}

TEST(BresenhamCircle, LargeCircleLiesOnTheRadius)
{
    const auto circle = murosim::grid::bresenham::createCircle(10, 10, 20);

    ASSERT_GT(circle.size(), static_cast<size_t>(0));
    for (const Eigen::Vector2i &point : circle)
    {
        ASSERT_NEAR(distanceFrom(point, 10, 10), 20.0f, RASTER_TOLERANCE) << "cell " << point.transpose();
    }
}

TEST(BresenhamCircle, NegativeCenter)
{
    const auto circle = murosim::grid::bresenham::createCircle(-5, -5, 3);

    ASSERT_GT(circle.size(), static_cast<size_t>(0));
    for (const Eigen::Vector2i &point : circle)
    {
        EXPECT_NEAR(distanceFrom(point, -5, -5), 3.0f, RASTER_TOLERANCE) << "cell " << point.transpose();
    }
}

// ============================================================================
// Filled Circle Tests
// ============================================================================

TEST(BresenhamCircleFilled, SmallFilledCircleStaysInsideTheRadius)
{
    const auto circle = murosim::grid::bresenham::createCircleFilled(0, 0, 3);
    const auto outline = murosim::grid::bresenham::createCircle(0, 0, 3);

    // The disk has more cells than its outline
    ASSERT_GT(circle.size(), outline.size());
    for (const Eigen::Vector2i &point : circle)
    {
        EXPECT_LE(distanceFrom(point, 0, 0), 3.0f + RASTER_TOLERANCE) << "cell " << point.transpose();
    }
}

TEST(BresenhamCircleFilled, ZeroRadiusIsTheCenter)
{
    const auto circle = murosim::grid::bresenham::createCircleFilled(0, 0, 0);

    ASSERT_EQ(circle.size(), static_cast<size_t>(1));
    EXPECT_EQ(circle[0].x(), 0);
    EXPECT_EQ(circle[0].y(), 0);
}

TEST(BresenhamCircleFilled, ContainsCenter)
{
    const auto circle = murosim::grid::bresenham::createCircleFilled(5, 5, 3);

    const auto it = std::find_if(circle.begin(), circle.end(), [](const Eigen::Vector2i &p)
    {
        return p.x() == 5 && p.y() == 5;
    });

    EXPECT_NE(it, circle.end());
}

// ============================================================================
// Kernel Circle Tests
// ============================================================================

TEST(BresenhamKernelCircle, SmallKernel)
{
    // createKernelCircle returns (min_y, max_y) for each x column of the
    // (2r + 1) x (2r + 1) window; the middle column spans the whole window
    const auto kernel = murosim::grid::bresenham::createKernelCircle(2);

    ASSERT_EQ(kernel.size(), static_cast<size_t>(5));
    EXPECT_EQ(kernel[0].first, 1);
    EXPECT_EQ(kernel[0].second, 3);
    EXPECT_EQ(kernel[2].first, 0);
    EXPECT_EQ(kernel[2].second, 4);
    EXPECT_EQ(kernel[4].first, 1);
    EXPECT_EQ(kernel[4].second, 3);
}

TEST(BresenhamKernelCircle, ZeroRadius)
{
    const auto kernel = murosim::grid::bresenham::createKernelCircle(0);

    // For radius < 2 the kernel is the single centre cell
    ASSERT_EQ(kernel.size(), static_cast<size_t>(1));
    EXPECT_EQ(kernel[0].first, 0);
    EXPECT_EQ(kernel[0].second, 0);
}

TEST(BresenhamKernelCircle, RadiusOne)
{
    const auto kernel = murosim::grid::bresenham::createKernelCircle(1);

    // For radius < 2 the kernel is the single centre cell
    ASSERT_EQ(kernel.size(), static_cast<size_t>(1));
    EXPECT_EQ(kernel[0].first, 0);
    EXPECT_EQ(kernel[0].second, 0);
}

TEST(BresenhamKernelCircle, LargeKernel)
{
    const auto kernel = murosim::grid::bresenham::createKernelCircle(10);

    ASSERT_EQ(kernel.size(), static_cast<size_t>(21));
    for (const auto &[min_y, max_y] : kernel)
    {
        EXPECT_LE(min_y, max_y);
        EXPECT_GE(min_y, 0);
        EXPECT_LE(max_y, 20);
    }

    // The middle column spans the whole window
    EXPECT_EQ(kernel[10].first, 0);
    EXPECT_EQ(kernel[10].second, 20);
}

// ============================================================================
// Kernel Rectangle Tests
// ============================================================================

TEST(BresenhamKernelRectangle, SmallKernel)
{
    // createKernelRectangle returns (0, kernel_length - 1) for each of the
    // kernel_length = 2 * length + 1 columns
    const auto kernel = murosim::grid::bresenham::createKernelRectangle(3);

    ASSERT_EQ(kernel.size(), static_cast<size_t>(7));
    for (const auto &[min_y, max_y] : kernel)
    {
        EXPECT_EQ(min_y, 0);
        EXPECT_EQ(max_y, 6);
    }
}

TEST(BresenhamKernelRectangle, SingleCell)
{
    const auto kernel = murosim::grid::bresenham::createKernelRectangle(1);

    // For length < 2 the kernel is the single centre cell
    ASSERT_EQ(kernel.size(), static_cast<size_t>(1));
    EXPECT_EQ(kernel[0].first, 0);
    EXPECT_EQ(kernel[0].second, 0);
}

TEST(BresenhamKernelRectangle, LengthTwo)
{
    const auto kernel = murosim::grid::bresenham::createKernelRectangle(2);

    ASSERT_EQ(kernel.size(), static_cast<size_t>(5));
    for (const auto &[min_y, max_y] : kernel)
    {
        EXPECT_EQ(min_y, 0);
        EXPECT_EQ(max_y, 4);
    }
}

TEST(BresenhamKernelRectangle, LargerSize)
{
    const auto kernel = murosim::grid::bresenham::createKernelRectangle(4);

    ASSERT_EQ(kernel.size(), static_cast<size_t>(9));
    for (const auto &[min_y, max_y] : kernel)
    {
        EXPECT_EQ(min_y, 0);
        EXPECT_EQ(max_y, 8);
    }
}

// ============================================================================
// Line Cells Tests
// ============================================================================

TEST(BresenhamLineCells, VisitsTheSameCellsAsCreateLineInOrder)
{
    const std::vector<Eigen::Vector2i> expected = murosim::grid::bresenham::createLine(-3, 7, 12, -2);
    std::vector<Eigen::Vector2i> visited;

    murosim::grid::bresenham::lineCells(-3, 7, 12, -2,
                                        [&visited](const Eigen::Vector2i cell) { visited.push_back(cell); });

    ASSERT_EQ(visited.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
    {
        EXPECT_EQ(visited[i], expected[i]) << "cell " << i;
    }
}

TEST(BresenhamLineCells, SinglePointVisitsOnce)
{
    std::vector<Eigen::Vector2i> visited;

    murosim::grid::bresenham::lineCells(4, 4, 4, 4,
                                        [&visited](const Eigen::Vector2i cell) { visited.push_back(cell); });

    ASSERT_EQ(visited.size(), static_cast<size_t>(1));
    EXPECT_EQ(visited.front(), Eigen::Vector2i(4, 4));
}
