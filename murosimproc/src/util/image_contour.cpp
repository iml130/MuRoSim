// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/util/image.hpp"

namespace
{

// Only used in assert(), which NDEBUG removes.
[[maybe_unused]] bool isPointAdjacent(const Eigen::Vector2i point1, const Eigen::Vector2i point2)
{
    const Eigen::Vector2i diff = (point1 - point2).cwiseAbs();
    return diff.x() <= 1 && diff.y() <= 1 && (diff.x() + diff.y() == 1);
}

void addBorderPoint(std::vector<Eigen::Vector2f> &contour, const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point_wall, const Eigen::Vector2i point_free)
{
    assert(murosimproc::util::image::isInImage(image, point_wall));
    assert(murosimproc::util::image::isInImage(image, point_free));
    assert(point_wall != point_free);

    const int horizontal_distance = std::abs(point_wall.x() - point_free.x());
    const int vertical_distance = std::abs(point_wall.y() - point_free.y());

    assert(horizontal_distance <= 1 || vertical_distance <= 1);
    assert(horizontal_distance + vertical_distance == 1);

    if (murosimproc::util::image::isInImage(image, point_wall) &&
        murosimproc::util::image::isInImage(image, point_free) &&
        (horizontal_distance <= 1 || vertical_distance <= 1) &&
        (horizontal_distance + vertical_distance == 1))
    {
        const Eigen::Vector2f point = (point_wall + point_free).cast<float>() * 0.5f;

        if(horizontal_distance == 1)
        {
            if (0 == point_wall.y())
            {
                contour.push_back(point + Eigen::Vector2f(0.5f, -0.0f));
            }
            else if (point_wall.y() == static_cast<int>(image.sizeY()) - 1)
            {
                contour.push_back(point + Eigen::Vector2f(0.5f, 1.0f));
            }
        }
        else
        {
            if (0 == point_wall.x())
            {
                contour.push_back(point + Eigen::Vector2f(-0.0f, 0.5f));
            }
            else if (point_wall.x() == static_cast<int>(image.sizeX()) - 1)
            {
                contour.push_back(point + Eigen::Vector2f(1.0f, 0.5f));
            }
        }
    }
}

std::vector<Eigen::Vector2f> followContourCounterClockwise(const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i start_point_wall, const Eigen::Vector2i start_point_free, bool &loop_closed)
{
    assert(murosimproc::util::image::isInImage(image, start_point_free));
    assert(murosimproc::util::image::isInImage(image, start_point_wall));
    assert(murosimproc::util::image::at(image, start_point_free) == 0);
    assert(murosimproc::util::image::at(image, start_point_wall) != 0);

    std::vector<Eigen::Vector2f> contour;

    size_t direction_free = NEIGHBORS4.size();

    for (size_t i = 0; i < NEIGHBORS4.size(); i++)
    {
        if (start_point_wall + NEIGHBORS4[i] == start_point_free)
        {
            direction_free = i;
            break;
        }
    }

    assert(direction_free < NEIGHBORS4.size());

    Eigen::Vector2i point_wall = start_point_wall;
    Eigen::Vector2i point_free = start_point_free;

    while(true)
    {
        const size_t direction_next = (direction_free + NEIGHBORS4.size() - 1) % NEIGHBORS4.size();
        const Eigen::Vector2i point = point_wall + NEIGHBORS4[direction_next];

        if (!murosimproc::util::image::isInImage(image, point))
        {
            break;
        }

        // check for the next direction is a wall point
        if (murosimproc::util::image::at(image, point) == 1)
        {
            const Eigen::Vector2i point_next = point + NEIGHBORS4[direction_free];

            if (!murosimproc::util::image::isInImage(image, point_next))
            {
                break;
            }

            // when the next point is free then wall is straight.
            if (murosimproc::util::image::at(image, point_next) == 0)
            {
                point_wall = point;
                point_free = point_next;
            }
            else
            {
                contour.push_back((point_wall + point_next).cast<float>() * 0.5f + Eigen::Vector2f(0.5f, 0.5f));
                point_wall = point_next;
                direction_free = (direction_free + 1) % NEIGHBORS4.size();
            }
        }
        else
        {
            contour.push_back((point_free + point).cast<float>() * 0.5f + Eigen::Vector2f(0.5f, 0.5f));
            point_free = point;
            direction_free = (direction_free + NEIGHBORS4.size() - 1) % NEIGHBORS4.size();
        }

        if (point_wall == start_point_wall && point_free == start_point_free)
        {
            loop_closed = true;
            contour.push_back(contour.front());
            return contour;
        }
    }

    addBorderPoint(contour, image, point_wall, point_free);
    loop_closed = false;

    return contour;
}

std::vector<Eigen::Vector2f> followContourClockwise(const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i start_point_wall, const Eigen::Vector2i start_point_free, bool &loop_closed)
{
    assert(murosimproc::util::image::isInImage(image, start_point_free));
    assert(murosimproc::util::image::isInImage(image, start_point_wall));
    assert(murosimproc::util::image::at(image, start_point_free) == 0);
    assert(murosimproc::util::image::at(image, start_point_wall) != 0);

    std::vector<Eigen::Vector2f> contour;

    size_t direction_free = NEIGHBORS4.size();

    for (size_t i = 0; i < NEIGHBORS4.size(); i++)
    {
        if (start_point_wall + NEIGHBORS4[i] == start_point_free)
        {
            direction_free = i;
            break;
        }
    }

    assert(direction_free < NEIGHBORS4.size());

    Eigen::Vector2i point_wall = start_point_wall;
    Eigen::Vector2i point_free = start_point_free;

    while(true)
    {
        const size_t direction_next = (direction_free + 1) % NEIGHBORS4.size();
        const Eigen::Vector2i point = point_wall + NEIGHBORS4[direction_next];

        if (!murosimproc::util::image::isInImage(image, point))
        {
            break;
        }

        // check for the next direction is a wall point
        if (murosimproc::util::image::at(image, point) == 1)
        {
            const Eigen::Vector2i point_next = point + NEIGHBORS4[direction_free];

            if (!murosimproc::util::image::isInImage(image, point_next))
            {
                break;
            }

            // when the next point is free then wall is straight.
            if (murosimproc::util::image::at(image, point_next) == 0)
            {
                point_wall = point;
                point_free = point_next;
            }
            else
            {
                contour.push_back((point_wall + point_next).cast<float>() * 0.5f + Eigen::Vector2f(0.5f, 0.5f));
                point_wall = point_next;
                direction_free = (direction_free + NEIGHBORS4.size() - 1) % NEIGHBORS4.size();
            }
        }
        else
        {
            contour.push_back((point_free + point).cast<float>() * 0.5f + Eigen::Vector2f(0.5f, 0.5f));
            point_free = point;
            direction_free = (direction_free + 1) % NEIGHBORS4.size();
        }

        if (point_wall == start_point_wall && point_free == start_point_free)
        {
            loop_closed = true;
            contour.push_back(contour.front());
            return contour;
        }
    }

    addBorderPoint(contour, image, point_wall, point_free);
    loop_closed = false;

    return contour;
}

std::vector<Eigen::Vector2f> followContour(const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i start_point1, const Eigen::Vector2i start_point2)
{
    std::vector<Eigen::Vector2f> contour;

    assert(murosimproc::util::image::isInImage(image, start_point1));
    assert(murosimproc::util::image::isInImage(image, start_point2));
    assert(isPointAdjacent(start_point1, start_point2));
    assert((murosimproc::util::image::at(image, start_point1) == 0) != (murosimproc::util::image::at(image, start_point2) == 0));

    const Eigen::Vector2i start_point_free = (murosimproc::util::image::at(image, start_point1) == 0) ? start_point1 : start_point2;
    const Eigen::Vector2i start_point_wall = (murosimproc::util::image::at(image, start_point1) == 0) ? start_point2 : start_point1;

    assert(start_point_free != start_point_wall);

    bool loop_closed = false;

    std::vector<Eigen::Vector2f> contour_clockwise = followContourClockwise(image, start_point_wall, start_point_free, loop_closed);

    if (loop_closed)
    {
        return contour_clockwise;
    }

    std::vector<Eigen::Vector2f> contour_counter_clockwise = followContourCounterClockwise(image, start_point_wall, start_point_free, loop_closed);

    contour.insert(contour.end(), contour_clockwise.crbegin(), contour_clockwise.crend());
    contour.insert(contour.end(), contour_counter_clockwise.cbegin(), contour_counter_clockwise.cend());

    return contour;
}

} // namespace


namespace murosimproc::util::image
{

std::vector<std::vector<Eigen::Vector2f>> findContourPoints(const murosimproc::util::Array2d<uint8_t> &image)
{
    std::vector<std::vector<Eigen::Vector2f>> contours;
    murosimproc::util::Array2d<uint8_t> image_binary = image;

    // binarize image, so that all non zero values are ones.
    for (size_t i = 0; i < image_binary.size(); i++)
    {
        image_binary[i] = (image_binary[i] != 0) ? 1 : 0;
    }

    // iterate over all pixels
    for (size_t y = 0; y < image_binary.sizeY() - 1; y++)
    {
        for (size_t x = 0; x < image_binary.sizeX() - 1; x++)
        {
            if(image_binary(x, y) != image_binary(x + 1, y))
            {
                std::vector<Eigen::Vector2f> contour_points = followContour(image_binary, Eigen::Vector2i(x, y), Eigen::Vector2i(x + 1, y));
                murosimproc::util::image::fillRegion(image_binary, Eigen::Vector2i(x, y), 0);
                murosimproc::util::image::fillRegion(image_binary, Eigen::Vector2i(x + 1, y), 0);

                if (!contour_points.empty())
                {
                    contours.push_back(contour_points);
                }
            }
            else if (image_binary(x, y) != image_binary(x, y + 1))
            {
                std::vector<Eigen::Vector2f> contour_points = followContour(image_binary, Eigen::Vector2i(x, y), Eigen::Vector2i(x, y + 1));
                murosimproc::util::image::fillRegion(image_binary, Eigen::Vector2i(x, y), 0);
                murosimproc::util::image::fillRegion(image_binary, Eigen::Vector2i(x, y + 1), 0);

                if (!contour_points.empty())
                {
                    contours.push_back(contour_points);
                }
            }
        }
    }

    return contours;
}

std::vector<murosim::geometry::Line> findContourLines(const murosimproc::util::Array2d<uint8_t> &image)
{
    std::vector<murosim::geometry::Line> lines;
    std::vector<std::vector<Eigen::Vector2f>> contours = findContourPoints(image);

    for (const std::vector<Eigen::Vector2f> &contour : contours)
    {
        if (1 < contour.size())
        {
            for (size_t i = 1; i < contour.size(); i++)
            {
                lines.emplace_back(contour[i - 1], contour[i]);
            }
        }
    }

    return lines;
}

} // namespace murosimproc::util::image
