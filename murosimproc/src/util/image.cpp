// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/util/image.hpp"

#include <queue>
#include <array>

static const std::array<const Eigen::Vector2i, 8> NEIGHBORS8 = {Eigen::Vector2i(0, -1), Eigen::Vector2i(1, -1), Eigen::Vector2i(1, 0), Eigen::Vector2i(1,1), Eigen::Vector2i(0, 1), Eigen::Vector2i(-1, 1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, -1)}; // North, NorthEast, East, SouthEast, South, SouthWest, West, NorthWest


namespace murosimproc::util::image
{

void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t old_value, const uint8_t new_value)
{
    // visited cells are marked by overwriting with new_value; with
    // old_value == new_value they would never stop matching and the fill
    // would re-push them forever (same guard as murosim's drawing::fillImpl)
    if (old_value == new_value)
    {
        return;
    }

    if (murosimproc::util::image::isInImage(image, point) && at(image, point) == old_value)
    {
        std::vector<Eigen::Vector2i> stack;
        stack.push_back(point);
        at(image, point) = new_value;

        while (!stack.empty())
        {
            Eigen::Vector2i current = stack.back();
            stack.pop_back();

            for (const Eigen::Vector2i &neighbor : NEIGHBORS4)
            {
                const Eigen::Vector2i next = current + neighbor;

                if (murosimproc::util::image::isInImage(image, next) && at(image, next) == old_value)
                {
                    stack.push_back(next);
                    at(image, next) = new_value;
                }
            }
        }
    }
}

void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t old_value, const uint8_t new_value, std::vector<size_t> &indices)
{
    indices.clear();

    // see the guard above: old_value == new_value would never terminate
    if (old_value == new_value)
    {
        return;
    }

    if (murosimproc::util::image::isInImage(image, point) && at(image, point) == old_value)
    {
        std::vector<Eigen::Vector2i> stack;
        stack.push_back(point);
        at(image, point) = new_value;
        indices.push_back(toIndex(image, point));

        while (!stack.empty())
        {
            Eigen::Vector2i current = stack.back();
            stack.pop_back();

            for (const Eigen::Vector2i &neighbor : NEIGHBORS4)
            {
                const Eigen::Vector2i next = current + neighbor;

                if (murosimproc::util::image::isInImage(image, next) && at(image, next) == old_value)
                {
                    stack.push_back(next);
                    const size_t index = toIndex(image, next);
                    indices.push_back(index);
                    image[index] = new_value;
                }
            }
        }
    }
}

void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t new_value)
{
    if (murosimproc::util::image::isInImage(image, point) && at(image, point) != new_value)
    {
        std::vector<Eigen::Vector2i> stack;
        stack.push_back(point);
        at(image, point) = new_value;

        while (!stack.empty())
        {
            Eigen::Vector2i current = stack.back();
            stack.pop_back();

            for (const Eigen::Vector2i &neighbor : NEIGHBORS4)
            {
                const Eigen::Vector2i next = current + neighbor;

                if (murosimproc::util::image::isInImage(image, next) && at(image, next) != new_value)
                {
                    stack.push_back(next);
                    at(image, next) = new_value;
                }
            }
        }
    }
}

void fillRegion(murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point, const uint8_t new_value, std::vector<size_t> &indices)
{
    indices.clear();

    if (murosimproc::util::image::isInImage(image, point) && at(image, point) != new_value)
    {
        std::vector<Eigen::Vector2i> stack;
        stack.push_back(point);
        indices.push_back(toIndex(image, point));
        at(image, point) = new_value;

        while (!stack.empty())
        {
            Eigen::Vector2i current = stack.back();
            stack.pop_back();

            for (const Eigen::Vector2i &neighbor : NEIGHBORS4)
            {
                const Eigen::Vector2i next = current + neighbor;

                if (murosimproc::util::image::isInImage(image, next) && at(image, next) != new_value)
                {
                    stack.push_back(next);
                    const size_t index = toIndex(image, next);
                    indices.push_back(index);
                    image[index] = new_value;
                }
            }
        }
    }
}

bool isInImage(const murosimproc::util::Array2d<uint8_t> &image, const Eigen::Vector2i point)
{
    return 0 <= point.x() && point.x() < static_cast<int>(image.sizeX()) &&
           0 <= point.y() && point.y() < static_cast<int>(image.sizeY());
}

bool isInImage(const murosimproc::util::Array2d<uint8_t> &image, const int x, const int y)
{
    return 0 <= x && x < static_cast<int>(image.sizeX()) && 0 <= y && y < static_cast<int>(image.sizeY());
}

bool isInImage(const murosimproc::util::Array2d<uint8_t> &image, const size_t x, const size_t y)
{
    return x < image.sizeX() && y < image.sizeY();
}

namespace
{

bool isFreeCell(const murosimproc::util::Array2d<uint8_t> &image, const int x, const int y)
{
    return isInImage(image, x, y) && image(static_cast<size_t>(x), static_cast<size_t>(y)) == 0;
}

} // anonymous namespace

bool isNodeCandidate(const murosimproc::util::Array2d<uint8_t> &image, const int x, const int y)
{
    if (!isFreeCell(image, x, y))
    {
        return false;
    }

    size_t free_neighbors = 0;
    for (const Eigen::Vector2i &offset : NEIGHBORS4)
    {
        if (isFreeCell(image, x + offset.x(), y + offset.y()))
        {
            free_neighbors++;
        }
    }

    // Dead end: a parked robot only blocks a cell nobody else needs to pass
    if (free_neighbors == 1)
    {
        return true;
    }

    // Open area: cell is part of a fully free 2x2 block, so other robots can
    // drive around a parked one. Everything else (corridor, curve,
    // intersection) would be blocked.
    for (const int dx : {-1, 1})
    {
        for (const int dy : {-1, 1})
        {
            if (isFreeCell(image, x + dx, y) && isFreeCell(image, x, y + dy) && isFreeCell(image, x + dx, y + dy))
            {
                return true;
            }
        }
    }

    return false;
}

} // namespace murosimproc::util::image

std::ostream& operator<<(std::ostream &os, const murosimproc::util::Array2d<uint8_t> &image)
{
    os << "+";
    for (size_t x = 0; x < image.sizeX(); x++)
    {
        os << "-";
    }
    os << "+\n";

    for (size_t y = 0; y < image.sizeY(); y++)
    {
        os << "|";
        for (size_t x = 0; x < image.sizeX(); x++)
        {
            switch (image(x, y))
            {
            case 0:
                {os << ' ';}
                break;
            case 1:
                {os << '#';}
                break;
            case 2:
                {os << '.';}
                break;
            default:
                {os << '?';}
                break;
            }
        }
        os << "|\n";
    }

    os << "+";
    for (size_t x = 0; x < image.sizeX(); x++)
    {
        os << "-";
    }
    os << "+\n";

    return os;
}
