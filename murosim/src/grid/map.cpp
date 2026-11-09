// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/grid/map.hpp"

#include "murosim/grid/drawing.hpp"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <iterator>
#include <limits>
#include <stdexcept>

const std::array<uint8_t, 256> murosim::grid::Map::LUT_WITH_BUFFER_UINT8_T = []() {
    std::array<uint8_t, 256> lut{};
    lut.fill(255);
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::FREE)] = 0;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::OCCUPIED)] = 255;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::BUFFER)] = 128;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::UNREACHABLE)] = 255;
    return lut;
}();

const std::array<uint8_t, 256> murosim::grid::Map::LUT_NO_BUFFER_UINT8_T = []() {
    std::array<uint8_t, 256> lut{};
    lut.fill(255);
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::FREE)] = 0;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::OCCUPIED)] = 255;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::BUFFER)] = 0;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::UNREACHABLE)] = 255;
    return lut;
}();

const std::array<float, 256> murosim::grid::Map::LUT_WITH_BUFFER_FLOAT = []() {
    std::array<float, 256> lut{};
    lut.fill(1.0f);
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::FREE)] = 0.0f;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::OCCUPIED)] = 1.0f;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::BUFFER)] = 0.5f;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::UNREACHABLE)] = 1.0f;
    return lut;
}();

const std::array<float, 256> murosim::grid::Map::LUT_NO_BUFFER_FLOAT = []() {
    std::array<float, 256> lut{};
    lut.fill(1.0f);
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::FREE)] = 0.0f;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::OCCUPIED)] = 1.0f;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::BUFFER)] = 0.0f;
    lut[static_cast<uint8_t>(murosim::grid::Map::FIELD::UNREACHABLE)] = 1.0f;
    return lut;
}();

namespace murosim::grid
{

namespace
{

auto findLargestRegion(const std::vector<std::vector<size_t>> &regions)
{
    return std::max_element(regions.begin(), regions.end(),
        [](const auto &a, const auto &b) { return a.size() < b.size(); });
}

} // namespace

Map::Map()
    : map_scale_(0)
    , map_scale_inv_(0)
    , buffer_scale_(0)
    , origin_(Eigen::Vector2f(0.0f, 0.0f))
    , length_(Eigen::Vector2f(0.0f, 0.0f))
    , size_x_(0)
    , size_y_(0)
{
}

Map::Map(const murosim::geometry::World &world, const std::vector<Eigen::Vector2f> &points, const float grid_scale, const float buffer_scale)
    : map_scale_(std::max(grid_scale, GRID_SCALE_MIN))
    , map_scale_inv_(1.0f / std::max(grid_scale, GRID_SCALE_MIN))
    , buffer_scale_(std::max(0.0f, buffer_scale))
    , origin_(world.getAabbMin())
    , length_((world.getAabbMax() - world.getAabbMin()).array().abs())
{
    createMap(world);
    createFreeRegion(points);
}

float Map::getGridScale() const
{
    return map_scale_;
}

float Map::getGridScaleInv() const
{
    return map_scale_inv_;
}

float Map::getBufferScale() const
{
    return buffer_scale_;
}

Eigen::Vector2f Map::getOrigin() const
{
    return origin_;
}

Eigen::Vector2f Map::getLength() const
{
    return length_;
}

size_t Map::size() const
{
    return map_.size();
}

size_t Map::sizeX() const
{
    return size_x_;
}

size_t Map::sizeY() const
{
    return size_y_;
}

const uint8_t& Map::operator()(const Eigen::Vector2i p) const
{
    assert(isInMap(p));
    return map_[static_cast<size_t>(p.x()) + static_cast<size_t>(p.y()) * size_x_];
}

uint8_t& Map::operator()(const Eigen::Vector2i p)
{
    assert(isInMap(p));
    return map_[static_cast<size_t>(p.x()) + static_cast<size_t>(p.y()) * size_x_];
}

const uint8_t& Map::operator()(const size_t x, const size_t y) const
{
    assert(isInMap(x, y));
    return map_[x + y * size_x_];
}

uint8_t& Map::operator()(const size_t x, const size_t y)
{
    assert(isInMap(x, y));
    return map_[x + y * size_x_];
}

const uint8_t& Map::operator[](const size_t index) const
{
    assert(index < map_.size());
    return map_[index];
}

uint8_t& Map::operator[](const size_t index)
{
    assert(index < map_.size());
    return map_[index];
}

void Map::fromWorldIntoMap(const float wx, const float wy, float &mx, float &my) const
{
    const Eigen::Vector2f p = (Eigen::Vector2f(wx, wy) - origin_) * map_scale_inv_;

    mx = p.x();
    my = (static_cast<float>(size_y_) - 1.0f) - p.y();
}

void Map::fromWorldIntoMap(const Eigen::Vector2f wp, float &mx, float &my) const
{
    const Eigen::Vector2f p = (wp - origin_) * map_scale_inv_;

    mx = p.x();
    my = (static_cast<float>(size_y_) - 1.0f) - p.y();
}

void Map::fromWorldIntoMap(const float wx, const float wy, Eigen::Vector2f &mp) const
{
    mp = (Eigen::Vector2f(wx, wy) - origin_) * map_scale_inv_;
    mp.y() = (static_cast<float>(size_y_) - 1.0f) - mp.y();
}

void Map::fromWorldIntoMap(const Eigen::Vector2f wp, Eigen::Vector2f &mp) const
{
    mp = (wp - origin_) * map_scale_inv_;
    mp.y() = (static_cast<float>(size_y_) - 1.0f) - mp.y();
}

void Map::fromWorldIntoMap(const float wx, const float wy, int &mx, int &my) const
{
    const Eigen::Vector2f p = ((Eigen::Vector2f(wx, wy) - origin_) * map_scale_inv_).array().round();

    mx = static_cast<int>(p.x());
    my = (static_cast<int>(size_y_) - 1) - static_cast<int>(p.y());
}

void Map::fromWorldIntoMap(const Eigen::Vector2f wp, int &mx, int &my) const
{
    const Eigen::Vector2f p = ((wp - origin_) * map_scale_inv_).array().round();

    mx = static_cast<int>(p.x());
    my = (static_cast<int>(size_y_) - 1) - static_cast<int>(p.y());
}

void Map::fromWorldIntoMap(const float wx, const float wy, Eigen::Vector2i &mp) const
{
    const Eigen::Vector2f p = ((Eigen::Vector2f(wx, wy) - origin_) * map_scale_inv_).array().round();

    mp.x() = static_cast<int>(p.x());
    mp.y() = (static_cast<int>(size_y_) - 1) - static_cast<int>(p.y());
}

void Map::fromWorldIntoMap(const Eigen::Vector2f wp, Eigen::Vector2i &mp) const
{
    const Eigen::Vector2f p = ((wp - origin_) * map_scale_inv_).array().round();

    mp.x() = static_cast<int>(p.x());
    mp.y() = (static_cast<int>(size_y_) - 1) - static_cast<int>(p.y());
}

void Map::fromMapIntoWorld(const int mx, const int my, float &wx, float &wy) const
{
    const Eigen::Vector2f wp = Eigen::Vector2f(static_cast<float>(mx), (static_cast<float>(size_y_) - 1.0f) - static_cast<float>(my)) * map_scale_ + origin_;

    wx = wp.x();
    wy = wp.y();
}

void Map::fromMapIntoWorld(const Eigen::Vector2i mp, float &wx, float &wy) const
{
    const Eigen::Vector2f wp = Eigen::Vector2f(static_cast<float>(mp.x()), (static_cast<float>(size_y_) - 1.0f) - static_cast<float>(mp.y())) * map_scale_ + origin_;
    wx = wp.x();
    wy = wp.y();
}

void Map::fromMapIntoWorld(const int x, const int y, Eigen::Vector2f &wp) const
{
    wp.x() = static_cast<float>(x);
    wp.y() = (static_cast<float>(size_y_) - 1.0f) - static_cast<float>(y);
    wp = wp * map_scale_ + origin_;
}

void Map::fromMapIntoWorld(const Eigen::Vector2i mp, Eigen::Vector2f &wp) const
{
    wp.x() = static_cast<float>(mp.x());
    wp.y() = (static_cast<float>(size_y_) - 1.0f) - static_cast<float>(mp.y());
    wp = wp * map_scale_ + origin_;
}

bool Map::isInMap(const size_t x, const size_t y) const
{
    return (x < size_x_ && y < size_y_);
}

bool Map::isInMap(const int x, const int y) const
{
    return (0 <= x && x < static_cast<int>(size_x_)) && (0 <= y && y < static_cast<int>(size_y_));
}

bool Map::isInMap(const Eigen::Vector2i point) const
{
    return (0 <= point.x() && point.x() < static_cast<int>(size_x_)) && (0 <= point.y() && point.y() < static_cast<int>(size_y_));
}

void Map::createMap(const murosim::geometry::World &world)
{
    Eigen::Vector2f l = length_ * map_scale_inv_;

    // Size to the rounded extent: fromWorldIntoMap() rounds world coordinates,
    // so the AABB-max corner maps to round(l). Using floor(l)+1 here would leave
    // that cell out of range and silently clip the far-edge obstacle pixels.
    size_x_ = static_cast<size_t>(std::lround(l.x())) + 1;
    size_y_ = static_cast<size_t>(std::lround(l.y())) + 1;

    map_.resize(size_x_ * size_y_, FIELD::FREE);

    for (size_t i = 0; i < world.getLineSize(); i++)
    {
        drawing::line(*this, FIELD::OCCUPIED, world.getLineAt(i));
    }

    for (size_t i = 0; i < world.getCircleSize(); i++)
    {
        drawing::circle(*this, FIELD::OCCUPIED, world.getCircleAt(i));
    }
}

void Map::createFreeRegion()
{
    std::vector<std::vector<size_t>> regions;

    // Iterate through the whole map and fill the FREE regions with UNREACHABLE
    for (size_t i = 0; i < map_.size(); i++)
    {
        if (isFree(i))
        {
            regions.emplace_back(std::vector<size_t>());
            drawing::fill(*this, FIELD::FREE, FIELD::UNREACHABLE, indexToPoint(i), regions.back());
        }
    }

    // If at least one region is found then take the biggest for the FREE region
    if (!regions.empty())
    {
        auto largest = findLargestRegion(regions);

        for (const size_t i : *largest)
        {
            map_[i] = FIELD::FREE;
        }

        applyBufferSpace();
    }
}

std::vector<std::vector<size_t>> Map::fillRegionsFromNodePoints(const std::vector<Eigen::Vector2f> &node_points)
{
    std::vector<std::vector<size_t>> regions;

    // flood-fill the FREE region around each node point with UNREACHABLE and
    // record its indices as one region
    for (const auto &node_point : node_points)
    {
        Eigen::Vector2i point;
        fromWorldIntoMap(node_point, point);

        if (isInMap(point) && isFree(point))
        {
            regions.emplace_back(std::vector<size_t>());
            drawing::fill(*this, FIELD::FREE, FIELD::UNREACHABLE, point, regions.back());
        }
    }

    return regions;
}

void Map::createFreeRegion(const std::vector<Eigen::Vector2f> &node_points)
{
    if (node_points.empty())
    {
        createFreeRegion();
        return;
    }

    std::vector<std::vector<size_t>> regions = fillRegionsFromNodePoints(node_points);

    // If no regions could be created through node points then go to the fallback function
    if (regions.empty())
    {
        createFreeRegion();
        return;
    }

    // Fill all remaining FREE space with UNREACHABLE
    for (auto &cell : map_)
    {
        if (cell == FIELD::FREE)
        {
            cell = FIELD::UNREACHABLE;
        }
    }

    // Fill back the FREE spaces from the regions
    for (const auto &region : regions)
    {
        for (const size_t index : region)
        {
            map_[index] = FIELD::FREE;
        }
    }

    applyBufferSpace();

    // Create the regions from the points again
    regions = fillRegionsFromNodePoints(node_points);

    // Fill all remaining FREE space with BUFFER
    for (auto &cell : map_)
    {
        if (cell == FIELD::FREE)
        {
            cell = FIELD::BUFFER;
        }
    }

    if (regions.empty())
    {
        throw std::runtime_error("No FREE region could be created through node points. Is the buffer scale too high?");
    }

    // Get the largest region
    auto largest = findLargestRegion(regions);
    size_t largest_index = static_cast<size_t>(std::distance(regions.cbegin(), largest));

    // Fill the largest region with FREE space, the others with BUFFER space
    for (size_t i = 0; i < regions.size(); i++)
    {
        const FIELD symbol = (i == largest_index) ? FIELD::FREE : FIELD::BUFFER;

        for (const size_t index : regions[i])
        {
            map_[index] = symbol;
        }
    }
}

std::vector<Eigen::Vector2i> Map::createBufferCircle() const
{
    std::vector<Eigen::Vector2i> points;

    const int begin = static_cast<int>(std::floor(-buffer_scale_ * getGridScaleInv()));
    const int end = static_cast<int>(std::ceil(buffer_scale_ * getGridScaleInv()));
    const float buffer_scale_squared = buffer_scale_ * buffer_scale_;

    Eigen::Vector2i pixel(begin, begin);

    for (pixel.x() = begin; pixel.x() <= end; pixel.x()++)
    {
        for (pixel.y() = begin; pixel.y() <= end; pixel.y()++)
        {
            const Eigen::Vector2f point = pixel.cast<float>() * getGridScale();

            if (point.squaredNorm() <= buffer_scale_squared)
            {
                points.emplace_back(pixel);
            }
        }
    }

    return points;
}

void Map::applyBufferSpace()
{
    if (map_scale_ <= buffer_scale_)
    {
        std::vector<Eigen::Vector2i> buffer_points = createBufferCircle();

        for (size_t i = 0; i < map_.size(); i++)
        {
            if (map_[i] == FIELD::OCCUPIED)
            {
                const Eigen::Vector2i pixel = indexToPoint(i);

                for (const auto &buffer_point : buffer_points)
                {
                    const Eigen::Vector2i p = pixel + buffer_point;

                    if (isInMap(p) && isFree(p))
                    {
                        (*this)(p) = FIELD::BUFFER;
                    }
                }
            }
        }
    }
}

bool Map::isObstacle(const int x, const int y) const
{
    assert(isInMap(x, y));
    const uint8_t &cell = (*this)(static_cast<size_t>(x), static_cast<size_t>(y));
    return (cell == FIELD::OCCUPIED || cell == FIELD::UNREACHABLE);
}

bool Map::isObstacle(const size_t x, const size_t y) const
{
    assert(isInMap(x, y));
    const uint8_t &cell = (*this)(x, y);
    return (cell == FIELD::OCCUPIED || cell == FIELD::UNREACHABLE);
}

bool Map::isObstacle(const size_t index) const
{
    assert(index < map_.size());
    const uint8_t &cell = map_[index];
    return (cell == FIELD::OCCUPIED || cell == FIELD::UNREACHABLE);
}

bool Map::isObstacle(const Eigen::Vector2i point) const
{
    assert(isInMap(point));
    return isObstacle(point.x(), point.y());
}

size_t Map::pointToIndex(const int x, const int y) const
{
    assert(isInMap(x, y));
    return static_cast<size_t>(x) + static_cast<size_t>(y) * size_x_;
}

size_t Map::pointToIndex(const Eigen::Vector2i point) const
{
    assert(isInMap(point));
    return static_cast<size_t>(point.x()) + static_cast<size_t>(point.y()) * size_x_;
}

Eigen::Vector2i Map::indexToPoint(const size_t index) const
{
    assert(index < map_.size());
    return Eigen::Vector2i{static_cast<int>(index % size_x_), static_cast<int>(index / size_x_)};
}

void Map::indexToPoint(const size_t index, int &x, int &y) const
{
    assert(index < map_.size());
    x = static_cast<int>(index % size_x_);
    y = static_cast<int>(index / size_x_);
}

bool Map::isFree(const size_t index) const
{
    assert(index < map_.size());
    return map_[index] == FIELD::FREE;
}

bool Map::isFree(const int x, const int y) const
{
    assert(isInMap(x, y));
    return (*this)(static_cast<size_t>(x), static_cast<size_t>(y)) == FIELD::FREE;
}

bool Map::isFree(const size_t x, const size_t y) const
{
    assert(isInMap(x, y));
    return (*this)(x, y) == FIELD::FREE;
}

bool Map::isFree(const Eigen::Vector2i point) const
{
    assert(isInMap(point));
    return (*this)(static_cast<size_t>(point.x()), static_cast<size_t>(point.y())) == FIELD::FREE;
}

bool Map::isBuffer(const size_t index) const
{
    assert(index < map_.size());
    return map_[index] == FIELD::BUFFER;
}

bool Map::isBuffer(const int x, const int y) const
{
    assert(isInMap(x, y));
    return (*this)(static_cast<size_t>(x), static_cast<size_t>(y)) == FIELD::BUFFER;
}

bool Map::isBuffer(const size_t x, const size_t y) const
{
    assert(isInMap(x, y));
    return (*this)(x, y) == FIELD::BUFFER;
}

bool Map::isBuffer(const Eigen::Vector2i point) const
{
    assert(isInMap(point));
    return (*this)(static_cast<size_t>(point.x()), static_cast<size_t>(point.y())) == FIELD::BUFFER;
}

void Map::set(const uint8_t symbol, const int x, const int y)
{
    if (isInMap(x, y))
    {
        (*this)(static_cast<size_t>(x), static_cast<size_t>(y)) = symbol;
    }
}

void Map::set(const uint8_t symbol, const Eigen::Vector2i point)
{
    if (isInMap(point))
    {
        (*this)(point) = symbol;
    }
}

std::ostream& operator<<(std::ostream &os, const Map &map)
{
    constexpr unsigned char lut[] = {' ', 'X', '*', '#'};

    // draw frame like +----------+
    os << "+";
    for (size_t i = 0; i < map.sizeX(); ++i)
    {
        os << "-";
    }
    os << "+" << '\n';

    // draw frame like |0000100101|
    for (size_t y = 0; y < map.sizeY(); ++y)
    {
        os << "|";
        for (size_t x = 0; x < map.sizeX(); ++x)
        {
            const uint8_t cell = map(x, y);
            os << (cell < sizeof(lut) ? lut[cell] : '?');
        }
        os << "|" << '\n';
    }

    // draw frame like +----------+
    os << "+";
    for (size_t i = 0; i < map.sizeX(); ++i)
    {
        os << "-";
    }
    os << "+" << '\n';

    return os;
}

} // namespace murosim::grid
