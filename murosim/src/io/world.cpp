// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/io/world.hpp"
#include "murosim/io/util.hpp"
#include <numbers>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace
{

constexpr float DEG_TO_RAD = static_cast<float>(std::numbers::pi / 180.0);
constexpr float RAD_TO_DEG = static_cast<float>(180.0 / std::numbers::pi);
constexpr std::string_view HEADER_NODES = "# nodes -> n x y r start_angle start_bias id group";
constexpr std::string_view HEADER_LINES = "# lines -> l x1 y1 x2 y2";
constexpr std::string_view HEADER_CIRCLES = "# circles -> c x y r";

bool isValidFloat(float value)
{
    return !std::isnan(value) && !std::isinf(value);
}

// max_digits10 guarantees a lossless float round-trip; a fixed decimal
// precision would quantize coordinates, which can collapse short lines into
// "zero length" entries the loader rejects.
void setLosslessFloatFormat(std::ofstream &file)
{
    file << std::setprecision(std::numeric_limits<float>::max_digits10);
}

bool hasExtraData(std::istringstream& iss)
{
    std::string extra;
    return static_cast<bool>(iss >> extra);
}

void writeNode(std::ofstream &file, const murosim::navigation::Node &node)
{
    setLosslessFloatFormat(file);
    file << "n " << node.getX() << " " << node.getY() << " " << node.getRadius() << " " << node.getStartAngle() * RAD_TO_DEG << " " << node.getStartBias() * RAD_TO_DEG << " " << node.getId() << " " << node.getGroup() << "\n";
}

void writeLine(std::ofstream &file, const murosim::geometry::Line &line)
{
    setLosslessFloatFormat(file);
    file << "l " << line.getPoint1X() << " " << line.getPoint1Y() << " " << line.getPoint2X() << " " << line.getPoint2Y() << "\n";
}

void writeCircle(std::ofstream &file, const murosim::geometry::Circle &circle)
{
    setLosslessFloatFormat(file);
    file << "c " << circle.getX() << " " << circle.getY() << " " << circle.getRadius() << "\n";
}

// Resolves the output path (forcing a .world extension) and opens it for writing.
std::ofstream openWorldFileForWrite(const std::filesystem::path &filename)
{
    std::filesystem::path output_path = filename;
    if (!murosim::io::world::isWorldFile(output_path)) {
        output_path.replace_extension(".world");
    }

    std::ofstream file(output_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not create world file '" + output_path.string() + "'!");
    }
    return file;
}

} // anonymous namespace

namespace murosim::io::world
{

murosim::io::DataContainer loadWorldFile(const std::filesystem::path &filename)
{
    if (!murosim::io::world::isWorldFile(filename))
    {
        throw std::runtime_error(std::string("The file '") + filename.string() + std::string("' is not a world file. File must end with '.world'."));
    }

    std::ifstream file;

    file.open(filename);
    if (!file.is_open())
    {
        throw std::runtime_error(std::string("Could not open world file '") + filename.string() + std::string("'!"));
    }

    std::vector<murosim::geometry::Line> lines;
    std::vector<murosim::geometry::Circle> circles;
    std::vector<murosim::navigation::Node> nodes;

    std::stringstream ss;
    bool error = false;
    size_t line_counter = 0;
    std::string line;

    while (std::getline(file, line))
    {
        ++line_counter;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string type;
        if (!(iss >> type)) continue; // skip whitespace-only lines

        if (type[0] == '#') // skip comments
        {
            continue;
        }
        else if (type == "c")
        {
            float x, y, r;
            if (iss >> x >> y >> r)
            {
                if (!isValidFloat(x) || !isValidFloat(y) || !isValidFloat(r))
                {
                    ss << "  [ERROR] Line " << line_counter << ": circle contains invalid float value (NaN or Inf)\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (r <= 0.0f)
                {
                    ss << "  [ERROR] Line " << line_counter << ": circle radius must be positive, got " << r << "\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (hasExtraData(iss))
                {
                    ss << "  [ERROR] Line " << line_counter << ": circle has unexpected extra data\n";
                    ss << "         -> \"" << line << "\"\n";
                    ss << "         Expected format: c x y r\n";
                    error = true;
                }
                else
                {
                    circles.emplace_back(x, y, r);
                }
            }
            else
            {
                ss << "  [ERROR] Line " << line_counter << ": failed to parse circle\n";
                ss << "         -> \"" << line << "\"\n";
                ss << "         Expected format: c x y r\n";
                error = true;
            }
        }
        else if (type == "l")
        {
            float x1, y1, x2, y2;

            if (iss >> x1 >> y1 >> x2 >> y2)
            {
                if (!isValidFloat(x1) || !isValidFloat(y1) || !isValidFloat(x2) || !isValidFloat(y2))
                {
                    ss << "  [ERROR] Line " << line_counter << ": line contains invalid float value (NaN or Inf)\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (x1 == x2 && y1 == y2)
                {
                    ss << "  [ERROR] Line " << line_counter << ": line has zero length (point1 == point2)\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (hasExtraData(iss))
                {
                    ss << "  [ERROR] Line " << line_counter << ": line has unexpected extra data\n";
                    ss << "         -> \"" << line << "\"\n";
                    ss << "         Expected format: l x1 y1 x2 y2\n";
                    error = true;
                }
                else
                {
                    lines.emplace_back(x1, y1, x2, y2);
                }
            }
            else
            {
                ss << "  [ERROR] Line " << line_counter << ": failed to parse line\n";
                ss << "         -> \"" << line << "\"\n";
                ss << "         Expected format: l x1 y1 x2 y2\n";
                error = true;
            }
        }
        else if (type == "n")
        {
            float x, y, r, start_angle, start_bias;
            int id, group;
            if (iss >> x >> y >> r >> start_angle >> start_bias >> id >> group)
            {
                if (!isValidFloat(x) || !isValidFloat(y) || !isValidFloat(r) ||
                    !isValidFloat(start_angle) || !isValidFloat(start_bias))
                {
                    ss << "  [ERROR] Line " << line_counter << ": node contains invalid float value (NaN or Inf)\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (id < 0)
                {
                    ss << "  [ERROR] Line " << line_counter << ": node id must be non-negative, got " << id << "\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (group < 0)
                {
                    ss << "  [ERROR] Line " << line_counter << ": node group must be non-negative, got " << group << "\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (r <= 0.0f)
                {
                    ss << "  [ERROR] Line " << line_counter << ": node radius must be positive, got " << r << "\n";
                    ss << "         -> \"" << line << "\"\n";
                    error = true;
                }
                else if (hasExtraData(iss))
                {
                    ss << "  [ERROR] Line " << line_counter << ": node has unexpected extra data\n";
                    ss << "         -> \"" << line << "\"\n";
                    ss << "         Expected format: n x y r start_angle start_bias id group\n";
                    error = true;
                }
                else
                {
                    nodes.emplace_back(murosim::geometry::Circle(x, y, r),
                                        start_angle * DEG_TO_RAD,
                                        start_bias * DEG_TO_RAD,
                                        static_cast<size_t>(id), static_cast<size_t>(group));
                }
            }
            else
            {
                ss << "  [ERROR] Line " << line_counter << ": failed to parse node\n";
                ss << "         -> \"" << line << "\"\n";
                ss << "         Expected format: n x y r start_angle start_bias id group\n";
                error = true;
            }
        }
        else
        {
            ss << "  [ERROR] Line " << line_counter << ": unknown type '" << type << "'\n";
            ss << "         -> \"" << line << "\"\n";
            ss << "         Valid types: 'c' (circle), 'l' (line), 'n' (node)\n";
            error = true;
        }
    }

    if (error)
    {
        throw std::runtime_error("Failed to parse world file '" + filename.string() + "':\n" + ss.str());
    }

    if (lines.empty() && circles.empty() && nodes.empty())
    {
        throw std::runtime_error("World file '" + filename.string() + "' contains no elements (no lines, circles, or nodes).");
    }

    return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), murosim::io::util::getFilename(filename));
}

void saveWorldFile(const std::filesystem::path &filename, const murosim::geometry::World &world, const murosim::navigation::Container &nodes)
{
    // World exposes only indexed access, so materialize vectors and delegate.
    std::vector<murosim::geometry::Line> lines;
    lines.reserve(world.getLineSize());
    for (size_t i = 0; i < world.getLineSize(); ++i) lines.push_back(world.getLineAt(i));

    std::vector<murosim::geometry::Circle> circles;
    circles.reserve(world.getCircleSize());
    for (size_t i = 0; i < world.getCircleSize(); ++i) circles.push_back(world.getCircleAt(i));

    saveWorldFile(filename, lines, circles, nodes.getNodes());
}

void saveWorldFile(const std::filesystem::path &filename, const murosim::geometry::Shape &shape, const murosim::navigation::Container &nodes)
{
    saveWorldFile(filename, shape.getLines(), shape.getCircles(), nodes.getNodes());
}

void saveWorldFile(const std::filesystem::path &filename, const std::vector<murosim::geometry::Line> &lines, const std::vector<murosim::geometry::Circle> &circles, const std::vector<murosim::navigation::Node> &nodes)
{
    // Core writer: all other overloads funnel here, so the file format lives in one place.
    std::ofstream file = openWorldFileForWrite(filename);

    file << HEADER_NODES << '\n';
    for (const murosim::navigation::Node &node : nodes)
    {
        writeNode(file, node);
    }
    file << '\n';

    file << HEADER_LINES << '\n';
    for (const murosim::geometry::Line &line : lines)
    {
        writeLine(file, line);
    }
    file << '\n';

    file << HEADER_CIRCLES << '\n';
    for (const murosim::geometry::Circle &circle : circles)
    {
        writeCircle(file, circle);
    }
    file << '\n';
}

void saveWorldFile(const std::filesystem::path &filename, const murosim::io::DataContainer &data)
{
    saveWorldFile(filename, data.getLines(), data.getCircles(), data.getNodes());
}

bool isWorldFile(const std::filesystem::path &path)
{
    return path.extension() == ".world";
}

} // namespace murosim::io::world
