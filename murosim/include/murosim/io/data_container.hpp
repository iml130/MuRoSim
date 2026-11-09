// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_IO_DATA_CONTAINER_HPP_
#define MUROSIM_IO_DATA_CONTAINER_HPP_

#include "murosim/geometry/world.hpp"
#include "murosim/navigation/container.hpp"

#include <string_view>
#include <utility>

namespace murosim::io
{

/// Default name used when none is given or loaded (see world::loadWorldFile, which overrides it with the file stem).
inline constexpr std::string_view DEFAULT_NAME = "UNSET";

/// Plain-data holder for a world description: obstacle lines/circles plus
/// navigation nodes, and the name it was loaded/created with. It is the
/// serialization-neutral form produced by io::world::loadWorldFile() and
/// consumed by Simulation's constructors; createWorld()/createShape()/
/// createNodeContainer() convert it into the runtime geometry/navigation
/// types actually used by the simulator.
class DataContainer
{
private:
    std::string name_;
    std::vector<murosim::geometry::Line> lines_;
    std::vector<murosim::geometry::Circle> circles_;
    std::vector<murosim::navigation::Node> nodes_;
public:
    /// Copies lines, circles and nodes; name_ defaults to DEFAULT_NAME.
    DataContainer(const std::vector<murosim::geometry::Line> &lines, const std::vector<murosim::geometry::Circle> &circles, const std::vector<murosim::navigation::Node> &nodes, const std::string &name = std::string(DEFAULT_NAME))
        : name_(name), lines_(lines), circles_(circles), nodes_(nodes) {}
    /// Move-constructs from lines, circles, nodes and name.
    DataContainer(std::vector<murosim::geometry::Line> &&lines, std::vector<murosim::geometry::Circle> &&circles, std::vector<murosim::navigation::Node> &&nodes, std::string &&name = std::string(DEFAULT_NAME))
        : name_(std::move(name)), lines_(std::move(lines)), circles_(std::move(circles)), nodes_(std::move(nodes)) {}
    /// Builds from a composite Shape (decomposed into lines/circles via Shape::copyTo) and a node list.
    DataContainer(const murosim::geometry::Shape &shape, const std::vector<murosim::navigation::Node> &nodes, const std::string &name = std::string(DEFAULT_NAME))
        : name_(name), lines_(), circles_(), nodes_(nodes) { shape.copyTo(lines_, circles_); }
    // Valid if it contains at least one line or circle; nodes are optional.
    /// @return true if it contains at least one line or circle (nodes alone do not count).
    inline bool isValid() const { return !(lines_.empty() && circles_.empty()); }
    inline const std::vector<murosim::geometry::Line>& getLines() const { return lines_; }
    inline const std::vector<murosim::geometry::Circle>& getCircles() const { return circles_; }
    inline const std::vector<murosim::navigation::Node>& getNodes() const { return nodes_; }
    inline const std::string& getName() const { return name_; }
    /// Wraps the node list in a navigation::Container (builds group indices).
    inline murosim::navigation::Container createNodeContainer() const { return murosim::navigation::Container(nodes_); }
    /// Builds the runtime World geometry from the stored lines and circles.
    inline murosim::geometry::World createWorld() const { return murosim::geometry::World(lines_, circles_); }
    /// Builds a composite Shape from the stored lines and circles.
    inline murosim::geometry::Shape createShape() const { return murosim::geometry::Shape(lines_, circles_); }
    /// Returns a copy scaled uniformly by scale_factor; see scale(Eigen::Vector2f).
    inline murosim::io::DataContainer scale(const float scale_factor) const { return scale(Eigen::Vector2f(scale_factor, scale_factor)); }
    /// Returns a copy scaled by independent x/y factors; see scale(Eigen::Vector2f).
    inline murosim::io::DataContainer scale(const float x, const float y) const { return scale(Eigen::Vector2f(x, y)); }
    /// Returns a copy with every line, circle and node scaled by scale_factor (per-axis); name is preserved.
    inline murosim::io::DataContainer scale(const Eigen::Vector2f &scale_factor) const
    {
        std::vector<murosim::geometry::Line> lines(lines_);
        std::vector<murosim::geometry::Circle> circles(circles_);
        std::vector<murosim::navigation::Node> nodes(nodes_);

        for (murosim::geometry::Line &line : lines) line.scale(scale_factor);
        for (murosim::geometry::Circle &circle : circles) circle.scale(scale_factor);
        for (murosim::navigation::Node &node : nodes) node.scale(scale_factor);

        return murosim::io::DataContainer(std::move(lines), std::move(circles), std::move(nodes), std::string(name_));
    }
};

} // namespace murosim::io

#endif // MUROSIM_IO_DATA_CONTAINER_HPP_
