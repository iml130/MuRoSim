// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/io/world.hpp"
#include "murosim/io/util.hpp"

#include <sstream>

namespace murosimpy {
namespace {

// Helper to collect lines from DataContainer (returns copy of internal vector)
std::vector<murosim::geometry::Line> collectLines(const murosim::io::DataContainer &c) {
    return c.getLines();
}

// Helper to collect circles from DataContainer (returns copy of internal vector)
std::vector<murosim::geometry::Circle> collectCircles(const murosim::io::DataContainer &c) {
    return c.getCircles();
}

// Helper to collect nodes from DataContainer (returns copy of internal vector)
std::vector<murosim::navigation::Node> collectNodes(const murosim::io::DataContainer &c) {
    return c.getNodes();
}

} // anonymous namespace

void initIODataContainer(pybind11::module &m)
{
    auto pyDataContainer = pybind11::class_<murosim::io::DataContainer>(m, "DataContainer",
        R"doc(Plain-data holder for a world description: obstacle lines/circles plus
navigation nodes, and the name it was loaded/created with.

This is the serialization-neutral form produced by load_world() and
consumed to build a Simulation. Use create_world()/create_shape()/
create_node_container() to convert it into the runtime geometry/navigation
types used by the simulator.)doc");

    // Constructors
    pyDataContainer.def(pybind11::init<const std::vector<murosim::geometry::Line>&,
                                        const std::vector<murosim::geometry::Circle>&,
                                        const std::vector<murosim::navigation::Node>&,
                                        const std::string&>(),
        "Create a DataContainer from lists of lines, circles, and nodes.\n\n"
        "Args:\n"
        "    lines: List of Line objects\n"
        "    circles: List of Circle objects\n"
        "    nodes: List of Node objects\n"
        "    name: Container name",
        pybind11::arg("lines"), pybind11::arg("circles"), pybind11::arg("nodes"), pybind11::arg("name"));
    pyDataContainer.def(pybind11::init<const murosim::geometry::Shape&,
                                        const std::vector<murosim::navigation::Node>&,
                                        const std::string&>(),
        "Create a DataContainer from a Shape and list of nodes.\n\n"
        "Args:\n"
        "    shape: Shape containing lines and circles\n"
        "    nodes: List of Node objects\n"
        "    name: Container name",
        pybind11::arg("shape"), pybind11::arg("nodes"), pybind11::arg("name"));

    // Factory methods
    pyDataContainer.def("create_node_container", &murosim::io::DataContainer::createNodeContainer,
        "Wrap the stored node list in a NodeContainer (sorts by id and builds group indices).");
    pyDataContainer.def("create_world", &murosim::io::DataContainer::createWorld,
        "Build the runtime World geometry from the stored lines and circles.");
    pyDataContainer.def("create_shape", &murosim::io::DataContainer::createShape,
        "Build a composite Shape from the stored lines and circles.");

    // Properties
    pyDataContainer.def_property_readonly("name", &murosim::io::DataContainer::getName,
        "Container name identifier.");
    pyDataContainer.def_property_readonly("line_count", [](const murosim::io::DataContainer &c) {
        return c.getLines().size();
    }, "Number of lines in the container.");
    pyDataContainer.def_property_readonly("circle_count", [](const murosim::io::DataContainer &c) {
        return c.getCircles().size();
    }, "Number of circles in the container.");
    pyDataContainer.def_property_readonly("node_count", [](const murosim::io::DataContainer &c) {
        return c.getNodes().size();
    }, "Number of nodes in the container.");
    pyDataContainer.def_property_readonly("lines", &collectLines,
        "List of all lines in the container.");
    pyDataContainer.def_property_readonly("circles", &collectCircles,
        "List of all circles in the container.");
    pyDataContainer.def_property_readonly("nodes", &collectNodes,
        "List of all nodes in the container.");

    // Indexed access
    pyDataContainer.def("line_at", [](const murosim::io::DataContainer &c, size_t index) {
        const auto& lines = c.getLines();
        checkIndex(index, lines.size(), "Line");
        return lines[index];
    }, "Get a copy of the line at the specified index.\n\n"
       "Args:\n"
       "    index: Zero-based index into the container's lines.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));
    pyDataContainer.def("circle_at", [](const murosim::io::DataContainer &c, size_t index) {
        const auto& circles = c.getCircles();
        checkIndex(index, circles.size(), "Circle");
        return circles[index];
    }, "Get a copy of the circle at the specified index.\n\n"
       "Args:\n"
       "    index: Zero-based index into the container's circles.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));
    pyDataContainer.def("node_at", [](const murosim::io::DataContainer &c, size_t index) {
        const auto& nodes = c.getNodes();
        checkIndex(index, nodes.size(), "Node");
        return nodes[index];
    }, "Get a copy of the node at the specified index.\n\n"
       "Args:\n"
       "    index: Zero-based index into the container's nodes.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));

    // Python special methods
    pyDataContainer.def("__repr__", [](const murosim::io::DataContainer &c) {
        std::ostringstream oss;
        oss << "<DataContainer '" << c.getName() << "' lines=" << c.getLines().size()
            << " circles=" << c.getCircles().size() << " nodes=" << c.getNodes().size() << ">";
        return oss.str();
    }, "Return a debug string representation.");
}

} // namespace murosimpy
