// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/geometry/shape.hpp"

#include <sstream>

namespace murosimpy {
namespace {

// Helper to collect lines from a Shape (returns copy of internal vector)
std::vector<murosim::geometry::Line> collectLines(const murosim::geometry::Shape &shape) {
    return shape.getLines();
}

// Helper to collect circles from a Shape (returns copy of internal vector)
std::vector<murosim::geometry::Circle> collectCircles(const murosim::geometry::Shape &shape) {
    return shape.getCircles();
}

} // anonymous namespace

void initGeometryShape(pybind11::module &m)
{
    auto pyShape  = pybind11::class_<murosim::geometry::Shape>(m, "Shape",
        R"doc(A composite 2D shape made of line segments and circles.

Used to describe robot/obstacle footprints (e.g. built from the
triangle/rectangle/square primitives or assembled piece by piece) that are
later transformed into the world frame and turned into a World.)doc");

    // === Shape ===
    pyShape.def(pybind11::init(),
        "Create an empty shape.");
    // Properties
    pyShape.def_property_readonly("line_count", [](const murosim::geometry::Shape &shape) {
        return shape.getLines().size();
    }, "Number of lines in this shape.");
    pyShape.def_property_readonly("circle_count", [](const murosim::geometry::Shape &shape) {
        return shape.getCircles().size();
    }, "Number of circles in this shape.");
    pyShape.def_property_readonly("lines", &collectLines,
        "List of all lines in this shape.");
    pyShape.def_property_readonly("circles", &collectCircles,
        "List of all circles in this shape.");
    // Add elements
    pyShape.def("add_line", pybind11::overload_cast<const murosim::geometry::Line&>(&murosim::geometry::Shape::addLine),
        "Add a line to this shape.",
        pybind11::arg("line"));
    pyShape.def("add_line", pybind11::overload_cast<const float, const float, const float, const float>(&murosim::geometry::Shape::addLine),
        "Add a line from (x1, y1) to (x2, y2), in metres.",
        pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"));
    pyShape.def("add_circle", pybind11::overload_cast<const murosim::geometry::Circle&>(&murosim::geometry::Shape::addCircle),
        "Add a circle to this shape.",
        pybind11::arg("circle"));
    pyShape.def("add_circle", pybind11::overload_cast<const float, const float, const float>(&murosim::geometry::Shape::addCircle),
        "Add a circle at (x, y) with given radius, in metres.",
        pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"));
    pyShape.def("add_shape", pybind11::overload_cast<const murosim::geometry::Shape &>(&murosim::geometry::Shape::addShape),
        "Add all elements from another shape.",
        pybind11::arg("shape"));
    // Transform methods
    pyShape.def("translate", pybind11::overload_cast<const float, const float>(&murosim::geometry::Shape::translate),
        "Translate every line and circle by (dx, dy), in metres.",
        pybind11::arg("dx"), pybind11::arg("dy"));
    pyShape.def("rotate", pybind11::overload_cast<const float>(&murosim::geometry::Shape::rotate),
        "Rotate every line and circle about the origin by angle radians.",
        pybind11::arg("angle"));
    pyShape.def("transform", pybind11::overload_cast<const float, const float, const float>(&murosim::geometry::Shape::transform),
        "Place every line and circle at pose (x, y, angle): rotate about the origin, then translate (T * R). "
        "x and y are in metres, angle in radians.",
        pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle"));
    pyShape.def("transform", pybind11::overload_cast<const Eigen::Affine2f &>(&murosim::geometry::Shape::transform),
        "Apply an affine transform to every line and circle.",
        pybind11::arg("transform"));
    pyShape.def("scale", pybind11::overload_cast<const float>(&murosim::geometry::Shape::scale),
        "Uniformly scale every line and circle about the origin by factor.",
        pybind11::arg("factor"));
    pyShape.def(Eigen::Affine2f() * pybind11::self,
        "Return a new shape with the affine transform applied to every line and circle (transform * shape).");
    // Indexed access
    pyShape.def("line_at", [](const murosim::geometry::Shape &shape, size_t index) {
        const auto& lines = shape.getLines();
        checkIndex(index, lines.size(), "Line");
        return lines[index];
    }, "Get a copy of the line at the specified index.\n\n"
       "Args:\n"
       "    index: Zero-based index into the shape's lines.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));
    pyShape.def("circle_at", [](const murosim::geometry::Shape &shape, size_t index) {
        const auto& circles = shape.getCircles();
        checkIndex(index, circles.size(), "Circle");
        return circles[index];
    }, "Get a copy of the circle at the specified index.\n\n"
       "Args:\n"
       "    index: Zero-based index into the shape's circles.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));
    // Python special methods
    pyShape.def("__repr__", [](const murosim::geometry::Shape &self) {
        std::ostringstream oss;
        oss << "<Shape lines=" << self.getLines().size() << " circles=" << self.getCircles().size() << ">";
        return oss.str();
    }, "Return a debug string representation.");
    pyShape.def("__str__", [](const murosim::geometry::Shape &self) {
        std::stringstream ss;
        ss << self;
        return ss.str();
    }, "Return a human-readable \"Shape(lines={...}, circles={...})\" representation.");
    pyShape.def("__copy__", [](const murosim::geometry::Shape &self) { return murosim::geometry::Shape(self); },
        "Support copy.copy(); returns an independent copy.");
    pyShape.def("__deepcopy__", [](const murosim::geometry::Shape &self, pybind11::dict) {
        return murosim::geometry::Shape(self);
    }, "Support copy.deepcopy(); returns an independent copy.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
