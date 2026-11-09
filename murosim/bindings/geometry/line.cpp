// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/geometry/line.hpp"

#include <sstream>

namespace murosimpy {

void initGeometryLine(pybind11::module &m)
{
    auto pyLine = pybind11::class_<murosim::geometry::Line>(m, "Line",
        R"doc(A 2D line segment defined by two endpoints, in metres.

The basic straight-edge obstacle/wall primitive used throughout geometry,
collision, grid and navigation code.)doc");

    // === Line ===
    pyLine.def(pybind11::init<const float, const float, const float, const float>(),
        "Create a line from (x1, y1) to (x2, y2), in metres.",
        pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"));
    // Properties
    pyLine.def_property_readonly("x1", &murosim::geometry::Line::getPoint1X,
        "X coordinate of the first endpoint, in metres.");
    pyLine.def_property_readonly("y1", &murosim::geometry::Line::getPoint1Y,
        "Y coordinate of the first endpoint, in metres.");
    pyLine.def_property_readonly("x2", &murosim::geometry::Line::getPoint2X,
        "X coordinate of the second endpoint, in metres.");
    pyLine.def_property_readonly("y2", &murosim::geometry::Line::getPoint2Y,
        "Y coordinate of the second endpoint, in metres.");
    // Transform methods
    pyLine.def("translate", pybind11::overload_cast<const float, const float>(&murosim::geometry::Line::translate),
        "Translate both endpoints by (dx, dy), in metres.",
        pybind11::arg("dx"), pybind11::arg("dy"));
    pyLine.def("rotate", pybind11::overload_cast<const float>(&murosim::geometry::Line::rotate),
        "Rotate both endpoints around the origin by angle radians.",
        pybind11::arg("angle"));
    pyLine.def("transform", pybind11::overload_cast<const float, const float, const float>(&murosim::geometry::Line::transform),
        "Place at pose (x, y, angle): rotate about the origin, then translate (T * R). "
        "x and y are in metres, angle in radians.",
        pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle"));
    pyLine.def("transform", pybind11::overload_cast<const Eigen::Affine2f &>(&murosim::geometry::Line::transform),
        "Apply an affine transform to both endpoints.",
        pybind11::arg("transform"));
    pyLine.def("scale", pybind11::overload_cast<const float>(&murosim::geometry::Line::scale),
        "Uniformly scale both endpoints about the origin by factor.",
        pybind11::arg("factor"));
    pyLine.def(Eigen::Affine2f() * pybind11::self,
        "Return a new line with the affine transform applied to both endpoints (transform * line).");
    // Python special methods
    pyLine.def("__repr__", [](const murosim::geometry::Line &self) {
        std::ostringstream oss;
        oss << "<Line (" << self.getPoint1X() << ", " << self.getPoint1Y()
            << ") -> (" << self.getPoint2X() << ", " << self.getPoint2Y() << ")>";
        return oss.str();
    }, "Return a debug string representation.");
    pyLine.def("__str__", [](const murosim::geometry::Line &self) {
        std::stringstream ss;
        ss << self;
        return ss.str();
    }, "Return a human-readable \"line(x1=.., y1=.., x2=.., y2=..)\" representation.");
    pyLine.def("__copy__", [](const murosim::geometry::Line &self) { return murosim::geometry::Line(self); },
        "Support copy.copy(); returns an independent copy.");
    pyLine.def("__deepcopy__", [](const murosim::geometry::Line &self, pybind11::dict) {
        return murosim::geometry::Line(self);
    }, "Support copy.deepcopy(); returns an independent copy.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
