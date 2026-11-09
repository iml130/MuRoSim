// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/geometry/circle.hpp"

#include <sstream>

namespace murosimpy {

void initGeometryCircle(pybind11::module &m)
{
    auto pyCircle = pybind11::class_<murosim::geometry::Circle>(m, "Circle",
        R"doc(A circle defined by a center point and a radius (metres).

The basic round obstacle/robot-shape primitive used throughout geometry,
collision, grid and navigation code. The radius is always clamped to be
non-negative.)doc");

    // === Circle ===
    pyCircle.def(pybind11::init<const float, const float, const float>(),
        "Create a circle at (x, y) with given radius, in metres. A negative radius is clamped to 0.",
        pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"));
    // Properties
    pyCircle.def_property_readonly("x", &murosim::geometry::Circle::getX,
        "X coordinate of the center, in metres.");
    pyCircle.def_property_readonly("y", &murosim::geometry::Circle::getY,
        "Y coordinate of the center, in metres.");
    pyCircle.def_property_readonly("radius", &murosim::geometry::Circle::getRadius,
        "Circle radius in metres (always >= 0).");
    // Transform methods
    pyCircle.def("translate", pybind11::overload_cast<const float, const float>(&murosim::geometry::Circle::translate),
        "Translate the circle by (dx, dy), in metres.",
        pybind11::arg("dx"), pybind11::arg("dy"));
    pyCircle.def("rotate", pybind11::overload_cast<const float>(&murosim::geometry::Circle::rotate),
        "Rotate the circle's center around the origin by angle radians (the radius is unaffected).",
        pybind11::arg("angle"));
    pyCircle.def("transform", pybind11::overload_cast<const float, const float, const float>(&murosim::geometry::Circle::transform),
        "Place at pose (x, y, angle): rotate about the origin, then translate (T * R). "
        "x and y are in metres, angle in radians; the radius is unaffected.",
        pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle"));
    pyCircle.def("transform", pybind11::overload_cast<const Eigen::Affine2f &>(&murosim::geometry::Circle::transform),
        "Apply an affine transform to the center, and shrink the radius by the "
        "smaller of the transform's two axis scale factors so a non-uniform scale does not grow the circle.",
        pybind11::arg("transform"));
    pyCircle.def("scale", pybind11::overload_cast<const float>(&murosim::geometry::Circle::scale),
        "Uniformly scale the center (about the origin) and the radius by factor.",
        pybind11::arg("factor"));
    pyCircle.def(Eigen::Affine2f() * pybind11::self,
        "Return a new circle with the affine transform applied (transform * circle); "
        "see transform() for how the radius is scaled.");
    // Python special methods
    pyCircle.def("__repr__", [](const murosim::geometry::Circle &self) {
        std::ostringstream oss;
        oss << "<Circle center=(" << self.getX() << ", " << self.getY()
            << ") radius=" << self.getRadius() << ">";
        return oss.str();
    }, "Return a debug string representation.");
    pyCircle.def("__str__", [](const murosim::geometry::Circle &self) {
        std::stringstream ss;
        ss << self;
        return ss.str();
    }, "Return a human-readable \"circle(x=.., y=.., r=..)\" representation.");
    pyCircle.def("__copy__", [](const murosim::geometry::Circle &self) { return murosim::geometry::Circle(self); },
        "Support copy.copy(); returns an independent copy.");
    pyCircle.def("__deepcopy__", [](const murosim::geometry::Circle &self, pybind11::dict) {
        return murosim::geometry::Circle(self);
    }, "Support copy.deepcopy(); returns an independent copy.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
