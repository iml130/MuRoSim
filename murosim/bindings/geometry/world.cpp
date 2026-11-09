// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/geometry/world.hpp"

#include <sstream>

namespace murosimpy {

void initGeometryWorld(pybind11::module &m)
{
    auto pyWorld = pybind11::class_<murosim::geometry::World>(m, "World",
        R"doc(The static world geometry: all lines and circles making up the simulated environment.

Built once from a Shape (or from explicit line/circle lists) and immutable
afterwards. Also tracks the overall axis-aligned bounding box (in metres) of
all geometry.)doc");

    // === World ===
    pyWorld.def(pybind11::init<const murosim::geometry::Shape&>(),
        "Build the world from a shape's lines and circles.",
        pybind11::arg("shape"));
    // Properties
    pyWorld.def_property_readonly("shape", &murosim::geometry::World::getShape,
        "Rebuilds and returns a Shape containing all of this world's lines and circles.");
    pyWorld.def_property_readonly("line_count", &murosim::geometry::World::getLineSize,
        "Number of lines in this world.");
    pyWorld.def_property_readonly("circle_count", &murosim::geometry::World::getCircleSize,
        "Number of circles in this world.");
    pyWorld.def_property_readonly("aabb_x", [](const murosim::geometry::World &self) {
        return pybind11::make_tuple(self.getAabbMinX(), self.getAabbMaxX());
    }, "X-axis bounds of the axis-aligned bounding box as (min, max), in metres.");
    pyWorld.def_property_readonly("aabb_y", [](const murosim::geometry::World &self) {
        return pybind11::make_tuple(self.getAabbMinY(), self.getAabbMaxY());
    }, "Y-axis bounds of the axis-aligned bounding box as (min, max), in metres.");
    // Deprecated aliases for backwards compatibility; prefer aabb_x / aabb_y.
    pyWorld.def_property_readonly("area_x", [](const murosim::geometry::World &self) {
        return pybind11::make_tuple(self.getAabbMinX(), self.getAabbMaxX());
    }, "Deprecated alias for aabb_x.");
    pyWorld.def_property_readonly("area_y", [](const murosim::geometry::World &self) {
        return pybind11::make_tuple(self.getAabbMinY(), self.getAabbMaxY());
    }, "Deprecated alias for aabb_y.");
    pyWorld.def_property_readonly("width", &murosim::geometry::World::getAabbSizeX,
        "World width (x-axis extent of the bounding box), in metres.");
    pyWorld.def_property_readonly("height", &murosim::geometry::World::getAabbSizeY,
        "World height (y-axis extent of the bounding box), in metres.");
    // Indexed access (bounds-checked: getLineAt/getCircleAt index the underlying
    // arrays unchecked, so a bad index from Python would read out of bounds)
    pyWorld.def("line_at", [](const murosim::geometry::World &self, size_t index) {
        checkIndex(index, self.getLineSize(), "Line");
        return self.getLineAt(index);
    }, "Get a copy of the line at the specified index.\n\n"
       "Args:\n"
       "    index: Zero-based index into the world's lines.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));
    pyWorld.def("circle_at", [](const murosim::geometry::World &self, size_t index) {
        checkIndex(index, self.getCircleSize(), "Circle");
        return self.getCircleAt(index);
    }, "Get a copy of the circle at the specified index.\n\n"
       "Args:\n"
       "    index: Zero-based index into the world's circles.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));
    // Python special methods
    pyWorld.def("__repr__", [](const murosim::geometry::World &self) {
        std::ostringstream oss;
        oss << "<World size=(" << self.getAabbSizeX() << ", " << self.getAabbSizeY()
            << ") lines=" << self.getLineSize() << " circles=" << self.getCircleSize() << ">";
        return oss.str();
    }, "Return a debug string representation.");
    pyWorld.def("__copy__", [](const murosim::geometry::World &self) { return murosim::geometry::World(self); },
        "Support copy.copy(); returns an independent copy.");
    pyWorld.def("__deepcopy__", [](const murosim::geometry::World &self, pybind11::dict) {
        return murosim::geometry::World(self);
    }, "Support copy.deepcopy(); returns an independent copy.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
