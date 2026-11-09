// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/navigation/node.hpp"
#include "murosim/math/constants.hpp"

#include <sstream>

namespace murosimpy {

void initNavigationNode(pybind11::module &m)
{
    auto pyNode = pybind11::class_<murosim::navigation::Node>(m, "Node",
        R"doc(A named navigation point: a circular area a robot can spawn at, drive
through as a waypoint, or use as a goal.

Carries a preferred start heading/bias and an optional group id for
clustered start/goal placement. Angles are in radians. Node ids are
assigned/used by NodeContainer; group ids are remapped to compact bucket
indices when a NodeContainer is built from a list of nodes.)doc");

    // === Node ===
    pyNode.def(pybind11::init<const float, const float, const float, const float, const float, const size_t, const size_t>(),
        "Create a navigation node from center coordinates and radius.\n\n"
        "Args:\n"
        "    x: X coordinate of the center, in metres\n"
        "    y: Y coordinate of the center, in metres\n"
        "    radius: Node radius, in metres\n"
        "    start_angle: Initial orientation angle in radians, wrapped into (-2*pi, 2*pi) (default: 0)\n"
        "    start_bias: Angular range in radians around start_angle a robot may spawn within, clamped to [0, 2*pi] (default: 2*pi)\n"
        "    id: Unique node identifier (default: 0)\n"
        "    group: Group identifier for node clustering; 0 means ungrouped (default: 0)",
        pybind11::arg("x"),
        pybind11::arg("y"),
        pybind11::arg("radius"),
        pybind11::arg("start_angle") = 0.0f,
        pybind11::arg("start_bias") = murosim::math::PI * 2.0f,
        pybind11::arg("id") = 0,
        pybind11::arg("group") = 0);
    pyNode.def(pybind11::init<const murosim::geometry::Circle, const float, const float, const size_t, const size_t>(),
        "Create a navigation node from a circle.\n\n"
        "Args:\n"
        "    circle: Circle defining position and radius, in metres\n"
        "    start_angle: Initial orientation angle in radians, wrapped into (-2*pi, 2*pi) (default: 0)\n"
        "    start_bias: Angular range in radians around start_angle a robot may spawn within, clamped to [0, 2*pi] (default: 2*pi)\n"
        "    id: Unique node identifier (default: 0)\n"
        "    group: Group identifier for node clustering; 0 means ungrouped (default: 0)",
        pybind11::arg("circle"),
        pybind11::arg("start_angle") = 0.0f,
        pybind11::arg("start_bias") = murosim::math::PI * 2.0f,
        pybind11::arg("id") = 0,
        pybind11::arg("group") = 0);

    // Properties
    pyNode.def_property_readonly("x", &murosim::navigation::Node::getX,
        "X coordinate of the node center, in metres.");
    pyNode.def_property_readonly("y", &murosim::navigation::Node::getY,
        "Y coordinate of the node center, in metres.");
    pyNode.def_property_readonly("radius", &murosim::navigation::Node::getRadius,
        "Node radius, in metres.");
    pyNode.def_property_readonly("circle", &murosim::navigation::Node::getCircle,
        "The node's area as a Circle.");
    pyNode.def_property_readonly("start_angle", &murosim::navigation::Node::getStartAngle,
        "Initial orientation angle, in radians.");
    pyNode.def_property_readonly("start_bias", &murosim::navigation::Node::getStartBias,
        "Angular range in radians around start_angle for random start orientations (in [0, 2*pi]).");
    pyNode.def_property_readonly("id", &murosim::navigation::Node::getId,
        "Unique node identifier.");
    pyNode.def_property_readonly("group", &murosim::navigation::Node::getGroup,
        "Group identifier for node clustering (a compact bucket index once owned by a NodeContainer).");

    // Transform methods
    pyNode.def("translate", pybind11::overload_cast<const float, const float>(&murosim::navigation::Node::translate),
        "Translate the node's circle by (dx, dy), in metres.",
        pybind11::arg("dx"), pybind11::arg("dy"));
    pyNode.def("rotate", pybind11::overload_cast<const float>(&murosim::navigation::Node::rotate),
        "Rotate the node's circle around the origin by angle radians and advance the start angle by the same amount.",
        pybind11::arg("angle"));
    pyNode.def("transform", pybind11::overload_cast<const Eigen::Affine2f &>(&murosim::navigation::Node::transform),
        "Apply an affine transform to the node's circle and keep the start angle in sync with the transform's rotation component.",
        pybind11::arg("transform"));

    // Python special methods
    pyNode.def("__repr__", [](const murosim::navigation::Node &self) {
        std::ostringstream oss;
        oss << "<Node id=" << self.getId() << " pos=(" << self.getX() << ", " << self.getY()
            << ") radius=" << self.getRadius() << " group=" << self.getGroup() << ">";
        return oss.str();
    }, "Return a debug string representation.");
    pyNode.def("__copy__", [](const murosim::navigation::Node &self) { return murosim::navigation::Node(self); },
        "Support copy.copy(); returns an independent copy.");
    pyNode.def("__deepcopy__", [](const murosim::navigation::Node &self, pybind11::dict) {
        return murosim::navigation::Node(self);
    }, "Support copy.deepcopy(); returns an independent copy.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
