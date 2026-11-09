// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/geometry/primitives.hpp"

namespace murosimpy {

void initGeometryPrimitives(pybind11::module &m)
{
    // === Geometry submodule ===
    m.def("triangle", pybind11::overload_cast<const float, const Eigen::Affine2f &>(&murosim::geometry::primitives::triangle),
        "Create an equilateral triangle shape (three lines) centered on its centroid, "
        "with one vertex pointing along +y, then apply transform.\n\n"
        "Args:\n"
        "    length: Side length, in metres\n"
        "    transform: Transform to apply after construction (default: identity)",
        pybind11::arg("length"), pybind11::arg_v("transform", Eigen::Affine2f::Identity(), "Transform()"));
    m.def("rectangle", pybind11::overload_cast<const float, const float, const Eigen::Affine2f &>(&murosim::geometry::primitives::rectangle),
        "Create a rectangle shape (four axis-aligned lines) centered on the origin, then apply transform.\n\n"
        "Args:\n"
        "    width: Rectangle width, in metres\n"
        "    height: Rectangle height, in metres\n"
        "    transform: Transform to apply after construction (default: identity)",
        pybind11::arg("width"), pybind11::arg("height"), pybind11::arg_v("transform", Eigen::Affine2f::Identity(), "Transform()"));
    m.def("square", pybind11::overload_cast<const float, const Eigen::Affine2f &>(&murosim::geometry::primitives::square),
        "Create a square shape (four axis-aligned lines) centered on the origin, then apply transform.\n\n"
        "Args:\n"
        "    length: Side length, in metres\n"
        "    transform: Transform to apply after construction (default: identity)",
        pybind11::arg("length"), pybind11::arg_v("transform", Eigen::Affine2f::Identity(), "Transform()"));
}

} // namespace murosimpy
