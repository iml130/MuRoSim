// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include <Eigen/Geometry>

#include <sstream>


namespace murosimpy {
namespace {

Eigen::Affine2f createTransformIdentity()
{
    return Eigen::Affine2f::Identity();
}

Eigen::Affine2f createTransform(const float x, const float y, const float angle)
{
    Eigen::Affine2f affine = Eigen::Affine2f::Identity();
    affine.translate(Eigen::Vector2f(x, y));
    affine.rotate(angle);
    return affine;
}

} // anonymous namespace

void initGeometryTransform(pybind11::module &m)
{
    auto pyTransform = pybind11::class_<Eigen::Affine2f>(m, "Transform",
        "2D affine transformation matrix for translating, rotating, and scaling shapes.");

    // === Transform ===
    pyTransform.def(pybind11::init(&createTransformIdentity),
        "Create an identity transform.");
    pyTransform.def(pybind11::init(&createTransform),
        "Create a transform with translation and rotation.\n\n"
        "Args:\n"
        "    x: Translation in x\n"
        "    y: Translation in y\n"
        "    angle: Rotation angle in radians",
        pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle"));
    pyTransform.def("inverse", [](const Eigen::Affine2f &affine) { return affine.inverse(); },
        "Return the inverse of this transform.");
    pyTransform.def("translate", [](Eigen::Affine2f &affine, float x, float y) {
        affine.translate(Eigen::Vector2f(x, y));
    }, "Apply a translation to this transform.",
       pybind11::arg("x"), pybind11::arg("y"));
    pyTransform.def("rotate", [](Eigen::Affine2f &affine, float angle) { affine.rotate(angle); },
        "Apply a rotation to this transform.",
        pybind11::arg("angle"));
    pyTransform.def("scale", [](Eigen::Affine2f &affine, float x, float y) {
        affine.scale(Eigen::Vector2f(x, y));
    }, "Apply non-uniform scaling.",
       pybind11::arg("x"), pybind11::arg("y"));
    pyTransform.def("scale", [](Eigen::Affine2f &affine, float scale) { affine.scale(scale); },
        "Apply uniform scaling.",
        pybind11::arg("scale"));
    pyTransform.def(pybind11::self * pybind11::self,
        "Compose two transforms (matrix product); the right-hand transform is applied first.");
    pyTransform.def("__repr__", [](const Eigen::Affine2f & /*self*/) {
        std::ostringstream oss;
        oss << "<Transform>";
        return oss.str();
    }, "Return a debug string representation.");
    pyTransform.def("__copy__", [](const Eigen::Affine2f &self) { return Eigen::Affine2f(self); },
        "Support copy.copy(); returns an independent copy.");
    pyTransform.def("__deepcopy__", [](const Eigen::Affine2f &self, pybind11::dict) {
        return Eigen::Affine2f(self);
    }, "Support copy.deepcopy(); returns an independent copy.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
