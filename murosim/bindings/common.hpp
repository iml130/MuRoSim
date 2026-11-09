// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPY_COMMON_HPP_
#define MUROSIMPY_COMMON_HPP_

// Common pybind11 includes for automatic type conversions
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>       // Auto-converts std::vector, std::map, std::shared_ptr, etc.
#include <pybind11/eigen.h>     // Auto-converts Eigen::Vector2f, Vector3f, Affine2f to numpy
#include <pybind11/operators.h> // Support for operator overloading

#include <memory>
#include <string>
#include <vector>

// These std::vector<shared_ptr<...>> types are registered as the opaque
// "Robots" / "Simulations" container classes. PYBIND11_MAKE_OPAQUE specialises
// pybind11::detail::type_caster<T>; per [temp.expl.spec] that specialisation
// must be visible in EVERY translation unit that converts the type. Declaring
// them here — in the header every binding TU includes — keeps the behaviour
// consistent. Otherwise TUs that don't see it auto-convert the type to a Python
// list while the rest use the opaque holder (ill-formed, no diagnostic required).
namespace murosim::robot { class Robot; }
namespace murosim::simulation { class Simulation; }
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<murosim::robot::Robot>>)
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<murosim::simulation::Simulation>>)

// Common namespace aliases
namespace py = pybind11;

namespace murosimpy
{

// Bounds check for indexed accessors exposed to Python: single owner of the
// "throw IndexError past the end" contract every binding getter shares.
inline void checkIndex(const size_t index, const size_t size, const char *what)
{
    if (index >= size)
    {
        throw pybind11::index_error(std::string(what) + " index out of range");
    }
}

} // namespace murosimpy

#endif // MUROSIMPY_COMMON_HPP_
