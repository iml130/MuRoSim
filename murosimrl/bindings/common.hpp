// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRLPY_COMMON_HPP_
#define MUROSIMRLPY_COMMON_HPP_

// Common pybind11 includes for automatic type conversions
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <memory>
#include <vector>

// Mirror murosimpy's opaque container registration (see murosim/bindings/
// common.hpp): robot/simulation vectors are the "Robots"/"Simulations"
// classes registered there. With the same PYBIND11_MAKE_OPAQUE visible in
// every murosimrlpy TU, those containers cross this module's boundary BY
// REFERENCE instead of being re-converted element-wise per call — and plain
// Python lists are (intentionally) no longer accepted for these types.
namespace murosim::robot { class Robot; }
namespace murosim::simulation { class Simulation; }
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<murosim::robot::Robot>>)
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<murosim::simulation::Simulation>>)

// Common namespace aliases
namespace py = pybind11;

namespace murosimrlpy
{

// Registers __copy__/__deepcopy__ via the copy constructor. Single owner of
// the idiom shared by the observation, reward, done and action bindings.
template<typename BoundType, typename... Extra>
void addCopySupport(py::class_<BoundType, Extra...> &pyClass)
{
    pyClass.def("__copy__", [](const BoundType &self) {
        return BoundType(self);
    }, "Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).");
    pyClass.def("__deepcopy__", [](const BoundType &self, py::dict) {
        return BoundType(self);
    }, "Return a deep copy of this object. The memo dict is accepted for "
       "protocol compatibility but is unused, since there is no shared "
       "mutable state to alias.");
}

} // namespace murosimrlpy

#endif // MUROSIMRLPY_COMMON_HPP_
