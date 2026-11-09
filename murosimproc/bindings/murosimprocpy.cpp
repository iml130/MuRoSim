// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "pybind11/pybind11.h"

#include "version.hpp"

#include "generator.hpp"
#include "factory.hpp"


namespace murosimprocpy {

PYBIND11_MODULE(murosimprocpy, m) {
    // Optional docstring
    m.doc() = "Python bindings for MuRoSimProc.";

    // Register the murosimpy types (DataContainer, Simulation, ...) before
    // binding anything that uses them, so the module works when imported on
    // its own and the signatures name the Python types instead of C++ ones.
    pybind11::module_::import("murosimpy");

    m.attr("__version__") = MUROSIMPROCPY_VERSION;

    initModuleGenerator(m);
    initModuleFactory(m);
}

} // namespace murosimprocpy
