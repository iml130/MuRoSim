// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT


#ifndef MUROSIMPROCPY_FACTORY_HPP_
#define MUROSIMPROCPY_FACTORY_HPP_

// murosim's binding header carries the PYBIND11_MAKE_OPAQUE declarations for
// the "Simulations" container. Without it, generate_simulations would convert
// its std::vector<std::shared_ptr<Simulation>> to a plain Python list, which
// murosimpy's ThreadPool.execute_* (opaque parameter) rejects.
#include "murosim/bindings/common.hpp"

namespace murosimprocpy {

void initModuleFactory(pybind11::module &m);

} // namespace murosimprocpy

#endif // MUROSIMPROCPY_FACTORY_HPP_
