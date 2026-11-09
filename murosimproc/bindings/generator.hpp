// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROCPY_GENERATOR_HPP_
#define MUROSIMPROCPY_GENERATOR_HPP_

#include "pybind11/pybind11.h"
#include "pybind11/numpy.h"



namespace murosimprocpy {

void initModuleGenerator(pybind11::module &m);

} // namespace murosimprocpy

#endif // MUROSIMPROCPY_GENERATOR_HPP_
