// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"
#include "murosim/models/world.hpp"

namespace murosimpy {

void initModelsWorldFactory(pybind11::module &m)
{
    m.def("line", &murosim::models::world::line,
        "Create a simple corridor world.\n\n"
        "Args:\n"
        "    world_size: Overall extent of the world, in metres; must be positive (default: 5.0)\n"
        "    space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)\n\n"
        "Returns:\n"
        "    DataContainer with world geometry.\n\n"
        "Raises:\n"
        "    ValueError: if world_size or space is not positive, or space exceeds world_size.",
        pybind11::arg("world_size") = 5.0f, pybind11::arg("space") = 1.0f);

    m.def("empty", &murosim::models::world::empty,
        "Create an empty rectangular world with only the outer boundary, no internal obstacles.\n\n"
        "Args:\n"
        "    world_size: Overall extent of the world, in metres; must be positive (default: 5.0)\n"
        "    space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)\n\n"
        "Returns:\n"
        "    DataContainer with world geometry.\n\n"
        "Raises:\n"
        "    ValueError: if world_size or space is not positive, or space exceeds world_size.",
        pybind11::arg("world_size") = 5.0f, pybind11::arg("space") = 1.0f);

    m.def("intersection", &murosim::models::world::intersection,
        "Create a four-way intersection world.\n\n"
        "Args:\n"
        "    world_size: Overall extent of the world, in metres; must be positive (default: 5.0)\n"
        "    space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)\n\n"
        "Returns:\n"
        "    DataContainer with world geometry.\n\n"
        "Raises:\n"
        "    ValueError: if world_size or space is not positive, or space exceeds world_size.",
        pybind11::arg("world_size") = 5.0f, pybind11::arg("space") = 1.0f);

    m.def("roundabout", &murosim::models::world::roundabout,
        "Create a roundabout world.\n\n"
        "Args:\n"
        "    world_size: Overall extent of the world, in metres; must be positive (default: 5.0)\n"
        "    space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)\n\n"
        "Returns:\n"
        "    DataContainer with world geometry.\n\n"
        "Raises:\n"
        "    ValueError: if world_size or space is not positive, or space exceeds world_size.",
        pybind11::arg("world_size") = 5.0f, pybind11::arg("space") = 1.0f);

    m.def("curve", &murosim::models::world::curve,
        "Create a curved corridor world.\n\n"
        "Args:\n"
        "    world_size: Overall extent of the world, in metres; must be positive (default: 5.0)\n"
        "    space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)\n\n"
        "Returns:\n"
        "    DataContainer with world geometry.\n\n"
        "Raises:\n"
        "    ValueError: if world_size or space is not positive, or space exceeds world_size.",
        pybind11::arg("world_size") = 5.0f, pybind11::arg("space") = 1.0f);

    m.def("bone", &murosim::models::world::bone,
        "Create a bone-shaped world with two open areas connected by a corridor.\n\n"
        "Args:\n"
        "    world_size: Overall extent of the world, in metres; must be positive (default: 5.0)\n"
        "    space: Corridor/gap width, in metres; must be positive and not exceed world_size (default: 1.0)\n\n"
        "Returns:\n"
        "    DataContainer with world geometry.\n\n"
        "Raises:\n"
        "    ValueError: if world_size or space is not positive, or space exceeds world_size.",
        pybind11::arg("world_size") = 5.0f, pybind11::arg("space") = 1.0f);
}

} // namespace murosimpy
