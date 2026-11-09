// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/io/world.hpp"

namespace murosimpy {

void initIOFunctions(pybind11::module &m)
{
    // Module-level I/O functions
    m.def("save_world", [](const std::string &path, const murosim::geometry::World &world,
                           const murosim::navigation::Container &nodes) {
        return murosim::io::world::saveWorldFile(path, world, nodes);
    }, "Save world geometry and nodes to a file.\n\n"
       "Args:\n"
       "    filename: Output file path\n"
       "    world: World geometry to save\n"
       "    nodes: Navigation nodes to save\n\n"
       "Raises:\n"
       "    RuntimeError: if the file could not be written.",
       pybind11::arg("filename"), pybind11::arg("world"), pybind11::arg("nodes"));

    m.def("save_world", [](const std::string &path, const murosim::geometry::Shape &shape,
                           const murosim::navigation::Container &nodes) {
        return murosim::io::world::saveWorldFile(path, shape, nodes);
    }, "Save shape geometry and nodes to a file.\n\n"
       "Args:\n"
       "    filename: Output file path\n"
       "    shape: Shape geometry to save\n"
       "    nodes: Navigation nodes to save\n\n"
       "Raises:\n"
       "    RuntimeError: if the file could not be written.",
       pybind11::arg("filename"), pybind11::arg("shape"), pybind11::arg("nodes"));

    m.def("save_world", [](const std::string &path, const murosim::io::DataContainer &data) {
        return murosim::io::world::saveWorldFile(path, data);
    }, "Save a DataContainer to a file.\n\n"
       "Args:\n"
       "    filename: Output file path\n"
       "    data: DataContainer to save\n\n"
       "Raises:\n"
       "    RuntimeError: if the file could not be written.",
       pybind11::arg("filename"), pybind11::arg("data"));

    m.def("load_world", [](const std::string &path, float scale, bool mirror_x, bool mirror_y) {
        if (scale != 1.0f || mirror_x || mirror_y) {
            const murosim::io::DataContainer data = murosim::io::world::loadWorldFile(path);
            Eigen::Vector2f s((mirror_x) ? -scale : scale, (mirror_y) ? -scale : scale);
            return data.scale(s);
        }
        return murosim::io::world::loadWorldFile(path);
    }, "Load world data from a file, named after the file's stem.\n\n"
       "Args:\n"
       "    filename: Input file path; must have a \".world\" extension\n"
       "    scale: Scale factor applied to every line, circle and node after loading (default: 1.0)\n"
       "    mirror_x: Mirror along the X axis (negates the X scale factor, default: False)\n"
       "    mirror_y: Mirror along the Y axis (negates the Y scale factor, default: False)\n\n"
       "Returns:\n"
       "    DataContainer with the loaded world data.\n\n"
       "Raises:\n"
       "    RuntimeError: if filename does not end in \".world\", the file cannot be opened, "
       "any line fails to parse or validate, or the file contains no lines, circles or nodes at all.",
       pybind11::arg("filename"),
       pybind11::arg("scale") = 1.0f,
       pybind11::arg("mirror_x") = false,
       pybind11::arg("mirror_y") = false);
}

} // namespace murosimpy
