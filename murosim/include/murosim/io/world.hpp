// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_IO_WORLD_HPP_
#define MUROSIM_IO_WORLD_HPP_

#include "murosim/io/data_container.hpp"

#include <vector>
#include <string>
#include <filesystem>

/// Reads and writes the plain-text `.world` file format: a "# nodes" section
/// (`n x y r start_angle start_bias id group`, angles in degrees on disk),
/// a "# lines" section (`l x1 y1 x2 y2`) and a "# circles" section
/// (`c x y r`); '#' lines and blank lines are ignored. Floats are written
/// with std::numeric_limits<float>::max_digits10 precision for a lossless
/// round-trip.
namespace murosim::io::world
{
    /// Parses filename into a DataContainer named after the file's stem.
    /// @throws std::runtime_error if filename does not end in ".world", the
    /// file cannot be opened, any line fails to parse or fails validation
    /// (NaN/Inf, non-positive radius, negative node id/group, zero-length
    /// line, unknown record type, trailing extra data), or the file contains
    /// no lines, circles or nodes at all.
    murosim::io::DataContainer loadWorldFile(const std::filesystem::path &filename);
    /// Writes world's lines/circles and nodes' nodes to filename (see the lines/circles/nodes overload).
    void saveWorldFile(const std::filesystem::path &filename, const murosim::geometry::World &world, const murosim::navigation::Container &nodes);
    /// Writes shape's decomposed lines/circles and nodes' nodes to filename (see the lines/circles/nodes overload).
    void saveWorldFile(const std::filesystem::path &filename, const murosim::geometry::Shape &shape, const murosim::navigation::Container &nodes);
    /// Core writer all other overloads funnel into. Forces a ".world" extension
    /// on filename if it does not already have one.
    /// @throws std::runtime_error if the output file cannot be created.
    void saveWorldFile(const std::filesystem::path &filename, const std::vector<murosim::geometry::Line> &lines, const std::vector<murosim::geometry::Circle> &circles, const std::vector<murosim::navigation::Node> &nodes);
    /// Writes data's lines, circles and nodes to filename (see the lines/circles/nodes overload).
    void saveWorldFile(const std::filesystem::path &filename, const murosim::io::DataContainer &data);
    /// @return true if entry's extension is ".world".
    bool isWorldFile(const std::filesystem::path &entry);

} // namespace murosim::io::world

#endif // MUROSIM_IO_WORLD_HPP_
