// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "binding.hpp"

#include "murosimrl/observation/observation_simulation.hpp"

// Simulation observations
#include "murosimrl/observation/simulation/occupancy_map.hpp"
#include "murosimrl/observation/simulation/velocity_map.hpp"
#include "murosimrl/observation/simulation/path_map.hpp"

#include "murosim/simulation/simulation.hpp"

namespace murosimrlpy {

using observation_binding::bindSimulationObservation;

void initObservationSimulation(py::module_ &m)
{
    // OccupancyMap observation
    {
        auto pyClass = py::class_<murosimrl::observation::simulation::OccupancyMap>(m, "OccupancyMap", R"doc(Full occupancy grid map of each simulation.

Embedded centered in a map_size x map_size buffer, with values from the
grid value lookup table (0.0 free, 1.0 occupied/unreachable, 0.5 buffer if
with_buffer); cells outside the map's own extent (padding, or when the map
is smaller than map_size) read 1.0. Feature shape is (1, map_size,
map_size) per simulation (channel-first for CNN input, not stacked, since
the map does not change within an episode). Batch size is the number of
simulations. The map must fit within map_size x map_size (raises
otherwise).

Args:
    map_size: Side length of the square output buffer, in cells; must be
        at least the largest grid map's size. Raises if 0.
    with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
)doc");
        pyClass.def(py::init<size_t, bool>(),
            "Args:\n"
            "    map_size: Side length of the square output buffer, in "
            "cells; must be at least the largest grid map's size. Raises "
            "if 0.\n"
            "    with_buffer: Map BUFFER cells to 0.5 instead of treating "
            "them as free.",
            py::arg("map_size"),
            py::arg("with_buffer") = false);
        pyClass.def("map_size", &murosimrl::observation::simulation::OccupancyMap::mapSize,
            "The configured output buffer side length, in cells.");
        pyClass.def("with_buffer", &murosimrl::observation::simulation::OccupancyMap::withBuffer,
            "Whether BUFFER cells map to 0.5 instead of free.");
        bindSimulationObservation(pyClass);
        addCopySupport(pyClass);
    }

    // VelocityMap observation
    {
        auto pyClass = py::class_<murosimrl::observation::simulation::VelocityMap>(m, "VelocityMap", R"doc(Rasterized world-frame velocity field of each simulation's robots.

Embedded centered in a map_size x map_size grid (same centering as
OccupancyMap). Channel 0 and 1 are the x and y world-frame velocity
components (metres/second; row order is y-flipped like the grid map, so
channel 1 stores the negated y velocity) written at each robot's occupied
cells; channel 2 (present only if history_size > 0) is a recency weight in
(0, 1] that fades older states in the same stacked frame towards 0.
Feature shape per stacked state is (2, map_size, map_size) (history_size
== 0) or (3, map_size, map_size). Cells with no robot stay 0. Each robot
occupies either just its center cell, or every cell within its collision
radius if use_collision_radius is set. If history_size > 0, up to that
many past states per stacked index are blended into the same frame;
robots without a recorded state yet are skipped. The map must fit within
map_size x map_size (raises otherwise).

Args:
    map_size: Side length of the square output buffer, in cells; must be
        at least the largest grid map's size. Raises if 0.
    history_size: Number of past states blended per stacked frame via a
        fading recency channel; 0 disables the history channel.
    use_collision_radius: Rasterize each robot's full collision-radius
        footprint instead of just its center cell.
    stack_size: Number of stacked past frames per simulation; 0 and 1 are
        equivalent (no stacking).
)doc");
        pyClass.def(py::init<size_t, size_t, bool, size_t>(),
            "Args:\n"
            "    map_size: Side length of the square output buffer, in "
            "cells; must be at least the largest grid map's size. Raises "
            "if 0.\n"
            "    history_size: Number of past states blended per stacked "
            "frame via a fading recency channel; 0 disables the history "
            "channel.\n"
            "    use_collision_radius: Rasterize each robot's full "
            "collision-radius footprint instead of just its center cell.\n"
            "    stack_size: Number of stacked past frames per simulation; "
            "0 and 1 are equivalent (no stacking).",
            py::arg("map_size"),
            py::arg("history_size") = 0,
            py::arg("use_collision_radius") = false,
            py::arg("stack_size") = 0);
        pyClass.def("map_size", &murosimrl::observation::simulation::VelocityMap::mapSize,
            "The configured output buffer side length, in cells.");
        pyClass.def("history_size", &murosimrl::observation::simulation::VelocityMap::historySize,
            "The configured history depth (0 means \"no history channel\").");
        pyClass.def("use_collision_radius", &murosimrl::observation::simulation::VelocityMap::useCollisionRadius,
            "Whether each robot's full collision-radius footprint is rasterized.");
        bindSimulationObservation(pyClass);
        addCopySupport(pyClass);
    }

    // PathMap observation
    {
        auto pyClass = py::class_<murosimrl::observation::simulation::PathMap>(m, "PathMap", R"doc(Occupancy grid map of each simulation with robot paths overlaid.

Same embedding and value convention as OccupancyMap, but with every
robot's planned path cells overwritten to -1.0. Feature shape is (1,
map_size, map_size) per simulation (channel-first for CNN input, not
stacked, since path points are not part of the robot state). Batch size is
the number of simulations. The map must fit within map_size x map_size
(raises otherwise). Path points outside the grid are silently skipped.

Args:
    map_size: Side length of the square output buffer, in cells; must be
        at least the largest grid map's size. Raises if 0.
    with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
)doc");
        pyClass.def(py::init<size_t, bool>(),
            "Args:\n"
            "    map_size: Side length of the square output buffer, in "
            "cells; must be at least the largest grid map's size. Raises "
            "if 0.\n"
            "    with_buffer: Map BUFFER cells to 0.5 instead of treating "
            "them as free.",
            py::arg("map_size"),
            py::arg("with_buffer") = false);
        pyClass.def("map_size", &murosimrl::observation::simulation::PathMap::mapSize,
            "The configured output buffer side length, in cells.");
        pyClass.def("with_buffer", &murosimrl::observation::simulation::PathMap::withBuffer,
            "Whether BUFFER cells map to 0.5 instead of free.");
        bindSimulationObservation(pyClass);
        addCopySupport(pyClass);
    }
}

} // namespace murosimrlpy
