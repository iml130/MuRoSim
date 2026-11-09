// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "binding.hpp"

#include "murosimrl/observation/observation_robot_simulation.hpp"

// Robot simulation observations
#include "murosimrl/observation/robot_simulation/pose_normalized.hpp"
#include "murosimrl/observation/robot_simulation/goal_normalized.hpp"
#include "murosimrl/observation/robot_simulation/pose_map.hpp"
#include "murosimrl/observation/robot_simulation/goal_map.hpp"

#include "murosim/simulation/simulation.hpp"

namespace murosimrlpy {

using observation_binding::bindSimulationObservation;

void initObservationRobotSimulation(py::module_ &m)
{
    // PoseNormalized observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot_simulation::PoseNormalized>(m, "PoseNormalized", R"doc(Per-robot pose, normalized against the simulation world extent.

Position is measured relative to the simulation world's AABB center and,
if normalize_distance is positive, divided by it and shifted by 0.5 (so
the world center maps to (0.5, 0.5)); with normalize_distance <= 0 the raw
metre offset from the world center is reported instead. Orientation is
appended as either a single angle in radians or a (cos, sin) unit vector.
Feature size is 3 (x, y, angle) or 4 (x, y, cos, sin) per stacked state.
Batch size is the total robot count across all simulations.

Args:
    normalize_distance: Distance (metres) the offset from world center is
        divided by; <= 0 disables normalization.
    orientation_as_unit_vector: Encode orientation as (cos, sin) instead
        of a single angle.
    stack_size: Number of stacked past poses per robot; 0 and 1 are
        equivalent (no stacking).
)doc");
        pyClass.def(py::init<float, bool, size_t>(),
            "Args:\n"
            "    normalize_distance: Distance (metres) the offset from "
            "world center is divided by; <= 0 disables normalization.\n"
            "    orientation_as_unit_vector: Encode orientation as (cos, "
            "sin) instead of a single angle.\n"
            "    stack_size: Number of stacked past poses per robot; 0 and "
            "1 are equivalent (no stacking).",
            py::arg("normalize_distance") = 0.0f,
            py::arg("orientation_as_unit_vector") = false,
            py::arg("stack_size") = 0);
        pyClass.def("normalize_distance", &murosimrl::observation::robot_simulation::PoseNormalized::normalizeDistance,
            "The configured normalization distance, in metres (<= 0 means \"no normalization\").");
        pyClass.def("orientation_as_unit_vector", &murosimrl::observation::robot_simulation::PoseNormalized::orientationAsUnitVector,
            "Whether orientation is encoded as a (cos, sin) unit vector instead of an angle.");
        bindSimulationObservation(pyClass);
        addCopySupport(pyClass);
    }

    // GoalNormalized observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot_simulation::GoalNormalized>(m, "GoalNormalized", R"doc(Per-robot goal position, normalized against the simulation world extent.

Goal (x, y) is measured relative to the simulation world's AABB center
and, if normalize_distance is positive, divided by it and shifted by 0.5
(so the world center maps to (0.5, 0.5)); with normalize_distance <= 0 the
raw metre offset from the world center is reported instead. Feature size
is 2 per stacked state. Batch size is the total robot count across all
simulations.

Args:
    normalize_distance: Distance (metres) the offset from world center is
        divided by; <= 0 disables normalization.
    stack_size: Number of stacked past goals per robot; 0 and 1 are
        equivalent (no stacking).
)doc");
        pyClass.def(py::init<float, size_t>(),
            "Args:\n"
            "    normalize_distance: Distance (metres) the offset from "
            "world center is divided by; <= 0 disables normalization.\n"
            "    stack_size: Number of stacked past goals per robot; 0 and "
            "1 are equivalent (no stacking).",
            py::arg("normalize_distance") = 0.0f,
            py::arg("stack_size") = 0);
        pyClass.def("normalize_distance", &murosimrl::observation::robot_simulation::GoalNormalized::normalizeDistance,
            "The configured normalization distance, in metres (<= 0 means \"no normalization\").");
        bindSimulationObservation(pyClass);
        addCopySupport(pyClass);
    }

    // PoseMap observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot_simulation::PoseMap>(m, "PoseMap", R"doc(Per-robot occupancy grid patch centered on the robot's own position.

The patch is patch_dim x patch_dim grid cells, per stacked state, with
values from the grid value lookup table (0.0 free, 1.0
occupied/unreachable/out-of-map, 0.5 buffer if with_buffer). Feature shape
is (patch_dim, patch_dim) per stacked state. Batch size is the total robot
count across all simulations.

Args:
    patch_dim: Side length of the square patch, in grid cells.
    with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
    stack_size: Number of stacked past patches per robot; 0 and 1 are
        equivalent (no stacking).
)doc");
        // noconvert: without it pybind11's bool caster accepts any int on its
        // convert pass, so stale positional calls from the old (patch_dim,
        // map_size, ...) signature would silently bind map_size to with_buffer.
        pyClass.def(py::init<size_t, bool, size_t>(),
            "Args:\n"
            "    patch_dim: Side length of the square patch, in grid cells.\n"
            "    with_buffer: Map BUFFER cells to 0.5 instead of treating "
            "them as free.\n"
            "    stack_size: Number of stacked past patches per robot; 0 "
            "and 1 are equivalent (no stacking).",
            py::arg("patch_dim"),
            py::arg("with_buffer").noconvert() = false,
            py::arg("stack_size") = 0);
        pyClass.def("patch_dim", &murosimrl::observation::robot_simulation::PoseMap::patchDim,
            "The configured patch side length, in grid cells.");
        pyClass.def("with_buffer", &murosimrl::observation::robot_simulation::PoseMap::withBuffer,
            "Whether BUFFER cells map to 0.5 instead of free.");
        bindSimulationObservation(pyClass);
        addCopySupport(pyClass);
    }

    // GoalMap observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot_simulation::GoalMap>(m, "GoalMap", R"doc(Per-robot occupancy grid patch centered on the robot's goal.

The patch is patch_dim x patch_dim grid cells, with values from the grid
value lookup table (0.0 free, 1.0 occupied/unreachable/out-of-map, 0.5
buffer if with_buffer). Feature shape is (1, patch_dim, patch_dim)
(channel-first for CNN input). Batch size is the total robot count across
all simulations. The goal patch is static over time steps, so there is no
stacking.

Args:
    patch_dim: Side length of the square patch, in grid cells.
    with_buffer: Map BUFFER cells to 0.5 instead of treating them as free.
)doc");
        // noconvert: see PoseMap above.
        pyClass.def(py::init<size_t, bool>(),
            "Args:\n"
            "    patch_dim: Side length of the square patch, in grid cells.\n"
            "    with_buffer: Map BUFFER cells to 0.5 instead of treating "
            "them as free.",
            py::arg("patch_dim"),
            py::arg("with_buffer").noconvert() = false);
        pyClass.def("patch_dim", &murosimrl::observation::robot_simulation::GoalMap::patchDim,
            "The configured patch side length, in grid cells.");
        pyClass.def("with_buffer", &murosimrl::observation::robot_simulation::GoalMap::withBuffer,
            "Whether BUFFER cells map to 0.5 instead of free.");
        bindSimulationObservation(pyClass);
        addCopySupport(pyClass);
    }
}

} // namespace murosimrlpy
