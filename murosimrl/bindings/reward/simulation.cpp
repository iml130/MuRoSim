// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

namespace murosimrlpy {

// The concrete simulation-scope rewards were removed: they were per-simulation
// averages of the robot-scope rewards (or unfinished). For a global signal
// (e.g. MAPPO), aggregate the robot-scope rewards trainer-side - the per-robot
// output is ordered simulation by simulation. The module and the
// RewardSimulation base class remain as the extension point.
void initRewardSimulation(py::module_ &)
{
}

} // namespace murosimrlpy
