// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_POSE_MAP_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_POSE_MAP_HPP_

#include "murosimrl/observation/observation_robot_simulation.hpp"

namespace murosimrl::observation::robot_simulation
{

/// Per-robot occupancy grid patch (patch_dim x patch_dim cells) centered on
/// the robot's own position, per stacked state, values from
/// util::grid::gridValueLut (0.0 free, 1.0 occupied/unreachable/out-of-map,
/// 0.5 buffer if with_buffer). Batch size is the total robot count across all
/// simulations. Reads Simulation::getGridMap() and each robot's
/// getStateAtOrOldest(j).getPose() for each stacked state j.
class PoseMap : public ObservationRobotSimulation
{
private:
    size_t patch_dim_;
    bool with_buffer_;

    /// Fill `array` with one patch_dim x patch_dim patch per robot per stacked state, in simulation then robot order.
    void fillObservationPoseMap(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const override;

public:
    /// @param patch_dim Side length of the square patch, in grid cells.
    /// @param with_buffer Map BUFFER cells to 0.5 instead of treating them as free.
    /// @param stack_size Number of stacked past patches per robot; 0 and 1 are equivalent (no stacking).
    PoseMap(const size_t patch_dim, const bool with_buffer, const size_t stack_size);

    /// Feature shape is [patch_dim, patch_dim] per stacked state per robot.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const override;

    /// The configured patch side length, in grid cells.
    size_t patchDim() const { return patch_dim_; }
    /// Whether BUFFER cells map to 0.5 instead of free.
    bool withBuffer() const { return with_buffer_; }
};

} // namespace murosimrl::observation::robot_simulation

#endif // MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_POSE_MAP_HPP_
