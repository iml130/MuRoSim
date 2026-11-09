// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_POSE_NORMALIZED_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_POSE_NORMALIZED_HPP_

#include "murosimrl/observation/observation_robot_simulation.hpp"

namespace murosimrl::observation::robot_simulation
{

/// Per-robot pose, position measured relative to the simulation world's AABB
/// center and, if normalize_distance is positive, divided by it and shifted
/// by 0.5 (so the world center maps to (0.5, 0.5)); with normalize_distance
/// <= 0 the raw metre offset from the world center is reported instead.
/// Orientation is appended as either a single angle in radians or a (cos,
/// sin) unit vector. Feature size is 3 (x, y, angle) or 4 (x, y, cos, sin)
/// per stacked state. Batch size is the total robot count across all
/// simulations. Reads Simulation::getWorld()'s AABB and each robot's
/// getStateAtOrOldest(j).getPose().
class PoseNormalized : public ObservationRobotSimulation
{
private:
    float normalize_distance_;
    bool orientation_as_unit_vector_;

    /// Fill `array` with (x, y, angle) per robot per stacked state, in simulation then robot order.
    void fillObservationPoseNormalized(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;
    /// Fill `array` with (x, y, cos, sin) per robot per stacked state, in simulation then robot order.
    void fillObservationPoseNormalizedWithOrientationAsUnitVector(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const override;

public:
    /// @param normalize_distance Distance (metres) the offset from world center is divided by; <= 0 disables normalization.
    /// @param orientation_as_unit_vector Encode orientation as (cos, sin) instead of a single angle.
    /// @param stack_size Number of stacked past poses per robot; 0 and 1 are equivalent (no stacking).
    PoseNormalized(const float normalize_distance, const bool orientation_as_unit_vector, const size_t stack_size);

    /// Per-robot feature size is 3 (x, y, angle) or 4 (x, y, cos, sin), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const override;

    /// The configured normalization distance (<= 0 means "no normalization").
    float normalizeDistance() const { return normalize_distance_; }
    /// Whether orientation is encoded as a (cos, sin) unit vector instead of an angle.
    bool orientationAsUnitVector() const { return orientation_as_unit_vector_; }
};

} // namespace murosimrl::observation::robot_simulation

#endif // MUROSIMRL_OBSERVATION_ROBOT_SIMULATION_POSE_NORMALIZED_HPP_
