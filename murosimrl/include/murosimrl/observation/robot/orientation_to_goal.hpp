// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_ORIENTATION_TO_GOAL_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_ORIENTATION_TO_GOAL_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Direction from the robot towards its goal, in the robot's local frame, as
/// a 2D vector (x, y). Feature size is 2 per stacked state. Unit length
/// (implicitly normalized to [-1, 1] per component) once the robot is
/// further than threshold_radius from the goal; inside that radius the
/// vector is scaled down linearly by distance / threshold_radius so it
/// shrinks to zero exactly on the goal, avoiding a discontinuous direction
/// flip right at the goal. Reads State::getPose()/getGoal() via
/// util::robot::orientationToGoalFromState/distanceToGoalFromState for each
/// stacked state.
class OrientationToGoal : public ObservationRobot
{
private:
    float threshold_radius_;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param threshold_radius Distance (metres) inside which the direction vector is scaled down towards zero. Clamped to >= 0.
    /// @param stack_size Number of stacked past values per robot; 0 and 1 are equivalent (no stacking).
    OrientationToGoal(const float threshold_radius, const size_t stack_size);

    /// Per-robot feature size is 2 (x, y direction), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;

    /// The configured threshold radius, in metres.
    float thresholdRadius() const { return threshold_radius_; }
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_ORIENTATION_TO_GOAL_HPP_
