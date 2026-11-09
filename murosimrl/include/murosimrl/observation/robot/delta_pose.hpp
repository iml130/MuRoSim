// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_DELTA_POSE_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_DELTA_POSE_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Robot motion between consecutive recorded states, expressed in the
/// robot's own frame at the earlier of the two states: (dx, dy) translation
/// in metres and rotation angle in radians. Feature size is 3 per stacked
/// state. Not normalized. Reads two consecutive states per stacked index j
/// via Robot::getStateAtOrOldest(j) and (j + 1); at the oldest recorded
/// state, both reads clamp to it and the delta is zero.
class DeltaPose : public ObservationRobot
{
protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param stack_size Number of stacked deltas per robot; 0 and 1 are equivalent (no stacking).
    DeltaPose(const size_t stack_size);

    /// Per-robot feature size is 3 (dx, dy, dtheta), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_DELTA_POSE_HPP_
