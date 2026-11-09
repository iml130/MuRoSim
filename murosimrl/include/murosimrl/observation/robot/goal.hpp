// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_GOAL_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_GOAL_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Robot's goal in world coordinates: center (x, y) in metres and radius in
/// metres, per stacked state. Feature size is 3 per stacked state. Not
/// normalized. Reads State::getGoal().getCircle() for each stacked state.
class Goal : public ObservationRobot
{
protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param stack_size Number of stacked past goals per robot; 0 and 1 are equivalent (no stacking).
    Goal(const size_t stack_size);

    /// Per-robot feature size is 3 (x, y, radius), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_GOAL_HPP_
