// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_DISTANCE_TO_GOAL_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_DISTANCE_TO_GOAL_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Euclidean distance from the robot's pose to its goal, in metres, per
/// stacked state. Feature size is 1 per stacked state. If normalize_distance
/// is positive, the value is divided by it (so a distance of
/// normalize_distance maps to 1.0); otherwise the raw metre distance is
/// reported. Reads State::getPose()/getGoal() via
/// util::robot::distanceToGoalFromState for each stacked state.
class DistanceToGoal : public ObservationRobot
{
private:
    float normalize_distance_;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param normalize_distance Distance (metres) that maps to 1.0; 0 or negative disables normalization. Clamped to >= 0.
    /// @param stack_size Number of stacked past distances per robot; 0 and 1 are equivalent (no stacking).
    DistanceToGoal(const float normalize_distance, const size_t stack_size);

    /// Per-robot feature size is 1 (distance), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;

    /// The configured normalization distance (0 means "no normalization").
    float normalizeDistance() const { return normalize_distance_; }
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_DISTANCE_TO_GOAL_HPP_
