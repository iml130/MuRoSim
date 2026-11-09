// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_SUBGOAL_DIRECTION_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_SUBGOAL_DIRECTION_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Chained directions, in the robot's local frame, towards a series of
/// subgoals spaced evenly along the robot's planned path up to
/// lookahead_distance ahead. Feature size is 2 * num_subgoals (no stacking:
/// always evaluated at the current state). Subgoal s sits at arc-length
/// distance ((s + 1) / num_subgoals) * lookahead_distance from the robot
/// along its path (util::path::interpolateAlongPath), clamping to the goal
/// once the path is exhausted; direction s is measured from subgoal (s - 1)
/// to subgoal s (from the robot's pose for s == 0), so the vectors trace a
/// polyline rather than all pointing from the robot. If normalize is set,
/// each direction is unit length (zero if degenerate); otherwise it carries
/// the raw metre offset between the two points. A subgoal that could not be
/// found (only possible for an empty path) contributes a zero vector. Reads
/// Robot::getPathPoints() and getPosition()/state pose.
class SubgoalDirection : public ObservationRobot
{
private:
    float lookahead_distance_;
    size_t num_subgoals_;
    bool normalize_;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param lookahead_distance Arc-length distance (metres) covered by the last subgoal. Clamped to >= 0.1.
    /// @param num_subgoals Number of evenly spaced subgoals. Clamped to >= 1.
    /// @param normalize Report unit-length directions instead of raw metre offsets.
    SubgoalDirection(const float lookahead_distance, const size_t num_subgoals, const bool normalize);

    /// Per-robot feature size is 2 * num_subgoals (no stacking).
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;

    /// The configured lookahead distance, in metres.
    float lookaheadDistance() const { return lookahead_distance_; }
    /// The configured number of subgoals.
    size_t numSubgoals() const { return num_subgoals_; }
    /// Whether directions are reported as unit vectors.
    bool normalize() const { return normalize_; }
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_SUBGOAL_DIRECTION_HPP_
