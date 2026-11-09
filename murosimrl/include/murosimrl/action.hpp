// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_ACTION_HPP_
#define MUROSIMRL_ACTION_HPP_

#include <murosim/robot/robot.hpp>
#include <memory>
#include <vector>

namespace murosimrl
{

/// Applies RL actions (one float per actuated axis) to robots.
///
/// The command written follows the robot's actuator CONTROL_MODE: a target
/// velocity in VELOCITY mode, an acceleration in ACCELERATION mode. With
/// normalize, the [-1, 1] action maps onto the matching range (the velocity
/// limits, or the symmetric [-max, max] acceleration limits). The actuator's
/// mode is the single source of truth - there is deliberately no mode flag
/// here that could disagree with it.
class Action
{
private:
    bool normalize_;

public:
    /// @param normalize Interpret action values as [-1, 1] and map them onto the actuator's command range.
    Action(const bool normalize);

    /// Apply an action to a single robot.
    /// @param robot Robot to command.
    /// @param data One float per actuated axis (see size()/shape()).
    void operator()(std::shared_ptr<murosim::robot::Robot> &robot, const float* data) const;

    /// Apply one action per robot, packed contiguously. All robots must share one actuator type.
    /// @param robots Robots to command, in the same order as `data`.
    /// @param data Concatenated per-robot actions, size(robots[0]) floats each.
    void operator()(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const float* data) const;

    /// Number of actuated axes (action floats) for a robot's actuator type.
    static size_t size(const std::shared_ptr<murosim::robot::Robot> &robot);
    /// Action shape for a single robot: {size(robot)}.
    static std::vector<size_t> shape(const std::shared_ptr<murosim::robot::Robot> &robot);
    /// Action shape for a robot list: {robots.size(), size(robots[0])}, or empty if `robots` is empty.
    static std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots);

    /// Whether actions are interpreted as normalized [-1, 1] values.
    bool normalize() const noexcept { return normalize_; }

    /// Write the action that commands zero (velocity or acceleration, per the robot's control mode) into `output`.
    /// @param robot Robot whose actuator range defines the neutral point.
    /// @param output Buffer of size(robot) floats.
    static void getNeutralPoint(const std::shared_ptr<murosim::robot::Robot> &robot, float* output);
    /// Same as the pointer overload, returned as a newly allocated vector.
    static std::vector<float> getNeutralPoint(const std::shared_ptr<murosim::robot::Robot> &robot);
};

} // namespace murosimrl

#endif // MUROSIMRL_ACTION_HPP_
