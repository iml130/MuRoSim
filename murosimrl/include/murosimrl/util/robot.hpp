// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_UTIL_ROBOT_HPP_
#define MUROSIMRL_UTIL_ROBOT_HPP_

#include "murosim/robot/robot.hpp"
#include "murosim/actuator/actuator.hpp"
#include <Eigen/Core>
#include <array>

namespace murosimrl::util::robot
{

/// Check whether all robots share the same actuator type. True for 0 or 1 robots.
bool checkRobotHasSameActuator(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots);

/// Check whether every robot has at least one recorded state.
bool checkRobotHasStates(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots);

/// Indices of the Vector3f axes (x=0, y=1, z=2) an actuator type actuates,
/// written to `axes`; returns the number of actuated axes. Throws on unknown
/// actuator types. Single owner of the axis layout shared by Action and the
/// velocity observations.
/// @param type Actuator type to look up.
/// @param axes Receives the actuated axis indices, in observation/action order.
/// @return Number of actuated axes (2 for X_ANGULAR and X_Y, 3 for X_Y_ANGULAR).
size_t axesForActuatorType(const murosim::actuator::Actuator::TYPE type, std::array<size_t, 3> &axes);

/// Throws if index_lidar is out of range for any robot or the laser sizes
/// differ. Checks lidar configuration only (no states required), so shape()
/// can run it before the simulation has stepped. Single owner of the contract
/// shared by the lidar observations.
/// @param robots Robots to check.
/// @param index_lidar Index into each robot's lidar list.
void requireConsistentLidars(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const size_t index_lidar);

/// Unit vector, in the robot's local frame, pointing from the robot's pose towards its goal in `state`.
/// Zero if the robot is exactly on its goal.
Eigen::Vector2f orientationToGoalFromState(const murosim::robot::State &state);

/// Euclidean distance, in metres, from the robot's pose to its goal in `state`.
float distanceToGoalFromState(const murosim::robot::State &state);

} // namespace murosimrl::util::robot

#endif // MUROSIMRL_UTIL_ROBOT_HPP_
