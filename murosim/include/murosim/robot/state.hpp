// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ROBOT_STATE_HPP_
#define MUROSIM_ROBOT_STATE_HPP_

#include "murosim/actuator/actuator.hpp"
#include "murosim/sensor/lidar.hpp"
#include "murosim/navigation/node.hpp"

#include <string>
#include <Eigen/Geometry>

namespace murosim::robot
{

/// A snapshot of one robot's pose, actuator command/velocity and per-lidar
/// distance readings, taken at a single simulation step. Robot::makeState()
/// records these into a StateBuffer ring buffer; they are the basis for
/// observations built by murosimrl.
class State
{
private:
    Eigen::Affine2f pose_;
    std::vector<murosim::simd::ArrayFloat> lidar_distance_;
    Eigen::Vector3f actuator_command_velocity_;
    Eigen::Vector3f actuator_command_acceleration_;
    Eigen::Vector3f actuator_velocity_;
    murosim::navigation::Node goal_;

public:
    /// Constructs a zero/identity state with no lidar readings.
    State();
    /// Constructs a state snapshot from the given actuator, lidars, pose and goal.
    State(const murosim::actuator::Actuator &actuator, const std::vector<murosim::sensor::Lidar> &lidars, const Eigen::Affine2f &pose, const murosim::navigation::Node &goal);
    /** \brief Fill this state in place, reusing the existing lidar buffers
     * when the sizes match (avoids per-step heap allocations). */
    void assign(const murosim::actuator::Actuator &actuator, const std::vector<murosim::sensor::Lidar> &lidars, const Eigen::Affine2f &pose, const murosim::navigation::Node &goal);
    /// Recorded pose at the time this state was taken.
    Eigen::Affine2f getPose() const;
    /// Distance readings of the lidar at index `i`.
    /// @param i Lidar index, must be < getLidarDistanceSize().
    const murosim::simd::ArrayFloat &getLidarDistanceAt(const size_t i) const;
    /// Number of lidars recorded in this state.
    size_t getLidarDistanceSize() const;
    /// Commanded body-frame velocity (x, y, angular) at the time this state was taken.
    Eigen::Vector3f getActuatorCommandVelocity() const;
    /// Commanded body-frame acceleration (x, y, angular) at the time this state was taken.
    Eigen::Vector3f getActuatorCommandAcceleration() const;
    /// Realized body-frame velocity (x, y, angular) at the time this state was taken.
    Eigen::Vector3f getActuatorVelocity() const;
    /// Goal node at the time this state was taken.
    const murosim::navigation::Node &getGoal() const;
};



} // namespace murosim::robot

#endif // MUROSIM_ROBOT_STATE_HPP_
