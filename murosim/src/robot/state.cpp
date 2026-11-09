// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/robot/state.hpp"

namespace murosim::robot
{

State::State()
    : pose_(Eigen::Affine2f::Identity()),
      actuator_command_velocity_(Eigen::Vector3f::Zero()),
      actuator_command_acceleration_(Eigen::Vector3f::Zero()),
      actuator_velocity_(Eigen::Vector3f::Zero()),
      goal_(murosim::navigation::Node())
{

}

State::State(const murosim::actuator::Actuator &actuator, const std::vector<murosim::sensor::Lidar> &lidars, const Eigen::Affine2f &pose, const murosim::navigation::Node &goal):
    pose_(pose),
    actuator_command_velocity_(actuator.getCommandVelocity()),
    actuator_command_acceleration_(actuator.getCommandAcceleration()),
    actuator_velocity_(actuator.getVelocity()),
    goal_(goal)
{
    for (size_t i = 0; i < lidars.size(); i++)
    {
        lidar_distance_.emplace_back(lidars[i].getArrayLaserDistance());
    }
}

void State::assign(const murosim::actuator::Actuator &actuator, const std::vector<murosim::sensor::Lidar> &lidars, const Eigen::Affine2f &pose, const murosim::navigation::Node &goal)
{
    pose_ = pose;
    actuator_command_velocity_ = actuator.getCommandVelocity();
    actuator_command_acceleration_ = actuator.getCommandAcceleration();
    actuator_velocity_ = actuator.getVelocity();
    goal_ = goal;

    lidar_distance_.resize(lidars.size());

    for (size_t i = 0; i < lidars.size(); i++)
    {
        // ArrayFloat::operator= reuses its allocation when the sizes match
        lidar_distance_[i] = lidars[i].getArrayLaserDistance();
    }
}

Eigen::Affine2f State::getPose() const
{
    return pose_;
}

const murosim::simd::ArrayFloat &State::getLidarDistanceAt(const size_t i) const
{
    return lidar_distance_[i];
}

size_t State::getLidarDistanceSize() const
{
    return lidar_distance_.size();
}

Eigen::Vector3f State::getActuatorCommandVelocity() const
{
    return actuator_command_velocity_;
}

Eigen::Vector3f State::getActuatorCommandAcceleration() const
{
    return actuator_command_acceleration_;
}

Eigen::Vector3f State::getActuatorVelocity() const
{
    return actuator_velocity_;
}

const murosim::navigation::Node &State::getGoal() const
{
    return goal_;
}


} // namespace murosim::robot
