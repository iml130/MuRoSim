// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/robot/robot.hpp"

#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/actuator/differential_simple.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/angle.hpp"
#include "murosim/sensor/lidar.hpp"
#include "murosim/math/constants.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iomanip>

namespace murosim::robot
{

Robot::Robot(const std::string name, const std::unique_ptr<murosim::actuator::Actuator> &actuator, const murosim::geometry::Shape &shape_robot, const murosim::geometry::Shape &shape_lidar, const std::vector<murosim::sensor::Lidar> &lidars)
    : name_(name),
      pose_(Eigen::Affine2f::Identity()),
      actuator_(actuator->clone()),
      shape_robot_(shape_robot),
      shape_lidar_(shape_lidar),
      lidars_(lidars),
      color_robot_(murosim::visualization::Color(0.0f, 0.0f, 1.0f)),
      color_lidar_(murosim::visualization::Color(0.0f, 1.0f, 0.0f)),
      color_path_ (murosim::visualization::Color(0.0f, 0.0f, 1.0f))
{
    collision_radius_ = shape_robot_.getCollisionRadius();

    reset();
}

Robot::Robot(const Robot& other):
    name_(other.name_),
    pose_(other.pose_),
    actuator_(other.actuator_ ? other.actuator_->clone() : nullptr),
    shape_robot_(other.shape_robot_),
    shape_lidar_(other.shape_lidar_),
    lidars_(other.lidars_),
    goal_(other.goal_),
    state_buffer_(other.state_buffer_),
    path_points_(other.path_points_),
    color_robot_(other.color_robot_),
    color_lidar_(other.color_lidar_),
    color_path_(other.color_path_),
    simulated_step_count_(other.simulated_step_count_),
    time_on_goal_(other.time_on_goal_),
    traveled_distance_(other.traveled_distance_),
    collision_radius_(other.collision_radius_),
    is_finished_(other.is_finished_),
    status_collision_(other.status_collision_),
    status_state_(other.status_state_)
{
}

Robot& Robot::operator=(Robot other)
{
    swap(*this, other);
    return *this;
}

Robot::Robot(Robot&& other) noexcept = default;

void swap(Robot& first, Robot& second) noexcept
{
    std::swap(first.name_, second.name_);
    std::swap(first.pose_, second.pose_);
    std::swap(first.shape_robot_, second.shape_robot_);
    std::swap(first.shape_lidar_, second.shape_lidar_);
    std::swap(first.actuator_, second.actuator_);
    std::swap(first.lidars_, second.lidars_);
    std::swap(first.goal_, second.goal_);
    std::swap(first.state_buffer_, second.state_buffer_);
    std::swap(first.color_robot_, second.color_robot_);
    std::swap(first.color_lidar_, second.color_lidar_);
    std::swap(first.color_path_, second.color_path_);
    std::swap(first.simulated_step_count_, second.simulated_step_count_);
    std::swap(first.time_on_goal_, second.time_on_goal_);
    std::swap(first.traveled_distance_, second.traveled_distance_);
    std::swap(first.collision_radius_, second.collision_radius_);
    std::swap(first.is_finished_, second.is_finished_);
    std::swap(first.status_collision_, second.status_collision_);
    std::swap(first.status_state_, second.status_state_);
    std::swap(first.path_points_, second.path_points_);
}

const std::string &Robot::getName() const
{
    return name_;
}

const std::vector<murosim::sensor::Lidar>& Robot::getLidars() const
{
    return lidars_;
}

std::vector<murosim::sensor::Lidar>& Robot::getLidars()
{
    return lidars_;
}

const murosim::sensor::Lidar& Robot::getLidarAt(const size_t index) const
{
    assert(index < lidars_.size());
    return lidars_[index];
}

sensor::Lidar& Robot::getLidarAt(const size_t index)
{
    assert(index < lidars_.size());
    return lidars_[index];
}

size_t Robot::getLidarSize() const
{
    return lidars_.size();
}

size_t Robot::getStateSize() const
{
    return state_buffer_.size();
}

bool Robot::hasStates() const
{
    return state_buffer_.size() != 0;
}

bool Robot::hasPathPoints() const
{
    return !path_points_.empty();
}

const State &Robot::getStateAt(const size_t index) const
{
    assert(index < state_buffer_.size() && "Index out of bounds in getStateAt");
    return state_buffer_[index];
}

const State &Robot::getStateAtOrOldest(const size_t index) const
{
    assert(0 < state_buffer_.size() && "getStateAtOrOldest requires at least one state");
    return state_buffer_[std::min(index, state_buffer_.size() - 1)];
}

void Robot::setSimulatedStepCount(const size_t simulated_step_count)
{
    simulated_step_count_ = simulated_step_count;
}

size_t Robot::getSimulatedStepCount() const
{
    return simulated_step_count_;
}

void Robot::move(const float time)
{
    const Eigen::Vector2f position_before = pose_.translation();
    pose_ = pose_ * actuator_->step(time);
    traveled_distance_ += (pose_.translation() - position_before).norm();
}

void Robot::reset()
{
    actuator_->reset();
    state_buffer_.reset();
    simulated_step_count_ = 0;

    status_collision_ = STATUS_COLLISION::NONE;
    status_state_ = STATUS_STATE::IDLE;
    is_finished_ = false;
    time_on_goal_ = 0.0f;
    traveled_distance_ = 0.0f;

    // a reset may teleport the robot: no ray measured before it may show up
    // in the next episode
    for (murosim::sensor::Lidar &lidar : lidars_)
    {
        lidar.reset();
    }
}

void Robot::makeState()
{
    // fill the ring-buffer slot in place to reuse its lidar buffers instead
    // of allocating a fresh State every step
    state_buffer_.nextState().assign(*actuator_, lidars_, pose_, goal_);
}

void Robot::setPathPoints(const std::vector<Eigen::Vector2f>& path_points)
{
    path_points_ = path_points;
}

void Robot::setPoseFromNode(const murosim::navigation::Node & node)
{
    setPose(node.getX(), node.getY(), node.getStartAngle());
}

Eigen::Vector2f Robot::randomPositionInCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator) const
{
    const float bias_position = std::max(0.0f, circle.getRadius() - collision_radius_);

    std::uniform_real_distribution<float> dis_position(-bias_position, bias_position);
    std::uniform_real_distribution<float> dis_pi(-murosim::math::PI, murosim::math::PI);

    return circle.getPoint() + Eigen::Rotation2Df(dis_pi(random_generator)) * Eigen::Vector2f(dis_position(random_generator), 0.0f);
}

void Robot::setPoseFromNode(const murosim::navigation::Node & node,murosim::random::RandomGenerator &random_generator)
{
    const float bias_angle = node.getStartBias() * 0.5f;
    std::uniform_real_distribution<float> dis_angle(-bias_angle, bias_angle);

    const float angle = node.getStartAngle() + dis_angle(random_generator);
    const Eigen::Vector2f position = randomPositionInCircle(node.getCircle(), random_generator);

    setPose(position.x(), position.y(), angle);
}

void Robot::setPoseFromCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator)
{
    std::uniform_real_distribution<float> dis_pi(-murosim::math::PI, murosim::math::PI);

    const float angle = dis_pi(random_generator);
    const Eigen::Vector2f position = randomPositionInCircle(circle, random_generator);

    setPose(position.x(), position.y(), angle);
}

void Robot::setPose(const Eigen::Affine2f &pose)
{
    pose_ = pose;
}

void Robot::setPose(const Eigen::Vector2f &position, const Eigen::Rotation2Df &rotation)
{
    pose_ = murosim::geometry::primitives::pose(position.x(), position.y(), rotation);
}

void Robot::setPose(const float x, const float y, const float rotation)
{
    pose_ = murosim::geometry::primitives::pose(x, y, rotation);
}

void Robot::setPosition(const Eigen::Vector2f &position)
{
    const Eigen::Rotation2Df rotation(pose_.rotation());
    pose_.setIdentity();
    pose_.translate(Eigen::Vector2f(position));
    pose_.rotate(rotation);
}

void Robot::setPosition(const float x, const float y)
{
    const Eigen::Rotation2Df rotation(pose_.rotation());
    pose_.setIdentity();
    pose_.translate(Eigen::Vector2f(x, y));
    pose_.rotate(rotation);
}

void Robot::setRotation(const Eigen::Rotation2Df &rotation)
{
    const Eigen::Vector2f position(pose_.translation());
    pose_.setIdentity();
    pose_.translate(Eigen::Vector2f(position));
    pose_.rotate(rotation);
}

void Robot::setRotation(const float rotation)
{
    const Eigen::Vector2f position(pose_.translation());
    pose_.setIdentity();
    pose_.translate(Eigen::Vector2f(position));
    pose_.rotate(rotation);
}

void Robot::setActuatorCommandVelocity(const float x, const float y, const float angular)
{
    actuator_->setCommandVelocity(x, y, angular);
}

void Robot::setActuatorCommandVelocity(const Eigen::Vector3f &cmd_vel)
{
    actuator_->setCommandVelocity(cmd_vel);
}

void Robot::setActuatorCommandAcceleration(const float x, const float y, const float angular)
{
    actuator_->setCommandAcceleration(x, y, angular);
}

void Robot::setActuatorCommandAcceleration(const Eigen::Vector3f &cmd_acc)
{
    actuator_->setCommandAcceleration(cmd_acc);
}

void Robot::setActuatorControlMode(const murosim::actuator::CONTROL_MODE mode)
{
    actuator_->setControlMode(mode);
}

void Robot::setStatusCollision(const STATUS_COLLISION status)
{
    status_collision_ = status;
}

void Robot::setStatusState(const STATUS_STATE status)
{
    status_state_ = status;
}

void Robot::setGoal(const murosim::navigation::Node &node, const float goal_radius)
{
    goal_ = node;
    goal_.setGoalRadius(goal_radius);
}

void Robot::setGoal(const murosim::navigation::Node &node)
{
    goal_ = node;
}

void Robot::setGoal(const float x, const float y, const float radius)
{
    goal_ = murosim::navigation::Node(x, y, radius);
}

void Robot::setGoal(const murosim::geometry::Circle &circle)
{
    goal_ = murosim::navigation::Node(circle);
}

void Robot::setTimeOnGoal(const float time)
{
    time_on_goal_ = time;
}

void Robot::setStateBufferSize(const size_t size)
{
    state_buffer_.setBufferSize(size);
}

void Robot::setColorRobot(const uint8_t r, const uint8_t g, const uint8_t b)
{
    color_robot_ = murosim::visualization::Color(r, g, b);
}

void Robot::setColorRobot(const float r, const float g, const float b)
{
    color_robot_ = murosim::visualization::Color(r, g, b);
}

void Robot::setColorRobot(const Eigen::Vector3f color)
{
    color_robot_ = murosim::visualization::Color(color);
}

void Robot::setColorRobot(const Eigen::Vector3i color)
{
    color_robot_ = murosim::visualization::Color(color);
}

void Robot::setColorLidar(const uint8_t r, const uint8_t g, const uint8_t b)
{
    color_lidar_ = murosim::visualization::Color(r, g, b);
}

void Robot::setColorLidar(const float r, const float g, const float b)
{
    color_lidar_ = murosim::visualization::Color(r, g, b);
}

void Robot::setColorLidar(const Eigen::Vector3f color)
{
    color_lidar_ = murosim::visualization::Color(color);
}

void Robot::setColorLidar(const Eigen::Vector3i color)
{
    color_lidar_ = murosim::visualization::Color(color);
}

void Robot::setColorPath(const uint8_t r, const uint8_t g, const uint8_t b)
{
    color_path_ = murosim::visualization::Color(r, g, b);
}

void Robot::setColorPath(const float r, const float g, const float b)
{
    color_path_ = murosim::visualization::Color(r, g, b);
}

void Robot::setColorPath(const Eigen::Vector3f color)
{
    color_path_ = murosim::visualization::Color(color);
}

void Robot::setColorPath(const Eigen::Vector3i color)
{
    color_path_ = murosim::visualization::Color(color);
}

const Eigen::Affine2f &Robot::getPose() const
{
    return pose_;
}

Eigen::Vector2f Robot::getPosition() const
{
    return pose_.translation();
}

Eigen::Rotation2Df Robot::getRotationMatrix() const
{
    return Eigen::Rotation2Df(pose_.rotation());
}

float Robot::getRotationAngle() const
{
    return murosim::math::rotationAngle(pose_);
}

const murosim::geometry::Shape& Robot::getShapeRobot() const
{
    return shape_robot_;
}

const murosim::geometry::Shape& Robot::getShapeLidar() const
{
    return shape_lidar_;
}

Eigen::Vector3f Robot::getActuatorCommandVelocity() const
{
    return actuator_->getCommandVelocity();
}

Eigen::Vector3f Robot::getActuatorCommandAcceleration() const
{
    return actuator_->getCommandAcceleration();
}

Eigen::Vector3f Robot::getActuatorVelocity() const
{
    return actuator_->getVelocity();
}

Eigen::Vector3f Robot::getActuatorVelocityMax() const
{
    return actuator_->getVelocityMax();
}

Eigen::Vector3f Robot::getActuatorVelocityMin() const
{
    return actuator_->getVelocityMin();
}

Eigen::Vector3f Robot::getActuatorAccelerationMax() const
{
    return actuator_->getAccelerationMax();
}

murosim::actuator::CONTROL_MODE Robot::getActuatorControlMode() const
{
    return actuator_->getControlMode();
}

murosim::actuator::Actuator::TYPE Robot::getActuatorType() const
{
    return actuator_->getType();
}

float Robot::getCollisionRadius() const
{
    return collision_radius_;
}

Robot::STATUS_COLLISION Robot::getStatusCollision() const
{
    return status_collision_;
}

Robot::STATUS_STATE Robot::getStatusState() const
{
    return status_state_;
}

float Robot::getTimeOnGoal() const
{
    return time_on_goal_;
}

float Robot::getTraveledDistance() const
{
    return traveled_distance_;
}

murosim::visualization::Color Robot::getColorRobot() const
{
    return color_robot_;
}

murosim::visualization::Color Robot::getColorLidar() const
{
    return color_lidar_;
}

murosim::visualization::Color Robot::getColorPath() const
{
    return color_path_;
}

std::string Robot::getStatusCollisionAsString(const STATUS_COLLISION collision)
{
    switch (collision)
    {
    case STATUS_COLLISION::NONE:
        return "NONE";

    case STATUS_COLLISION::WORLD:
        return "WORLD";

    case STATUS_COLLISION::DYNAMIC_ENTITY:
        return "DYNAMIC_ENTITY";

    case STATUS_COLLISION::ROBOT:
        return "ROBOT";

    default:
        return "UNKNOWN";
    }
}

std::string Robot::getStatusCollisionAsString() const
{
    return getStatusCollisionAsString(status_collision_);
}

std::string Robot::getStatusStateAsString(const STATUS_STATE state)
{
    switch (state)
    {
    case STATUS_STATE::IDLE:
        return "IDLE";

    case STATUS_STATE::ON_GOAL:
        return "ON_GOAL";

    case STATUS_STATE::REACHED_GOAL:
        return "REACHED_GOAL";

    case STATUS_STATE::SEARCH_GOAL:
        return "SEARCH_GOAL";

    case STATUS_STATE::UNUSED:
        return "UNUSED";

    default:
        return "UNKNOWN";
    }
}

std::string Robot::getStatusStateAsString() const
{
    return getStatusStateAsString(status_state_);
}

const murosim::navigation::Node &Robot::getGoal() const
{
    return goal_;
}

size_t Robot::getGroup() const
{
    return goal_.getGroup();
}

bool Robot::hasLidar() const
{
    return !lidars_.empty();
}

bool Robot::isFinished() const
{
    return is_finished_;
}

void Robot::finished()
{
    is_finished_ = true;
}

void Robot::planning(const std::unique_ptr<murosim::planning::Planner>& planner, const murosim::grid::Map& map)
{
    path_points_ = planner->plan(map, pose_.translation(), goal_.getPoint());
}

const std::vector<Eigen::Vector2f>& Robot::getPathPoints() const
{
    return path_points_;
}

size_t Robot::getPathPointSize() const
{
    return path_points_.size();
}

Eigen::Vector2f Robot::getPathPointAt(const size_t index) const
{
    assert(index < path_points_.size());
    return path_points_[index];
}

std::ostream& operator<<(std::ostream &os, const Robot& robot)
{
    os << "robot:\n"
       << "  position x: " << std::setw( 6 ) << std::setprecision( 4 ) << robot.getPosition().x()
       << "  y: " << std::setw( 6 ) << std::setprecision( 4 ) << robot.getPosition().y()
       << "  a: " << std::setw( 6 ) << std::setprecision( 4 ) << robot.getRotationAngle() << "\n"
       << "  goal x: " << std::setw( 6 ) << std::setprecision( 4 ) << robot.getGoal().getX()
       << "  y: " << std::setw( 6 ) << std::setprecision( 4 ) << robot.getGoal().getY()
       << "  r: " << std::setw( 6 ) << std::setprecision( 4 ) <<  robot.getGoal().getRadius() << "\n"
       << "  ss: " << robot.getStatusStateAsString() << "\n"
       << "  sc: " << robot.getStatusCollisionAsString() << "\n";

    return os;
}



} // namespace murosim::robot
