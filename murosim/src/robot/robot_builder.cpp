// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/robot/robot_builder.hpp"
#include "murosim/robot/robot.hpp"

#include <stdexcept>

namespace murosim::robot
{

// --- Name ---

RobotBuilder& RobotBuilder::setName(const std::string& name)
{
    name_ = name;
    return *this;
}

// --- Actuator ---

RobotBuilder& RobotBuilder::setActuator(std::unique_ptr<murosim::actuator::Actuator> actuator)
{
    actuator_ = std::move(actuator);
    has_actuator_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setActuatorDifferentialSimple()
{
    actuator_ = std::make_unique<murosim::actuator::DifferentialSimple>();
    has_actuator_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setActuatorDifferentialSimple(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max)
{
    actuator_ = std::make_unique<murosim::actuator::DifferentialSimple>(velocity_linear_min, velocity_linear_max, velocity_angular_max);
    has_actuator_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setActuatorDifferentialSimple(const float velocity_linear_min, const float velocity_linear_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular)
{
    actuator_ = std::make_unique<murosim::actuator::DifferentialSimple>(velocity_linear_min, velocity_linear_max, velocity_angular_max, acceleration_linear, acceleration_angular);
    has_actuator_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setActuatorHolonomicSimple()
{
    actuator_ = std::make_unique<murosim::actuator::HolonomicSimple>();
    has_actuator_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setActuatorHolonomicSimple(const float linear_x_min, const float linear_x_max, const float linear_y_max, const float angular_max)
{
    actuator_ = std::make_unique<murosim::actuator::HolonomicSimple>(linear_x_min, linear_x_max, linear_y_max, angular_max);
    has_actuator_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setActuatorHolonomicSimple(const float linear_x_min, const float linear_x_max, const float linear_y_max, const float angular_max, const float acceleration_linear, const float acceleration_angular)
{
    actuator_ = std::make_unique<murosim::actuator::HolonomicSimple>(linear_x_min, linear_x_max, linear_y_max, angular_max, acceleration_linear, acceleration_angular);
    has_actuator_ = true;
    return *this;
}

// --- Robot shape ---

RobotBuilder& RobotBuilder::setShapeRobot(const murosim::geometry::Shape& shape)
{
    shape_robot_ = shape;
    has_shape_robot_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setShapeRobot(murosim::geometry::Shape&& shape)
{
    shape_robot_ = std::move(shape);
    has_shape_robot_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setShapeRobotRectangle(const float width, const float height)
{
    shape_robot_ = murosim::geometry::primitives::rectangle(width, height);
    has_shape_robot_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::setShapeRobotCircle(const float radius)
{
    murosim::geometry::Shape shape;
    shape.addCircle(0.0f, 0.0f, radius);
    shape_robot_ = std::move(shape);
    has_shape_robot_ = true;
    return *this;
}

// --- Optional ---

RobotBuilder& RobotBuilder::setShapeLidar(const murosim::geometry::Shape& shape)
{
    shape_lidar_ = shape;
    has_shape_lidar_ = true;
    return *this;
}

RobotBuilder& RobotBuilder::addLidar(const murosim::sensor::Lidar& lidar)
{
    lidars_.push_back(lidar);
    return *this;
}

RobotBuilder& RobotBuilder::addLidar(murosim::sensor::Lidar&& lidar)
{
    lidars_.push_back(std::move(lidar));
    return *this;
}

RobotBuilder& RobotBuilder::addLidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max, const float frequency, const murosim::sensor::Lidar::NOISE_TYPE noise_type, const float noise, const float angle_start, const bool rotation_clockwise)
{
    lidars_.emplace_back(pose, fov, resolution, range_min, range_max, frequency, noise_type, noise, angle_start, rotation_clockwise);
    return *this;
}

// --- Build ---

std::shared_ptr<Robot> RobotBuilder::build() const
{
    if (!has_actuator_)
    {
        throw std::runtime_error("RobotBuilder::build: actuator is required");
    }
    if (!has_shape_robot_)
    {
        throw std::runtime_error("RobotBuilder::build: robot shape is required");
    }
    // Without a lidar shape of its own the robot is seen by other lidars as its body
    const murosim::geometry::Shape &shape_lidar = has_shape_lidar_ ? shape_lidar_ : shape_robot_;
    return std::make_shared<Robot>(name_, actuator_, shape_robot_, shape_lidar, lidars_);
}

// --- Reset ---

void RobotBuilder::reset()
{
    name_.clear();
    actuator_.reset();
    shape_robot_ = murosim::geometry::Shape();
    shape_lidar_ = murosim::geometry::Shape();
    lidars_.clear();
    has_actuator_ = false;
    has_shape_robot_ = false;
    has_shape_lidar_ = false;
}

} // namespace murosim::robot
