// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/models/robot.hpp"

#include "murosim/actuator/differential_kobuki.hpp"
#include "murosim/actuator/robomaster.hpp"
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/actuator/differential_simple.hpp"
#include "murosim/math/constants.hpp"

#include <algorithm>

namespace murosim::models::robot
{

std::shared_ptr<murosim::robot::Robot> robomaster(const float velocity_x_min, const float velocity_x_max, const float velocity_y_max, const float velocity_angular_max, const float acceleration_linear, const float acceleration_angular, const bool holonomic)
{
    const std::string name = "robomaster";
    murosim::actuator::Actuator::TYPE type = (holonomic)? murosim::actuator::Actuator::TYPE::X_Y_ANGULAR: murosim::actuator::Actuator::TYPE::X_ANGULAR;
    std::unique_ptr<murosim::actuator::Actuator> actuator = std::make_unique<murosim::actuator::RoboMaster>(velocity_x_min, velocity_x_max, velocity_y_max, velocity_angular_max, acceleration_linear, acceleration_angular, type);
    murosim::geometry::Shape shape_robot;
    shape_robot.addLine(murosim::geometry::Line( 0.16f,  0.12f, -0.16f,  0.12f));
    shape_robot.addLine(murosim::geometry::Line(-0.16f,  0.12f, -0.16f, -0.12f));
    shape_robot.addLine(murosim::geometry::Line(-0.16f, -0.12f,  0.16f, -0.12f));
    shape_robot.addLine(murosim::geometry::Line( 0.16f, -0.12f,  0.16f,  0.12f));
    murosim::geometry::Shape shape_lidar;
    shape_lidar.addCircle(murosim::geometry::Circle(0.1335f, 0.0f, 0.02f));
    shape_lidar.addLine(murosim::geometry::Line( 0.045f,  0.055f,  0.045f, -0.055f));
    shape_lidar.addLine(murosim::geometry::Line( 0.045f, -0.055f, -0.045f, -0.055f));
    shape_lidar.addLine(murosim::geometry::Line(-0.045f, -0.055f, -0.045f,  0.055f));
    shape_lidar.addLine(murosim::geometry::Line(-0.045f,  0.055f,  0.045f,  0.055f));
    std::vector<murosim::sensor::Lidar> lidars;
    Eigen::Affine2f pose_lidar = Eigen::Affine2f::Identity();
    pose_lidar.translate(Eigen::Vector2f(0.1335f, 0.0f));
    lidars.push_back(murosim::sensor::Lidar(pose_lidar, murosim::math::PI * 1.5f, 1200, 0.05f, 30.0f, 10.0f, murosim::sensor::Lidar::NOISE_TYPE::NORMAL, 0.0125f, murosim::math::PI, true));
    return std::make_shared<murosim::robot::Robot>(name, actuator, shape_robot, shape_lidar, lidars);
}

std::shared_ptr<murosim::robot::Robot> mir100(const float velocity_x_min, const float velocity_x_max, const float velocity_angular_max)
{
    // MiR100 datasheet: 1.5 m/s forwards, 0.3 m/s backwards
    const float vel_x_min = std::clamp(velocity_x_min, -0.3f, 1.5f);
    const float vel_x_max = std::clamp(velocity_x_max, -1.5f, 1.5f);
    const float vel_angular_max = std::min(std::abs(velocity_angular_max), murosim::math::PI_2);

    const std::string name = "mir100";
    const float acceleration_linear = 1.5f;
    const float acceleration_angular = murosim::math::PI_2;
    std::unique_ptr<murosim::actuator::Actuator> actuator = std::make_unique<murosim::actuator::DifferentialSimple>(vel_x_min, vel_x_max, vel_angular_max, acceleration_linear, acceleration_angular);
    murosim::geometry::Shape shape_robot;
    shape_robot.addLine( 0.45f,  0.30f, -0.45f,  0.30f);
    shape_robot.addLine( 0.45f, -0.30f, -0.45f, -0.30f);
    shape_robot.addLine( 0.45f,  0.30f,  0.45f, -0.30f);
    shape_robot.addLine(-0.45f,  0.30f, -0.45f, -0.30f);
    // orientation line as visual helper
    shape_robot.addLine(0.10f,  0.15f, 0.225f, 0.0f);
    shape_robot.addLine(0.10f, -0.15f, 0.225f, 0.0f);
    murosim::geometry::Shape shape_lidar;
    shape_lidar.addLine( 0.32f,  0.21f, -0.32f,  0.21f);
    shape_lidar.addLine( 0.32f, -0.21f, -0.32f, -0.21f);
    shape_lidar.addLine( 0.32f,  0.21f,  0.32f, -0.21f);
    shape_lidar.addLine(-0.32f,  0.21f, -0.32f, -0.21f);
    shape_lidar.addCircle( 0.32f,  0.21f,  0.047f);
    shape_lidar.addCircle(-0.32f, -0.21f,  0.047f);

    std::vector<murosim::sensor::Lidar> lidars;

    // SICK S300 measurement data output (operating instructions, technical
    // data, Long Range variant): a random error of 9 mm at 1 sigma on top of a
    // systematic error of up to +-20 mm, listed together as +-29 mm at 1 sigma
    constexpr float lidar_noise_sigma = 0.009f;

    Eigen::Affine2f pose_lidar = Eigen::Affine2f::Identity();
    pose_lidar.translate(Eigen::Vector2f( 0.32f,  0.21f));
    pose_lidar.rotate(murosim::math::PI_4);
    lidars.push_back(murosim::sensor::Lidar(pose_lidar, murosim::math::PI * 1.5f, 541, 0.05f, 29.0f, 12.5f, murosim::sensor::Lidar::NOISE_TYPE::NORMAL, lidar_noise_sigma, murosim::math::PI, false));

    pose_lidar = Eigen::Affine2f::Identity();
    pose_lidar.translate(Eigen::Vector2f(-0.32f,  -0.21f));
    pose_lidar.rotate((murosim::math::PI + murosim::math::PI_4));
    lidars.push_back(murosim::sensor::Lidar(pose_lidar, murosim::math::PI * 1.5f, 541, 0.05f, 29.0f, 12.5f, murosim::sensor::Lidar::NOISE_TYPE::NORMAL, lidar_noise_sigma, murosim::math::PI, false));

    return std::make_shared<murosim::robot::Robot>(name, actuator, shape_robot, shape_lidar, lidars);
}

} // namespace murosim::models::robot
