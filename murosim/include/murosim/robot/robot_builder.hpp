// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ROBOT_ROBOT_BUILDER_HPP_
#define MUROSIM_ROBOT_ROBOT_BUILDER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "murosim/actuator/actuator.hpp"
#include "murosim/actuator/differential_simple.hpp"
#include "murosim/actuator/differential_kobuki.hpp"
#include "murosim/actuator/holonomic_simple.hpp"
#include "murosim/actuator/robomaster.hpp"
#include "murosim/geometry/shape.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/sensor/lidar.hpp"
#include "murosim/math/constants.hpp"

namespace murosim::robot
{

class Robot;

/// Fluent builder for assembling a Robot from an actuator, a body shape and
/// optional lidar sensors, without exposing Robot's constructor directly.
/// build() requires an actuator and a robot shape to have been set, and
/// throws std::runtime_error otherwise. The builder can be reset() and reused
/// to construct further robots.
class RobotBuilder
{
public:
    RobotBuilder() = default;
    ~RobotBuilder() = default;

    RobotBuilder(const RobotBuilder&) = delete;
    RobotBuilder& operator=(const RobotBuilder&) = delete;
    RobotBuilder(RobotBuilder&&) = default;
    RobotBuilder& operator=(RobotBuilder&&) = default;

    // --- Name --

    /// Sets the robot's name.
    RobotBuilder& setName(const std::string& name);

    // --- Actuator (required) ---

    /// Sets a caller-supplied actuator, taking ownership.
    RobotBuilder& setActuator(std::unique_ptr<murosim::actuator::Actuator> actuator);

    // DifferentialSimple overloads
    /// Uses a DifferentialSimple actuator with default limits.
    RobotBuilder& setActuatorDifferentialSimple();
    /// Uses a DifferentialSimple actuator with the given velocity limits.
    RobotBuilder& setActuatorDifferentialSimple(float velocity_linear_min, float velocity_linear_max, float velocity_angular_max);
    /// Uses a DifferentialSimple actuator with the given velocity and acceleration limits.
    RobotBuilder& setActuatorDifferentialSimple(float velocity_linear_min, float velocity_linear_max, float velocity_angular_max, float acceleration_linear, float acceleration_angular);

    // HolonomicSimple overloads
    /// Uses a HolonomicSimple actuator with default limits.
    RobotBuilder& setActuatorHolonomicSimple();
    /// Uses a HolonomicSimple actuator with the given velocity limits.
    RobotBuilder& setActuatorHolonomicSimple(float linear_x_min, float linear_x_max, float linear_y_max, float angular_max);
    /// Uses a HolonomicSimple actuator with the given velocity and acceleration limits.
    RobotBuilder& setActuatorHolonomicSimple(float linear_x_min, float linear_x_max, float linear_y_max, float angular_max, float acceleration_linear, float acceleration_angular);

    // --- Robot shape (required) ---

    /// Sets the robot's collision shape (copied).
    RobotBuilder& setShapeRobot(const murosim::geometry::Shape& shape);
    /// Sets the robot's collision shape (moved).
    RobotBuilder& setShapeRobot(murosim::geometry::Shape&& shape);
    /// Sets the robot's collision shape to a centered rectangle of the given width/height (metres).
    RobotBuilder& setShapeRobotRectangle(float width, float height);
    /// Sets the robot's collision shape to a centered circle of the given radius (metres).
    RobotBuilder& setShapeRobotCircle(float radius);

    // --- Optional ---

    /// Sets the shape other robots' lidars see this robot as; optional, defaults to the robot shape.
    /// An explicitly set empty shape makes the robot invisible to other lidars.
    RobotBuilder& setShapeLidar(const murosim::geometry::Shape& shape);
    /// Adds a preconfigured lidar sensor (copied).
    RobotBuilder& addLidar(const murosim::sensor::Lidar& lidar);
    /// Adds a preconfigured lidar sensor (moved).
    RobotBuilder& addLidar(murosim::sensor::Lidar&& lidar);
    /// Constructs and adds a lidar sensor in place.
    /// @param pose Mounting pose relative to the robot's local frame.
    /// @param fov Field of view in radians.
    /// @param resolution Number of laser rays.
    /// @param range_min Minimum measurable range (metres); closer hits read -inf.
    /// @param range_max Maximum measurable range (metres); rays without a hit within it read +inf.
    /// @param frequency Update frequency in Hz.
    /// @param noise_type Measurement noise model.
    /// @param noise Noise magnitude in metres: the standard deviation for NORMAL, the half-width for UNIFORM.
    /// @throws std::invalid_argument if resolution is 0 or the range is not
    /// finite with 0 <= range_min < range_max.
    RobotBuilder& addLidar(const Eigen::Affine2f& pose, float fov, size_t resolution, float range_min, float range_max, float frequency, murosim::sensor::Lidar::NOISE_TYPE noise_type = murosim::sensor::Lidar::NOISE_TYPE::NONE, float noise = 0.0f, float angle_start = murosim::math::PI, bool rotation_clockwise = false);

    // --- Build ---

    /// Builds the Robot from the configured actuator, shapes and lidars.
    /// @throws std::runtime_error if no actuator or no robot shape has been set.
    std::shared_ptr<Robot> build() const;

    // --- Reset ---

    /// Clears all configured state so the builder can be reused from scratch.
    void reset();

private:
    std::string name_;
    std::unique_ptr<murosim::actuator::Actuator> actuator_;
    murosim::geometry::Shape shape_robot_;
    murosim::geometry::Shape shape_lidar_;
    std::vector<murosim::sensor::Lidar> lidars_;
    bool has_actuator_ = false;
    bool has_shape_robot_ = false;
    bool has_shape_lidar_ = false;
};

} // namespace murosim::robot

#endif // MUROSIM_ROBOT_ROBOT_BUILDER_HPP_
