// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/robot/robot.hpp"

#include <sstream>

namespace murosimpy {

void initCoreRobot(pybind11::module &m)
{
    auto pyRobot = pybind11::class_<murosim::robot::Robot, std::shared_ptr<murosim::robot::Robot>>(m, "Robot",
        "Represents a robot agent in the simulation with sensors and actuators.");

    // Read-only properties (new pythonic API)
    pyRobot.def_property_readonly("name", &murosim::robot::Robot::getName,
        "Robot's unique name identifier.");
    pyRobot.def_property_readonly("finished", &murosim::robot::Robot::isFinished,
        "Whether the robot has reached its goal or is otherwise finished.");
    pyRobot.def_property_readonly("steps", &murosim::robot::Robot::getSimulatedStepCount,
        "Number of simulation steps this robot has executed.");
    pyRobot.def_property_readonly("lidar_count", [](const murosim::robot::Robot &robot) {
        return robot.getLidars().size();
    }, "Number of lidar sensors on this robot.");
    pyRobot.def_property_readonly("actuator_type", &murosim::robot::Robot::getActuatorType,
        "Type of actuator (ActuatorType enum).");
    pyRobot.def_property("control_mode",
        &murosim::robot::Robot::getActuatorControlMode,
        &murosim::robot::Robot::setActuatorControlMode,
        "Actuator control mode (ControlMode enum): whether RL actions command\n"
        "a target velocity or an acceleration.");
    pyRobot.def_property_readonly("collision_status", &murosim::robot::Robot::getStatusCollision,
        "Current collision status (CollisionStatus enum).");
    pyRobot.def_property_readonly("state_status", &murosim::robot::Robot::getStatusState,
        "Current state status (RobotState enum).");
    pyRobot.def_property("pose",
        [](const murosim::robot::Robot &robot) {
            return Eigen::Vector3f(robot.getPosition().x(), robot.getPosition().y(), robot.getRotationAngle());
        },
        [](murosim::robot::Robot &robot, const Eigen::Vector3f &pose) {
            robot.setPose(pose.x(), pose.y(), pose.z());
        },
        "Current robot pose as [x, y, theta].");
    pyRobot.def_property("position",
        &murosim::robot::Robot::getPosition,
        [](murosim::robot::Robot &robot, const Eigen::Vector2f &pos) {
            robot.setPosition(pos.x(), pos.y());
        },
        "Current robot position as [x, y].");
    pyRobot.def_property("rotation",
        &murosim::robot::Robot::getRotationAngle,
        [](murosim::robot::Robot &robot, const float theta) {
            robot.setRotation(theta);
        },
        "Current robot rotation angle in radians.");
    pyRobot.def_property_readonly("traveled_distance", &murosim::robot::Robot::getTraveledDistance,
        "Cumulative distance traveled by the robot since the last reset.");
    pyRobot.def_property_readonly("velocity", &murosim::robot::Robot::getActuatorVelocity,
        "Current actuator velocity as [vx, vy, omega].");
    pyRobot.def_property("command_velocity",
        &murosim::robot::Robot::getActuatorCommandVelocity,
        [](murosim::robot::Robot &robot, const Eigen::Vector3f &vel) {
            robot.setActuatorCommandVelocity(vel.x(), vel.y(), vel.z());
        },
        "Commanded velocity as [vx, vy, omega].");
    /*
    pyRobot.def_property("command_acceleration",
        &murosim::robot::Robot::getActuatorCommandAcceleration,
        [](murosim::robot::Robot &robot, const Eigen::Vector3f &accel) {
            robot.setActuatorCommandAcceleration(accel.x(), accel.y(), accel.z());
        },
        "Commanded acceleration as [ax, ay, alpha].");
    */
    pyRobot.def_property_readonly("velocity_max", &murosim::robot::Robot::getActuatorVelocityMax,
        "Maximum actuator velocity as [vx_max, vy_max, omega_max].");
    pyRobot.def_property_readonly("velocity_min", &murosim::robot::Robot::getActuatorVelocityMin,
        "Minimum actuator velocity as [vx_min, vy_min, omega_min].");
    pyRobot.def_property("goal",
        [](const murosim::robot::Robot &robot) {
            return Eigen::Vector3f(robot.getGoal().getX(), robot.getGoal().getY(), robot.getGoal().getRadius());
        },
        [](murosim::robot::Robot &robot, const Eigen::Vector3f &goal) {
            // rebuild the node instead of Robot::setGoal(x, y, radius), which
            // would reset the group/id/angles and the effective goal radius
            const murosim::navigation::Node &old_goal = robot.getGoal();
            const murosim::navigation::Node node(murosim::geometry::Circle(goal.x(), goal.y(), goal.z()), old_goal.getStartAngle(), old_goal.getStartBias(), old_goal.getId(), old_goal.getGroup());
            // getGoalRadius() == getRadius() means the radius was "auto";
            // keep it auto (resolves to the new circle radius) instead of
            // freezing the old circle radius as an explicit goal radius
            const float goal_radius = (old_goal.getGoalRadius() == old_goal.getRadius()) ? -1.0f : old_goal.getGoalRadius();
            robot.setGoal(node, goal_radius);
        },
        "Current goal as [x, y, radius] (the goal node's circle). Assigning\n"
        "preserves the goal's group, id, start angle/bias and goal radius.");
    pyRobot.def_property("color_robot",
        [](const murosim::robot::Robot &robot) -> Eigen::Vector3f {
            // Materialize into a Vector3f: .head<3>() is a block referencing the
            // temporary Vector4f from asVector(), which dies at the return.
            return robot.getColorRobot().asVector().head<3>();
        },
        [](murosim::robot::Robot &robot, const Eigen::Vector3f &color) {
            robot.setColorRobot(color);
        },
        "Robot rendering color as [r, g, b].");
    pyRobot.def_property("color_path",
        [](const murosim::robot::Robot &robot) -> Eigen::Vector3f {
            return robot.getColorPath().asVector().head<3>();
        },
        [](murosim::robot::Robot &robot, const Eigen::Vector3f &color) {
            robot.setColorPath(color);
        },
        "Path rendering color as [r, g, b].");
    pyRobot.def_property("color_lidar",
        [](const murosim::robot::Robot &robot) -> Eigen::Vector3f {
            return robot.getColorLidar().asVector().head<3>();
        },
        [](murosim::robot::Robot &robot, const Eigen::Vector3f &color) {
            robot.setColorLidar(color);
        },
        "Lidar rendering color as [r, g, b].");

    // Lidar access
    pyRobot.def("lidar_distance", [](const murosim::robot::Robot &robot, const size_t index, const bool normalized) {
        // throw like every other indexed accessor in the bindings; a
        // silently empty array only surfaces later as an opaque numpy error
        checkIndex(index, robot.getLidars().size(), "Lidar");
        const murosim::sensor::Lidar &lidar = robot.getLidars()[index];
        pybind11::array_t<float> array(static_cast<ssize_t>(lidar.getLaserSize()), {});
        float *data = array.mutable_data();
        std::copy(lidar.getArrayLaserDistance().data(),
                  lidar.getArrayLaserDistance().data() + lidar.getLaserSize(), data);
        if (normalized) {
            // clamping maps +inf (no return) to 1 and -inf (too close) to 0
            const float range_min = lidar.getRangeMin();
            const float scale = 1.0f / (lidar.getRangeMax() - range_min);
            for (size_t i = 0; i < lidar.getLaserSize(); i++) {
                data[i] = (lidar.clampDistance(data[i]) - range_min) * scale;
            }
        }
        return array;
    }, "Get lidar distance readings as a numpy array.\n\n"
       "Readings follow ROS REP 117: +inf means no return within range_max,\n"
       "-inf a hit closer than range_min; finite readings lie in\n"
       "[range_min, range_max].\n\n"
       "Args:\n"
       "    index: Lidar sensor index (default: 0)\n"
       "    normalized: If True, clamp to [range_min, range_max] and map that\n"
       "        range to [0, 1], so +inf reads 1 and -inf reads 0 (default: False)\n\n"
       "Returns:\n"
       "    numpy array of distance readings.\n\n"
       "Raises:\n"
       "    IndexError: If index is out of range.",
       pybind11::arg("index") = static_cast<size_t>(0), pybind11::arg("normalized") = false);

    // Python special methods
    pyRobot.def("__repr__", [](const murosim::robot::Robot &robot) {
        std::ostringstream oss;
        oss << "<Robot '" << robot.getName() << "' pos=("
            << robot.getPosition().x() << ", " << robot.getPosition().y()
            << ") finished=" << (robot.isFinished() ? "True" : "False") << ">";
        return oss.str();
    }, "Return a short debug string with the robot's name, position and finished status.");
    pyRobot.def("clone", [](const murosim::robot::Robot &robot) {
        return std::make_shared<murosim::robot::Robot>(robot);
    }, "Create a copy of this robot.");
    pyRobot.def("__copy__", [](const murosim::robot::Robot &self) {
        return std::make_shared<murosim::robot::Robot>(self);
    }, "Support for copy.copy(): returns a deep copy, same as clone().");
    pyRobot.def("__deepcopy__", [](const murosim::robot::Robot &self, pybind11::dict /*memo*/) {
        return std::make_shared<murosim::robot::Robot>(self);
    }, "Support for copy.deepcopy(): returns a deep copy, same as clone().",
       pybind11::arg("memo"));
}

} // namespace murosimpy
