// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/robot/robot.hpp"
#include "murosim/robot/robot_builder.hpp"
#include "murosim/sensor/lidar.hpp"
#include "murosim/math/constants.hpp"

namespace murosimpy {

void initCoreRobotBuilder(pybind11::module &m)
{
    namespace py = pybind11;
    using RobotBuilder = murosim::robot::RobotBuilder;
    using NoiseType = murosim::sensor::Lidar::NOISE_TYPE;

    // Register NOISE_TYPE enum if not already registered
    if (!py::detail::get_type_info(typeid(NoiseType))) {
        py::enum_<NoiseType>(m, "LidarNoiseType",
            "Lidar measurement noise model.")
            .value("NONE", NoiseType::NONE, "No noise")
            .value("NORMAL", NoiseType::NORMAL, "Gaussian noise")
            .value("UNIFORM", NoiseType::UNIFORM, "Uniform noise");
    }

    auto pyBuilder = py::class_<RobotBuilder>(m, "RobotBuilder",
        "Builder for creating Robot instances with a fluent API.\n\n"
        "Configure once, then call build() repeatedly to spawn multiple robots\n"
        "from the same template. Throws RuntimeError if actuator or shape is missing.");

    pyBuilder.def(py::init<>(), "Create an empty RobotBuilder.");

    // --- Name ---

    pyBuilder.def("set_name", &RobotBuilder::setName,
        py::return_value_policy::reference_internal,
        "Set the robot name.\n\n"
        "Returns:\n"
        "    This builder (bound to the same lifetime as self), for method chaining.",
        py::arg("name"));

    // --- Actuator setters ---

    // DifferentialSimple
    pyBuilder.def("set_actuator_differential_simple",
        py::overload_cast<>(&RobotBuilder::setActuatorDifferentialSimple),
        py::return_value_policy::reference_internal,
        "Set a default DifferentialSimple actuator (no velocity/acceleration limits configured).\n\n"
        "Returns:\n"
        "    This builder, for method chaining.");
    pyBuilder.def("set_actuator_differential_simple",
        py::overload_cast<float, float, float>(&RobotBuilder::setActuatorDifferentialSimple),
        py::return_value_policy::reference_internal,
        "Set a DifferentialSimple actuator with velocity limits.\n\n"
        "Args:\n"
        "    velocity_linear_min: Minimum linear velocity\n"
        "    velocity_linear_max: Maximum linear velocity\n"
        "    velocity_angular_max: Maximum angular velocity\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("velocity_linear_min"), py::arg("velocity_linear_max"), py::arg("velocity_angular_max"));
    pyBuilder.def("set_actuator_differential_simple",
        py::overload_cast<float, float, float, float, float>(&RobotBuilder::setActuatorDifferentialSimple),
        py::return_value_policy::reference_internal,
        "Set a DifferentialSimple actuator with velocity and acceleration limits.\n\n"
        "Args:\n"
        "    velocity_linear_min: Minimum linear velocity\n"
        "    velocity_linear_max: Maximum linear velocity\n"
        "    velocity_angular_max: Maximum angular velocity\n"
        "    acceleration_linear: Maximum linear acceleration\n"
        "    acceleration_angular: Maximum angular acceleration\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("velocity_linear_min"), py::arg("velocity_linear_max"), py::arg("velocity_angular_max"),
        py::arg("acceleration_linear"), py::arg("acceleration_angular"));

    // HolonomicSimple
    pyBuilder.def("set_actuator_holonomic_simple",
        py::overload_cast<>(&RobotBuilder::setActuatorHolonomicSimple),
        py::return_value_policy::reference_internal,
        "Set a default HolonomicSimple actuator (no velocity/acceleration limits configured).\n\n"
        "Returns:\n"
        "    This builder, for method chaining.");
    pyBuilder.def("set_actuator_holonomic_simple",
        py::overload_cast<float, float, float, float>(&RobotBuilder::setActuatorHolonomicSimple),
        py::return_value_policy::reference_internal,
        "Set a HolonomicSimple actuator with velocity limits.\n\n"
        "Args:\n"
        "    linear_x_min: Minimum X linear velocity\n"
        "    linear_x_max: Maximum X linear velocity\n"
        "    linear_y_max: Maximum Y linear velocity\n"
        "    angular_max: Maximum angular velocity\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("linear_x_min"), py::arg("linear_x_max"), py::arg("linear_y_max"), py::arg("angular_max"));
    pyBuilder.def("set_actuator_holonomic_simple",
        py::overload_cast<float, float, float, float, float, float>(&RobotBuilder::setActuatorHolonomicSimple),
        py::return_value_policy::reference_internal,
        "Set a HolonomicSimple actuator with velocity and acceleration limits.\n\n"
        "Args:\n"
        "    linear_x_min: Minimum X linear velocity\n"
        "    linear_x_max: Maximum X linear velocity\n"
        "    linear_y_max: Maximum Y linear velocity\n"
        "    angular_max: Maximum angular velocity\n"
        "    acceleration_linear: Maximum linear acceleration\n"
        "    acceleration_angular: Maximum angular acceleration\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("linear_x_min"), py::arg("linear_x_max"), py::arg("linear_y_max"), py::arg("angular_max"),
        py::arg("acceleration_linear"), py::arg("acceleration_angular"));

    // --- Shape setters ---

    pyBuilder.def("set_shape_robot",
        py::overload_cast<const murosim::geometry::Shape&>(&RobotBuilder::setShapeRobot),
        py::return_value_policy::reference_internal,
        "Set the robot's collision shape (copied).\n\n"
        "Args:\n"
        "    shape: Collision shape, in the robot's local frame\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("shape"));
    pyBuilder.def("set_shape_robot_rectangle", &RobotBuilder::setShapeRobotRectangle,
        py::return_value_policy::reference_internal,
        "Set the robot's collision shape to a centered rectangle.\n\n"
        "Args:\n"
        "    width: Width in metres\n"
        "    height: Height in metres\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("width"), py::arg("height"));
    pyBuilder.def("set_shape_robot_circle", &RobotBuilder::setShapeRobotCircle,
        py::return_value_policy::reference_internal,
        "Set the robot's collision shape to a centered circle.\n\n"
        "Args:\n"
        "    radius: Circle radius in metres\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("radius"));

    // --- Lidar ---

    pyBuilder.def("set_shape_lidar",
        &RobotBuilder::setShapeLidar,
        py::return_value_policy::reference_internal,
        "Set the shape other robots' lidars see this robot as (optional; defaults to the robot shape).\n"
        "An explicitly set empty shape makes the robot invisible to other lidars.\n\n"
        "Args:\n"
        "    shape: Lidar collision shape, in the robot's local frame\n\n"
        "Returns:\n"
        "    This builder, for method chaining.",
        py::arg("shape"));
    // addLidar(const Lidar&) is deliberately not bound: Lidar has no Python type.
    pyBuilder.def("add_lidar",
        py::overload_cast<const Eigen::Affine2f&, float, size_t, float, float, float, murosim::sensor::Lidar::NOISE_TYPE, float, float, bool>(&RobotBuilder::addLidar),
        py::return_value_policy::reference_internal,
        "Construct and add a lidar sensor in place.\n\n"
        "Args:\n"
        "    pose: Lidar mounting pose relative to the robot's local frame\n"
        "    fov: Field of view in radians; 2*pi spreads the rays evenly around\n"
        "        the circle, a smaller FOV measures both of its edges\n"
        "    resolution: Number of laser rays\n"
        "    range_min: Minimum measurable range in metres; closer hits read -inf\n"
        "    range_max: Maximum measurable range in metres; rays without a hit\n"
        "        within it read +inf\n"
        "    frequency: Update frequency in Hz\n"
        "    noise_type: Measurement noise model (default: LidarNoiseType.NONE)\n"
        "    noise: Noise magnitude in metres: the standard deviation for NORMAL,\n"
        "        the half-width for UNIFORM (default: 0.0)\n"
        "    angle_start: Start angle in radians (default: pi)\n"
        "    rotation_clockwise: Rotation direction (default: False)\n\n"
        "Returns:\n"
        "    This builder, for method chaining.\n\n"
        "Raises:\n"
        "    ValueError: If resolution is 0 or the range is not finite with\n"
        "        0 <= range_min < range_max.",
        py::arg("pose"), py::arg("fov"), py::arg("resolution"), py::arg("range_min"), py::arg("range_max"), py::arg("frequency"),
        py::arg("noise_type") = murosim::sensor::Lidar::NOISE_TYPE::NONE,
        py::arg("noise") = 0.0f,
        py::arg("angle_start") = murosim::math::PI,
        py::arg("rotation_clockwise") = false);

    // --- Build & Reset ---

    pyBuilder.def("build", &RobotBuilder::build,
        "Build a new Robot.\n\n"
        "Can be called repeatedly to spawn multiple robots from the same template.\n"
        "Throws RuntimeError if actuator or robot shape has not been set.\n\n"
        "Returns:\n"
        "    A new Robot instance");
    pyBuilder.def("reset", &RobotBuilder::reset,
        "Reset the builder, clearing all configuration.");

    // --- Repr ---

    pyBuilder.def("__repr__", [](const RobotBuilder &) {
        return "<RobotBuilder>";
    }, "Return a short debug string.");
}

} // namespace murosimpy
