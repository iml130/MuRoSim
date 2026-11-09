// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "common.hpp"

#include "murosim/robot/robot.hpp"
#include "murosim/actuator/actuator.hpp"
#include "murosim/actuator/robomaster.hpp"
#include "murosim/simulation/simulation.hpp"
#include "murosim/simulation/simulation_config.hpp"
#include "murosim/planning/planner.hpp"

#include <sstream>

namespace murosimpy {

void initModuleDefinitions(pybind11::module &m)
{
    // Enum: CollisionStatus
    pybind11::enum_<murosim::robot::Robot::STATUS_COLLISION>(m, "CollisionStatus",
        "Robot collision status.")
        .value("NONE", murosim::robot::Robot::STATUS_COLLISION::NONE,
            "No collision detected.")
        .value("WORLD", murosim::robot::Robot::STATUS_COLLISION::WORLD,
            "Collision with static world geometry.")
        .value("DYNAMIC_ENTITY", murosim::robot::Robot::STATUS_COLLISION::DYNAMIC_ENTITY,
            "Collision with a dynamic entity.")
        .value("ROBOT", murosim::robot::Robot::STATUS_COLLISION::ROBOT,
            "Collision with another robot.")
        .export_values();

    // Enum: RobotState
    pybind11::enum_<murosim::robot::Robot::STATUS_STATE>(m, "RobotState",
        "Robot state status.")
        .value("IDLE", murosim::robot::Robot::STATUS_STATE::IDLE,
            "Robot is idle.")
        .value("SEARCH_GOAL", murosim::robot::Robot::STATUS_STATE::SEARCH_GOAL,
            "Robot is searching for a goal.")
        .value("ON_GOAL", murosim::robot::Robot::STATUS_STATE::ON_GOAL,
            "Robot is on the goal.")
        .value("REACHED_GOAL", murosim::robot::Robot::STATUS_STATE::REACHED_GOAL,
            "Robot has reached the goal.")
        .value("UNUSED", murosim::robot::Robot::STATUS_STATE::UNUSED,
            "Robot state is unused.")
        .export_values();

    // Enum: ActuatorType
    pybind11::enum_<murosim::actuator::Actuator::TYPE>(m, "ActuatorType",
        "Robot actuator type.")
        .value("X_ANGULAR", murosim::actuator::Actuator::TYPE::X_ANGULAR,
            "Differential drive (forward velocity + angular velocity).")
        .value("X_Y_ANGULAR", murosim::actuator::Actuator::TYPE::X_Y_ANGULAR,
            "Omnidirectional drive (x, y velocity + angular velocity).")
        .value("X_Y", murosim::actuator::Actuator::TYPE::X_Y,
            "Holonomic drive (x, y velocity only).")
        .export_values();

    // Enum: ControlMode
    pybind11::enum_<murosim::actuator::CONTROL_MODE>(m, "ControlMode",
        "Control mode for actuator commands. Determines what an RL action\n"
        "commands: a target velocity or an acceleration.")
        .value("VELOCITY", murosim::actuator::CONTROL_MODE::VELOCITY,
            "Commands set a target velocity; the actuator accelerates toward it\n"
            "using its acceleration limits.")
        .value("ACCELERATION", murosim::actuator::CONTROL_MODE::ACCELERATION,
            "Commands set an acceleration, integrated each timestep and clamped\n"
            "to the velocity limits. Requires acceleration limits on all actuated axes.")
        .export_values();

    // Hardware limits of the RoboMaster base, reachable through models.robot.robomaster()
    pybind11::module robomaster = m.def_submodule("robomaster",
        "Hardware limits of the DJI RoboMaster base used by models.robot.robomaster().\n\n"
        "VELOCITY_* are the defaults of the actuator. The wheels cannot exceed\n"
        "WHEEL_RPM_MAX and need TIME_TO_MAX_RPM seconds to get there from standstill,\n"
        "independent of the acceleration limits passed to robomaster(); motion is\n"
        "integrated in sub-steps of SAMPLE_TIME seconds regardless of the simulation\n"
        "time step.");
    robomaster.attr("VELOCITY_LINEAR_MAX") = murosim::actuator::RoboMaster::VELOCITY_LINEAR_MAX;
    robomaster.attr("VELOCITY_ANGULAR_MAX") = murosim::actuator::RoboMaster::VELOCITY_ANGULAR_MAX;
    robomaster.attr("WHEEL_RPM_MAX") = murosim::actuator::RoboMaster::WHEEL_RPM_MAX;
    robomaster.attr("TIME_TO_MAX_RPM") = murosim::actuator::RoboMaster::TIME_TO_MAX_RPM;
    robomaster.attr("SAMPLE_TIME") = murosim::actuator::RoboMaster::SAMPLE_TIME;

    // Enum: InitMode
    pybind11::enum_<murosim::simulation::Simulation::INIT_MODE>(m, "InitMode",
        "Robot initialization mode for simulation reset.")
        .value("MANUAL", murosim::simulation::Simulation::INIT_MODE::MANUAL,
            "Robots keep their current positions.")
        .value("FREE_ALL_RANDOM", murosim::simulation::Simulation::INIT_MODE::FREE_ALL_RANDOM,
            "Robots are placed randomly in free space.")
        .value("NODE_ALL_RANDOM", murosim::simulation::Simulation::INIT_MODE::NODE_ALL_RANDOM,
            "Robots are placed randomly on any node.")
        .value("NODE_GROUP_RANDOM", murosim::simulation::Simulation::INIT_MODE::NODE_GROUP_RANDOM,
            "Robots are placed randomly within their assigned node group.")
        .export_values();

    // Enum: SensorUpdateMode
    pybind11::enum_<murosim::simulation::SimulationConfig::SENSOR_UPDATE_MODE>(m, "SensorUpdateMode",
        "Sensor update mode for simulation.\n\n"
        "Determines how sensors (e.g., lidar) are updated during simulation steps.")
        .value("SYNCHRONIZED", murosim::simulation::SimulationConfig::SENSOR_UPDATE_MODE::SYNCHRONIZED,
            "All sensors update synchronized at end of simulation step (full sweep at once).")
        .value("CONTINUOUS", murosim::simulation::SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS,
            "Sensors update continuously during simulation step (progressive scanning).")
        .export_values();

    // Bound before SimulationConfig: its planner_type property uses this enum.
    // Enum: PlannerType
    pybind11::enum_<murosim::planning::Planner::TYPE>(m, "PlannerType",
        "Path planning algorithm type.")
        .value("NONE", murosim::planning::Planner::TYPE::NONE,
            "No path planning.")
        .value("ASTAR4", murosim::planning::Planner::TYPE::ASTAR4,
            "A* with 4-connected grid (cardinal directions only).")
        .value("ASTAR8", murosim::planning::Planner::TYPE::ASTAR8,
            "A* with 8-connected grid (includes diagonals).")
        .value("VORONOI", murosim::planning::Planner::TYPE::VORONOI,
            "Voronoi-based path planning.")
        .export_values();

    // Class: SimulationConfig
    pybind11::class_<murosim::simulation::SimulationConfig>(m, "SimulationConfig",
        "Configuration parameters for Simulation.\n\n"
        "This class holds all configurable parameters for creating and running a simulation.\n"
        "All parameters are validated when set (invalid values raise ValueError) to ensure\n"
        "simulation stability. It can be reused across multiple simulation instances, making\n"
        "it ideal for factory patterns and batch simulation creation.")
        .def(pybind11::init<>(),
            "Create a SimulationConfig with default values and a random seed.")
        .def_property("time_step",
            &murosim::simulation::SimulationConfig::getTimeStep,
            &murosim::simulation::SimulationConfig::setTimeStep,
            "Duration of each simulation step [seconds] (default: 0.1).\n"
            "Raises ValueError if not positive.")
        .def_property("time_interval_collision",
            &murosim::simulation::SimulationConfig::getTimeIntervalCollision,
            &murosim::simulation::SimulationConfig::setTimeIntervalCollision,
            "Sub-step interval [seconds] between collision checks (default: 0.01,\n"
            "minimum: 1e-6). Raises ValueError if set below the minimum.")
        .def_property("time_interval_actuator",
            &murosim::simulation::SimulationConfig::getTimeIntervalActuator,
            &murosim::simulation::SimulationConfig::setTimeIntervalActuator,
            "Sub-step interval [seconds] between actuator updates (default: 0.01,\n"
            "minimum: 1e-6). Raises ValueError if set below the minimum.")
        .def_property("time_interval_sensor",
            &murosim::simulation::SimulationConfig::getTimeIntervalSensor,
            &murosim::simulation::SimulationConfig::setTimeIntervalSensor,
            "Sub-step interval [seconds] between sensor updates in CONTINUOUS mode\n"
            "(default: 0.01, minimum: 1e-6). Raises ValueError if set below the minimum.")
        .def_property("time_on_goal",
            &murosim::simulation::SimulationConfig::getTimeOnGoal,
            &murosim::simulation::SimulationConfig::setTimeOnGoal,
            "Seconds a robot must dwell on its goal before being considered\n"
            "REACHED_GOAL (default: -1.0, meaning instant finish).")
        .def_property("latch_goal_timer",
            &murosim::simulation::SimulationConfig::getPersistentGoalTimer,
            &murosim::simulation::SimulationConfig::setPersistentGoalTimer,
            "If True, the on-goal timer keeps counting after the robot leaves the\n"
            "goal radius instead of resetting to 0 (default: False).")
        .def_property("grid_scale",
            &murosim::simulation::SimulationConfig::getGridScale,
            &murosim::simulation::SimulationConfig::setGridScale,
            "Metres per occupancy grid cell (default: 0.1, minimum: 0.01).\n"
            "Raises ValueError if set below the minimum.")
        .def_property("buffer_scale",
            &murosim::simulation::SimulationConfig::getBufferScale,
            &murosim::simulation::SimulationConfig::setBufferScale,
            "Extra clearance [metres] added around obstacles when building the\n"
            "occupancy grid (default: 0.2). Raises ValueError if negative.")
        .def_property("goal_radius",
            &murosim::simulation::SimulationConfig::getGoalRadius,
            &murosim::simulation::SimulationConfig::setGoalRadius,
            "Goal acceptance radius in metres; -1.0 means derive it automatically\n"
            "(e.g. from the goal node's radius) (default: -1.0). Must be either\n"
            "exactly -1.0 or a positive value; any other value raises ValueError.")
        .def_property("use_new_goal",
            &murosim::simulation::SimulationConfig::getUseNewGoal,
            &murosim::simulation::SimulationConfig::setUseNewGoal,
            "Whether a robot that reaches its goal is assigned a new one instead\n"
            "of finishing (default: False).")
        .def_property("init_mode",
            &murosim::simulation::SimulationConfig::getInitMode,
            &murosim::simulation::SimulationConfig::setInitMode,
            "How robot start/goal positions are (re-)chosen on reset() (InitMode\n"
            "enum, default: InitMode.MANUAL).")
        .def_property("sensor_update_mode",
            &murosim::simulation::SimulationConfig::getSensorUpdateMode,
            &murosim::simulation::SimulationConfig::setSensorUpdateMode,
            "When lidar sensors are updated relative to the simulation step loop\n"
            "(SensorUpdateMode enum, default: SensorUpdateMode.SYNCHRONIZED).")
        .def_property("planner_type",
            &murosim::simulation::SimulationConfig::getPlannerType,
            &murosim::simulation::SimulationConfig::setPlannerType,
            "Type of path planner to use (PlannerType enum, default: PlannerType.NONE).")
        .def_property("seed",
            &murosim::simulation::SimulationConfig::getSeed,
            &murosim::simulation::SimulationConfig::setSeed,
            "Random seed used to construct the simulation's random generator\n"
            "(default: a hardware-derived random value).")
        .def("__repr__", [](const murosim::simulation::SimulationConfig &config) {
            std::ostringstream oss;
            oss << "<SimulationConfig time_step=" << config.getTimeStep()
                << " init_mode=" << static_cast<int>(config.getInitMode())
                << " seed=" << config.getSeed() << ">";
            return oss.str();
        }, "Return a short debug string with the time step, init mode and seed.");

}

} // namespace murosimpy
