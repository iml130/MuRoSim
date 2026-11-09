# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Core simulation components: Robot, Simulation, and ThreadPool.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.definitions
import murosimpy.geometry
import murosimpy.io
import murosimpy.navigation
import numpy
import numpy.typing
import typing
__all__: list[str] = ['LidarNoiseType', 'Robot', 'RobotBuilder', 'Simulation', 'ThreadPool']
class Robot:
    """
    Represents a robot agent in the simulation with sensors and actuators.
    """
    def __copy__(self) -> Robot:
        """
        Support for copy.copy(): returns a deep copy, same as clone().
        """
    def __deepcopy__(self, memo: dict) -> Robot:
        """
        Support for copy.deepcopy(): returns a deep copy, same as clone().
        """
    def __repr__(self) -> str:
        """
        Return a short debug string with the robot's name, position and finished status.
        """
    def clone(self) -> Robot:
        """
        Create a copy of this robot.
        """
    def lidar_distance(self, index: typing.SupportsInt | typing.SupportsIndex = 0, normalized: bool = False) -> numpy.typing.NDArray[numpy.float32]:
        """
        Get lidar distance readings as a numpy array.

        Readings follow ROS REP 117: +inf means no return within range_max,
        -inf a hit closer than range_min; finite readings lie in
        [range_min, range_max].

        Args:
            index: Lidar sensor index (default: 0)
            normalized: If True, clamp to [range_min, range_max] and map that
                range to [0, 1], so +inf reads 1 and -inf reads 0 (default: False)

        Returns:
            numpy array of distance readings.

        Raises:
            IndexError: If index is out of range.
        """
    @property
    def actuator_type(self) -> murosimpy.definitions.ActuatorType:
        """
        Type of actuator (ActuatorType enum).
        """
    @property
    def collision_status(self) -> murosimpy.definitions.CollisionStatus:
        """
        Current collision status (CollisionStatus enum).
        """
    @property
    def color_lidar(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Lidar rendering color as [r, g, b].
        """
    @color_lidar.setter
    def color_lidar(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def color_path(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Path rendering color as [r, g, b].
        """
    @color_path.setter
    def color_path(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def color_robot(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Robot rendering color as [r, g, b].
        """
    @color_robot.setter
    def color_robot(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def command_velocity(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Commanded velocity as [vx, vy, omega].
        """
    @command_velocity.setter
    def command_velocity(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def control_mode(self) -> murosimpy.definitions.ControlMode:
        """
        Actuator control mode (ControlMode enum): whether RL actions command
        a target velocity or an acceleration.
        """
    @control_mode.setter
    def control_mode(self, arg1: murosimpy.definitions.ControlMode) -> None:
        ...
    @property
    def finished(self) -> bool:
        """
        Whether the robot has reached its goal or is otherwise finished.
        """
    @property
    def goal(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Current goal as [x, y, radius] (the goal node's circle). Assigning
        preserves the goal's group, id, start angle/bias and goal radius.
        """
    @goal.setter
    def goal(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def lidar_count(self) -> int:
        """
        Number of lidar sensors on this robot.
        """
    @property
    def name(self) -> str:
        """
        Robot's unique name identifier.
        """
    @property
    def pose(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Current robot pose as [x, y, theta].
        """
    @pose.setter
    def pose(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def position(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[2, 1]"]:
        """
        Current robot position as [x, y].
        """
    @position.setter
    def position(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"]) -> None:
        ...
    @property
    def rotation(self) -> float:
        """
        Current robot rotation angle in radians.
        """
    @rotation.setter
    def rotation(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def state_status(self) -> murosimpy.definitions.RobotState:
        """
        Current state status (RobotState enum).
        """
    @property
    def steps(self) -> int:
        """
        Number of simulation steps this robot has executed.
        """
    @property
    def traveled_distance(self) -> float:
        """
        Cumulative distance traveled by the robot since the last reset.
        """
    @property
    def velocity(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Current actuator velocity as [vx, vy, omega].
        """
    @property
    def velocity_max(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Maximum actuator velocity as [vx_max, vy_max, omega_max].
        """
    @property
    def velocity_min(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Minimum actuator velocity as [vx_min, vy_min, omega_min].
        """
class LidarNoiseType:
    """
    Lidar measurement noise model.

    Members:

      NONE : No noise

      NORMAL : Gaussian noise

      UNIFORM : Uniform noise
    """
    NONE: typing.ClassVar[LidarNoiseType]
    NORMAL: typing.ClassVar[LidarNoiseType]
    UNIFORM: typing.ClassVar[LidarNoiseType]
    __members__: typing.ClassVar[dict[str, LidarNoiseType]]
    @typing.overload
    def __eq__(self, other: LidarNoiseType) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: LidarNoiseType) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class RobotBuilder:
    """
    Builder for creating Robot instances with a fluent API.

    Configure once, then call build() repeatedly to spawn multiple robots
    from the same template. Throws RuntimeError if actuator or shape is missing.
    """
    def __init__(self) -> None:
        """
        Create an empty RobotBuilder.
        """
    def __repr__(self) -> str:
        """
        Return a short debug string.
        """
    def add_lidar(self, pose: murosimpy.geometry.Transform, fov: typing.SupportsFloat | typing.SupportsIndex, resolution: typing.SupportsInt | typing.SupportsIndex, range_min: typing.SupportsFloat | typing.SupportsIndex, range_max: typing.SupportsFloat | typing.SupportsIndex, frequency: typing.SupportsFloat | typing.SupportsIndex, noise_type: LidarNoiseType = LidarNoiseType.NONE, noise: typing.SupportsFloat | typing.SupportsIndex = 0.0, angle_start: typing.SupportsFloat | typing.SupportsIndex = 3.1415927410125732, rotation_clockwise: bool = False) -> RobotBuilder:
        """
        Construct and add a lidar sensor in place.

        Args:
            pose: Lidar mounting pose relative to the robot's local frame
            fov: Field of view in radians; 2*pi spreads the rays evenly around
                the circle, a smaller FOV measures both of its edges
            resolution: Number of laser rays
            range_min: Minimum measurable range in metres; closer hits read -inf
            range_max: Maximum measurable range in metres; rays without a hit
                within it read +inf
            frequency: Update frequency in Hz
            noise_type: Measurement noise model (default: LidarNoiseType.NONE)
            noise: Noise magnitude in metres: the standard deviation for NORMAL,
                the half-width for UNIFORM (default: 0.0)
            angle_start: Start angle in radians (default: pi)
            rotation_clockwise: Rotation direction (default: False)

        Returns:
            This builder, for method chaining.

        Raises:
            ValueError: If resolution is 0 or the range is not finite with
                0 <= range_min < range_max.
        """
    def build(self) -> Robot:
        """
        Build a new Robot.

        Can be called repeatedly to spawn multiple robots from the same template.
        Throws RuntimeError if actuator or robot shape has not been set.

        Returns:
            A new Robot instance
        """
    def reset(self) -> None:
        """
        Reset the builder, clearing all configuration.
        """
    @typing.overload
    def set_actuator_differential_simple(self) -> RobotBuilder:
        """
        Set a default DifferentialSimple actuator (no velocity/acceleration limits configured).

        Returns:
            This builder, for method chaining.
        """
    @typing.overload
    def set_actuator_differential_simple(self, velocity_linear_min: typing.SupportsFloat | typing.SupportsIndex, velocity_linear_max: typing.SupportsFloat | typing.SupportsIndex, velocity_angular_max: typing.SupportsFloat | typing.SupportsIndex) -> RobotBuilder:
        """
        Set a DifferentialSimple actuator with velocity limits.

        Args:
            velocity_linear_min: Minimum linear velocity
            velocity_linear_max: Maximum linear velocity
            velocity_angular_max: Maximum angular velocity

        Returns:
            This builder, for method chaining.
        """
    @typing.overload
    def set_actuator_differential_simple(self, velocity_linear_min: typing.SupportsFloat | typing.SupportsIndex, velocity_linear_max: typing.SupportsFloat | typing.SupportsIndex, velocity_angular_max: typing.SupportsFloat | typing.SupportsIndex, acceleration_linear: typing.SupportsFloat | typing.SupportsIndex, acceleration_angular: typing.SupportsFloat | typing.SupportsIndex) -> RobotBuilder:
        """
        Set a DifferentialSimple actuator with velocity and acceleration limits.

        Args:
            velocity_linear_min: Minimum linear velocity
            velocity_linear_max: Maximum linear velocity
            velocity_angular_max: Maximum angular velocity
            acceleration_linear: Maximum linear acceleration
            acceleration_angular: Maximum angular acceleration

        Returns:
            This builder, for method chaining.
        """
    @typing.overload
    def set_actuator_holonomic_simple(self) -> RobotBuilder:
        """
        Set a default HolonomicSimple actuator (no velocity/acceleration limits configured).

        Returns:
            This builder, for method chaining.
        """
    @typing.overload
    def set_actuator_holonomic_simple(self, linear_x_min: typing.SupportsFloat | typing.SupportsIndex, linear_x_max: typing.SupportsFloat | typing.SupportsIndex, linear_y_max: typing.SupportsFloat | typing.SupportsIndex, angular_max: typing.SupportsFloat | typing.SupportsIndex) -> RobotBuilder:
        """
        Set a HolonomicSimple actuator with velocity limits.

        Args:
            linear_x_min: Minimum X linear velocity
            linear_x_max: Maximum X linear velocity
            linear_y_max: Maximum Y linear velocity
            angular_max: Maximum angular velocity

        Returns:
            This builder, for method chaining.
        """
    @typing.overload
    def set_actuator_holonomic_simple(self, linear_x_min: typing.SupportsFloat | typing.SupportsIndex, linear_x_max: typing.SupportsFloat | typing.SupportsIndex, linear_y_max: typing.SupportsFloat | typing.SupportsIndex, angular_max: typing.SupportsFloat | typing.SupportsIndex, acceleration_linear: typing.SupportsFloat | typing.SupportsIndex, acceleration_angular: typing.SupportsFloat | typing.SupportsIndex) -> RobotBuilder:
        """
        Set a HolonomicSimple actuator with velocity and acceleration limits.

        Args:
            linear_x_min: Minimum X linear velocity
            linear_x_max: Maximum X linear velocity
            linear_y_max: Maximum Y linear velocity
            angular_max: Maximum angular velocity
            acceleration_linear: Maximum linear acceleration
            acceleration_angular: Maximum angular acceleration

        Returns:
            This builder, for method chaining.
        """
    def set_name(self, name: str) -> RobotBuilder:
        """
        Set the robot name.

        Returns:
            This builder (bound to the same lifetime as self), for method chaining.
        """
    def set_shape_lidar(self, shape: murosimpy.geometry.Shape) -> RobotBuilder:
        """
        Set the shape other robots' lidars see this robot as (optional; defaults to the robot shape).
        An explicitly set empty shape makes the robot invisible to other lidars.

        Args:
            shape: Lidar collision shape, in the robot's local frame

        Returns:
            This builder, for method chaining.
        """
    def set_shape_robot(self, shape: murosimpy.geometry.Shape) -> RobotBuilder:
        """
        Set the robot's collision shape (copied).

        Args:
            shape: Collision shape, in the robot's local frame

        Returns:
            This builder, for method chaining.
        """
    def set_shape_robot_circle(self, radius: typing.SupportsFloat | typing.SupportsIndex) -> RobotBuilder:
        """
        Set the robot's collision shape to a centered circle.

        Args:
            radius: Circle radius in metres

        Returns:
            This builder, for method chaining.
        """
    def set_shape_robot_rectangle(self, width: typing.SupportsFloat | typing.SupportsIndex, height: typing.SupportsFloat | typing.SupportsIndex) -> RobotBuilder:
        """
        Set the robot's collision shape to a centered rectangle.

        Args:
            width: Width in metres
            height: Height in metres

        Returns:
            This builder, for method chaining.
        """
class Simulation:
    """
    Owns and advances one multi-robot scenario.

    Holds the static world, the navigation graph, the derived occupancy grid,
    the set of robots, and the per-simulation random generator and planner.
    step() advances the whole scenario by one SimulationConfig.time_step,
    handling actuation, collision detection, lidar updates, goal bookkeeping
    and (re-)initialisation.

    Not thread-safe: concurrent simulations must each own their own instance
    (see ThreadPool for stepping many simulations in parallel).
    """
    def __copy__(self) -> Simulation:
        """
        Support for copy.copy(): returns a deep copy, same as clone().
        """
    def __deepcopy__(self, memo: dict) -> Simulation:
        """
        Support for copy.deepcopy(): returns a deep copy, same as clone().
        """
    def __getitem__(self, index: typing.SupportsInt | typing.SupportsIndex) -> Robot:
        """
        Get a robot by index.

        Raises:
            IndexError: If index is out of range.
        """
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, grid_scale: typing.SupportsFloat | typing.SupportsIndex = 0.10000000149011612, buffer_scale: typing.SupportsFloat | typing.SupportsIndex = 0.20000000298023224) -> None:
        """
        Create a simulation with an empty world.

        Args:
            scale: World scale factor (default: 1.0)
            grid_scale: Grid resolution in meters per cell (default: 0.1)
            buffer_scale: Safety buffer around obstacles in meters (default: 0.2)
        """
    @typing.overload
    def __init__(self, worldfile: str, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, grid_scale: typing.SupportsFloat | typing.SupportsIndex = 0.10000000149011612, buffer_scale: typing.SupportsFloat | typing.SupportsIndex = 0.20000000298023224, planner_type: murosimpy.definitions.PlannerType = murosimpy.definitions.PlannerType.NONE) -> None:
        """
        Load a simulation from a world file.

        Args:
            worldfile: Path to the world file
            scale: World scale factor (default: 1.0)
            grid_scale: Grid resolution in meters per cell (default: 0.1)
            buffer_scale: Safety buffer around obstacles (default: 0.2)
            planner_type: Path planner type (default: PlannerType.NONE)
        """
    @typing.overload
    def __init__(self, data: murosimpy.io.DataContainer, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, grid_scale: typing.SupportsFloat | typing.SupportsIndex = 0.10000000149011612, buffer_scale: typing.SupportsFloat | typing.SupportsIndex = 0.20000000298023224, planner_type: murosimpy.definitions.PlannerType = murosimpy.definitions.PlannerType.NONE) -> None:
        """
        Create a simulation from a DataContainer.

        Args:
            data: DataContainer with world geometry and nodes
            scale: World scale factor (default: 1.0)
            grid_scale: Grid resolution in meters per cell (default: 0.1)
            buffer_scale: Safety buffer around obstacles (default: 0.2)
            planner_type: Path planner type (default: PlannerType.NONE)
        """
    @typing.overload
    def __init__(self, world: murosimpy.geometry.World, nodes: murosimpy.navigation.NodeContainer, name: str = '', grid_scale: typing.SupportsFloat | typing.SupportsIndex = 0.10000000149011612, buffer_scale: typing.SupportsFloat | typing.SupportsIndex = 0.20000000298023224) -> None:
        """
        Create a simulation from World and NodeContainer objects.

        Args:
            world: World geometry
            nodes: Navigation nodes
            name: Simulation name (default: empty)
            grid_scale: Grid resolution in meters per cell (default: 0.1)
            buffer_scale: Safety buffer around obstacles (default: 0.2)
        """
    @typing.overload
    def __init__(self, data: murosimpy.io.DataContainer, config: murosimpy.definitions.SimulationConfig) -> None:
        """
        Create a simulation from a DataContainer with SimulationConfig.

        Args:
            data: DataContainer with world geometry and nodes
            config: SimulationConfig with all simulation parameters
        """
    @typing.overload
    def __init__(self, world: murosimpy.geometry.World, nodes: murosimpy.navigation.NodeContainer, name: str, config: murosimpy.definitions.SimulationConfig) -> None:
        """
        Create a simulation from World and NodeContainer with SimulationConfig.

        Args:
            world: World geometry
            nodes: Navigation nodes
            name: Simulation name
            config: SimulationConfig with all simulation parameters
        """
    def __len__(self) -> int:
        """
        Return the number of robots.
        """
    def __repr__(self) -> str:
        """
        Return a short debug string with the simulation's name, robot count and step count.
        """
    def add_robot(self, robot: Robot) -> None:
        """
        Add a robot to the simulation.

        Raises:
            RuntimeError: If the robot is already in the simulation.
        """
    def clone(self) -> Simulation:
        """
        Create a deep copy of this simulation.
        """
    def get_map(self, with_buffer: bool = True) -> numpy.typing.NDArray[numpy.uint8]:
        """
        Get the occupancy grid map as a numpy array.

        Args:
            with_buffer: Include safety buffer around obstacles (default: True)

        Returns:
            2D numpy array of uint8 occupancy values.
        """
    def get_robot(self, index: typing.SupportsInt | typing.SupportsIndex) -> Robot:
        """
        Get a robot by index.

        Raises:
            IndexError: If index is out of range.
        """
    def planning(self) -> None:
        """
        Execute path planning for all robots.
        """
    def remove_all_robots(self) -> None:
        """
        Remove all robots from the simulation.
        """
    def reset(self) -> None:
        """
        Reset every robot (including its lidars), re-initialise robot positions
        per the configured InitMode and run one step() to populate the initial
        lidar/state, then rewind the step counter to 0. In CONTINUOUS sensor
        mode every lidar first takes a complete scan of the new pose, so no
        reading from before the reset survives into the episode.

        Raises:
            RuntimeError: If position initialisation fails.
        """
    def set_config(self, arg0: murosimpy.definitions.SimulationConfig) -> None:
        """
        Set the simulation config and rebuild the state derived from it
        (grid map, random generator seed, planner). Robot positions are
        re-initialised by a reset on the next simulate() call.
        """
    def step(self) -> None:
        """
        Advance the simulation by one time step (SimulationConfig.time_step
        seconds), sub-stepping actuators/collision/sensors at their configured
        intervals. If the simulation is not ready (e.g. just constructed or
        after set_config()), calls reset() instead, which itself performs one step.
        """
    @property
    def config(self) -> murosimpy.definitions.SimulationConfig:
        """
        Copy of the SimulationConfig. Use set_config() to modify the simulation.
        """
    @property
    def finished(self) -> bool:
        """
        Whether all robots have finished (reached goals or failed).
        """
    @property
    def max_robots(self) -> int:
        """
        Maximum number of robots that can be added.
        """
    @property
    def name(self) -> str:
        """
        Simulation name identifier.
        """
    @property
    def nodes(self) -> murosimpy.navigation.NodeContainer:
        """
        Navigation nodes for goal assignment.
        """
    @property
    def robot_count(self) -> int:
        """
        Number of robots currently in the simulation.
        """
    @property
    def robots(self) -> murosimpy.containers.Robots:
        """
        List of all robots in the simulation.
        """
    @property
    def steps(self) -> int:
        """
        Number of simulation steps executed.
        """
    @property
    def world(self) -> murosimpy.geometry.World:
        """
        World geometry containing lines and circles.
        """
class ThreadPool:
    """
    Fixed-size pool of worker threads for stepping many simulations in parallel.

    Each execute_* method blocks the calling thread until every job it queued
    has run (or one has thrown, in which case the exception is re-raised after
    all jobs finish). The GIL is released while jobs run, so multiple Python
    threads calling into the pool do not serialize on it. Not copyable; the
    pool's C++ destructor stops and joins all worker threads.
    """
    def __init__(self, number_of_threads: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Create a thread pool with specified number of threads.

        Args:
            number_of_threads: Number of worker threads (0 = auto-detect)
        """
    def __repr__(self) -> str:
        """
        Return a short debug string with the number of worker threads.
        """
    def execute_planning(self, simulations: murosimpy.containers.Simulations) -> None:
        """
        Execute path planning on all simulations in parallel.

        Args:
            simulations: List of simulations to run planning on
        """
    def execute_reset(self, simulations: murosimpy.containers.Simulations) -> None:
        """
        Reset all simulations in parallel.

        Args:
            simulations: List of simulations to reset
        """
    def execute_reset_by_finished(self, simulations: murosimpy.containers.Simulations, steps: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Reset simulations that are finished or have exceeded step limit.

        Args:
            simulations: List of simulations to potentially reset
            steps: Maximum step count before reset (0 = only reset finished)
        """
    def execute_simulate(self, simulations: murosimpy.containers.Simulations) -> None:
        """
        Execute simulation step on all simulations in parallel.

        Args:
            simulations: List of simulations to step
        """
    @property
    def thread_count(self) -> int:
        """
        Number of worker threads in the pool.
        """
