# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Enum definitions for robot states, collision status, actuator types, etc.
"""
from __future__ import annotations
import typing
from . import robomaster
__all__: list[str] = ['ACCELERATION', 'ASTAR4', 'ASTAR8', 'ActuatorType', 'CONTINUOUS', 'CollisionStatus', 'ControlMode', 'DYNAMIC_ENTITY', 'FREE_ALL_RANDOM', 'IDLE', 'InitMode', 'MANUAL', 'NODE_ALL_RANDOM', 'NODE_GROUP_RANDOM', 'NONE', 'ON_GOAL', 'PlannerType', 'REACHED_GOAL', 'ROBOT', 'RobotState', 'SEARCH_GOAL', 'SYNCHRONIZED', 'SensorUpdateMode', 'SimulationConfig', 'UNUSED', 'VELOCITY', 'VORONOI', 'WORLD', 'X_ANGULAR', 'X_Y', 'X_Y_ANGULAR', 'robomaster']
class CollisionStatus:
    """
    Robot collision status.

    Members:

      NONE : No collision detected.

      WORLD : Collision with static world geometry.

      DYNAMIC_ENTITY : Collision with a dynamic entity.

      ROBOT : Collision with another robot.
    """
    DYNAMIC_ENTITY: typing.ClassVar[CollisionStatus]
    NONE: typing.ClassVar[CollisionStatus]
    ROBOT: typing.ClassVar[CollisionStatus]
    WORLD: typing.ClassVar[CollisionStatus]
    __members__: typing.ClassVar[dict[str, CollisionStatus]]
    @typing.overload
    def __eq__(self, other: CollisionStatus) -> bool:
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
    def __ne__(self, other: CollisionStatus) -> bool:
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
class RobotState:
    """
    Robot state status.

    Members:

      IDLE : Robot is idle.

      SEARCH_GOAL : Robot is searching for a goal.

      ON_GOAL : Robot is on the goal.

      REACHED_GOAL : Robot has reached the goal.

      UNUSED : Robot state is unused.
    """
    IDLE: typing.ClassVar[RobotState]
    ON_GOAL: typing.ClassVar[RobotState]
    REACHED_GOAL: typing.ClassVar[RobotState]
    SEARCH_GOAL: typing.ClassVar[RobotState]
    UNUSED: typing.ClassVar[RobotState]
    __members__: typing.ClassVar[dict[str, RobotState]]
    @typing.overload
    def __eq__(self, other: RobotState) -> bool:
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
    def __ne__(self, other: RobotState) -> bool:
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
class ActuatorType:
    """
    Robot actuator type.

    Members:

      X_ANGULAR : Differential drive (forward velocity + angular velocity).

      X_Y_ANGULAR : Omnidirectional drive (x, y velocity + angular velocity).

      X_Y : Holonomic drive (x, y velocity only).
    """
    X_ANGULAR: typing.ClassVar[ActuatorType]
    X_Y: typing.ClassVar[ActuatorType]
    X_Y_ANGULAR: typing.ClassVar[ActuatorType]
    __members__: typing.ClassVar[dict[str, ActuatorType]]
    @typing.overload
    def __eq__(self, other: ActuatorType) -> bool:
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
    def __ne__(self, other: ActuatorType) -> bool:
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
class ControlMode:
    """
    Control mode for actuator commands. Determines what an RL action
    commands: a target velocity or an acceleration.

    Members:

      VELOCITY : Commands set a target velocity; the actuator accelerates toward it
    using its acceleration limits.

      ACCELERATION : Commands set an acceleration, integrated each timestep and clamped
    to the velocity limits. Requires acceleration limits on all actuated axes.
    """
    ACCELERATION: typing.ClassVar[ControlMode]
    VELOCITY: typing.ClassVar[ControlMode]
    __members__: typing.ClassVar[dict[str, ControlMode]]
    @typing.overload
    def __eq__(self, other: ControlMode) -> bool:
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
    def __ne__(self, other: ControlMode) -> bool:
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
class InitMode:
    """
    Robot initialization mode for simulation reset.

    Members:

      MANUAL : Robots keep their current positions.

      FREE_ALL_RANDOM : Robots are placed randomly in free space.

      NODE_ALL_RANDOM : Robots are placed randomly on any node.

      NODE_GROUP_RANDOM : Robots are placed randomly within their assigned node group.
    """
    FREE_ALL_RANDOM: typing.ClassVar[InitMode]
    MANUAL: typing.ClassVar[InitMode]
    NODE_ALL_RANDOM: typing.ClassVar[InitMode]
    NODE_GROUP_RANDOM: typing.ClassVar[InitMode]
    __members__: typing.ClassVar[dict[str, InitMode]]
    @typing.overload
    def __eq__(self, other: InitMode) -> bool:
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
    def __ne__(self, other: InitMode) -> bool:
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
class SensorUpdateMode:
    """
    Sensor update mode for simulation.

    Determines how sensors (e.g., lidar) are updated during simulation steps.

    Members:

      SYNCHRONIZED : All sensors update synchronized at end of simulation step (full sweep at once).

      CONTINUOUS : Sensors update continuously during simulation step (progressive scanning).
    """
    CONTINUOUS: typing.ClassVar[SensorUpdateMode]
    SYNCHRONIZED: typing.ClassVar[SensorUpdateMode]
    __members__: typing.ClassVar[dict[str, SensorUpdateMode]]
    @typing.overload
    def __eq__(self, other: SensorUpdateMode) -> bool:
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
    def __ne__(self, other: SensorUpdateMode) -> bool:
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
class PlannerType:
    """
    Path planning algorithm type.

    Members:

      NONE : No path planning.

      ASTAR4 : A* with 4-connected grid (cardinal directions only).

      ASTAR8 : A* with 8-connected grid (includes diagonals).

      VORONOI : Voronoi-based path planning.
    """
    ASTAR4: typing.ClassVar[PlannerType]
    ASTAR8: typing.ClassVar[PlannerType]
    NONE: typing.ClassVar[PlannerType]
    VORONOI: typing.ClassVar[PlannerType]
    __members__: typing.ClassVar[dict[str, PlannerType]]
    @typing.overload
    def __eq__(self, other: PlannerType) -> bool:
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
    def __ne__(self, other: PlannerType) -> bool:
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
class SimulationConfig:
    """
    Configuration parameters for Simulation.

    This class holds all configurable parameters for creating and running a simulation.
    All parameters are validated when set (invalid values raise ValueError) to ensure
    simulation stability. It can be reused across multiple simulation instances, making
    it ideal for factory patterns and batch simulation creation.
    """
    def __init__(self) -> None:
        """
        Create a SimulationConfig with default values and a random seed.
        """
    def __repr__(self) -> str:
        """
        Return a short debug string with the time step, init mode and seed.
        """
    @property
    def buffer_scale(self) -> float:
        """
        Extra clearance [metres] added around obstacles when building the
        occupancy grid (default: 0.2). Raises ValueError if negative.
        """
    @buffer_scale.setter
    def buffer_scale(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def goal_radius(self) -> float:
        """
        Goal acceptance radius in metres; -1.0 means derive it automatically
        (e.g. from the goal node's radius) (default: -1.0). Must be either
        exactly -1.0 or a positive value; any other value raises ValueError.
        """
    @goal_radius.setter
    def goal_radius(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def grid_scale(self) -> float:
        """
        Metres per occupancy grid cell (default: 0.1, minimum: 0.01).
        Raises ValueError if set below the minimum.
        """
    @grid_scale.setter
    def grid_scale(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def init_mode(self) -> InitMode:
        """
        How robot start/goal positions are (re-)chosen on reset() (InitMode
        enum, default: InitMode.MANUAL).
        """
    @init_mode.setter
    def init_mode(self, arg1: InitMode) -> None:
        ...
    @property
    def latch_goal_timer(self) -> bool:
        """
        If True, the on-goal timer keeps counting after the robot leaves the
        goal radius instead of resetting to 0 (default: False).
        """
    @latch_goal_timer.setter
    def latch_goal_timer(self, arg1: bool) -> None:
        ...
    @property
    def planner_type(self) -> PlannerType:
        """
        Type of path planner to use (PlannerType enum, default: PlannerType.NONE).
        """
    @planner_type.setter
    def planner_type(self, arg1: PlannerType) -> None:
        ...
    @property
    def seed(self) -> int:
        """
        Random seed used to construct the simulation's random generator
        (default: a hardware-derived random value).
        """
    @seed.setter
    def seed(self, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def sensor_update_mode(self) -> SensorUpdateMode:
        """
        When lidar sensors are updated relative to the simulation step loop
        (SensorUpdateMode enum, default: SensorUpdateMode.SYNCHRONIZED).
        """
    @sensor_update_mode.setter
    def sensor_update_mode(self, arg1: SensorUpdateMode) -> None:
        ...
    @property
    def time_interval_actuator(self) -> float:
        """
        Sub-step interval [seconds] between actuator updates (default: 0.01,
        minimum: 1e-6). Raises ValueError if set below the minimum.
        """
    @time_interval_actuator.setter
    def time_interval_actuator(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def time_interval_collision(self) -> float:
        """
        Sub-step interval [seconds] between collision checks (default: 0.01,
        minimum: 1e-6). Raises ValueError if set below the minimum.
        """
    @time_interval_collision.setter
    def time_interval_collision(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def time_interval_sensor(self) -> float:
        """
        Sub-step interval [seconds] between sensor updates in CONTINUOUS mode
        (default: 0.01, minimum: 1e-6). Raises ValueError if set below the minimum.
        """
    @time_interval_sensor.setter
    def time_interval_sensor(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def time_on_goal(self) -> float:
        """
        Seconds a robot must dwell on its goal before being considered
        REACHED_GOAL (default: -1.0, meaning instant finish).
        """
    @time_on_goal.setter
    def time_on_goal(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def time_step(self) -> float:
        """
        Duration of each simulation step [seconds] (default: 0.1).
        Raises ValueError if not positive.
        """
    @time_step.setter
    def time_step(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def use_new_goal(self) -> bool:
        """
        Whether a robot that reaches its goal is assigned a new one instead
        of finishing (default: False).
        """
    @use_new_goal.setter
    def use_new_goal(self, arg1: bool) -> None:
        ...
ACCELERATION: ControlMode
ASTAR4: PlannerType
ASTAR8: PlannerType
CONTINUOUS: SensorUpdateMode
DYNAMIC_ENTITY: CollisionStatus
FREE_ALL_RANDOM: InitMode
IDLE: RobotState
MANUAL: InitMode
NODE_ALL_RANDOM: InitMode
NODE_GROUP_RANDOM: InitMode
NONE: PlannerType
ON_GOAL: RobotState
REACHED_GOAL: RobotState
ROBOT: CollisionStatus
SEARCH_GOAL: RobotState
SYNCHRONIZED: SensorUpdateMode
UNUSED: RobotState
VELOCITY: ControlMode
VORONOI: PlannerType
WORLD: CollisionStatus
X_ANGULAR: ActuatorType
X_Y: ActuatorType
X_Y_ANGULAR: ActuatorType
