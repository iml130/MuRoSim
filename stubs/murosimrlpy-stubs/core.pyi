# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Core RL components: Action and Done.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.core
import numpy
import numpy.typing
import typing
__all__: list[str] = ['Action', 'Done']
class Action:
    """
    Applies RL actions (one float per actuated axis) to robots.

    The command written follows the robot's actuator control mode: a target
    velocity in VELOCITY mode, an acceleration in ACCELERATION mode. With
    normalize, the [-1, 1] action maps onto the matching range (the velocity
    limits, or the symmetric [-max, max] acceleration limits). The actuator's
    mode is the single source of truth - there is deliberately no mode flag
    here that could disagree with it.
    """
    @staticmethod
    def get_neutral_point(robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Return the action that commands zero (velocity or acceleration, per the robot's control mode), as a numpy array of size(robot) floats.
        """
    @staticmethod
    @typing.overload
    def shape(robot: murosimpy.core.Robot) -> list[int]:
        """
        Action shape for a single robot: (size(robot),).
        """
    @staticmethod
    @typing.overload
    def shape(robots: murosimpy.containers.Robots) -> list[int]:
        """
        Action shape for a robot list: (robots.size(), size(robots[0])), or empty if `robots` is empty.
        """
    @staticmethod
    def size(robot: murosimpy.core.Robot) -> int:
        """
        Number of actuated axes (action floats) for a robot's actuator type.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot, data: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> None:
        """
        Apply an action to a single robot.

        Args:
            robot: Robot to command.
            data: One float per actuated axis (see size()/shape()).
        """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots, data: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> None:
        """
        Apply one action per robot, packed contiguously. All robots must share one actuator type.

        Args:
            robots: Robots to command, in the same order as `data`.
            data: Concatenated per-robot actions, size(robots[0]) floats each.
        """
    def __copy__(self) -> Action:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> Action:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, normalize: bool = True) -> None:
        """
        Args:
            normalize: Interpret action values as [-1, 1] and map them onto the actuator's command range.
        """
    def normalize(self) -> bool:
        """
        Whether actions are interpreted as normalized [-1, 1] values.
        """
class Done:
    """
    Reports each robot's episode-termination flag.

    True once a robot has reached its goal or collided (Robot.is_finished()).
    One bool per robot, in robot order (or simulation then robot order, for
    the simulation overloads); no aggregation is done here (a caller wanting a
    per-simulation "all done" needs to combine these itself).
    """
    @staticmethod
    @typing.overload
    def shape(robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for a single robot: (1,).
        """
    @staticmethod
    @typing.overload
    def shape(robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for a robot list: (robots.size(), 1).
        """
    @staticmethod
    @typing.overload
    def shape(simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for a single simulation: (simulation's robot count, 1).
        """
    @staticmethod
    @typing.overload
    def shape(simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for a simulation list: (total robot count, 1).
        """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.bool_]:
        """
        Return each robot's is-finished flag, in robot order, as a boolean array.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.bool_]:
        """
        Return the robot's is-finished flag as a single-element boolean array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.bool_]:
        """
        Return each robot's is-finished flag, in simulation then robot order, as a boolean array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.bool_]:
        """
        Return each of the simulation's robots' is-finished flags, in robot order, as a boolean array.
        """
    def __copy__(self) -> Done:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> Done:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self) -> None:
        """
        Construct a Done check (stateless, no configuration).
        """
