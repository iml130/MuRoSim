# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Robot-level observations.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.core
import numpy
import numpy.typing
import typing
__all__: list[str] = ['CommandVelocity', 'DeltaPose', 'DistanceToGoal', 'Goal', 'Lidar', 'LidarPoint', 'ObservationRobot', 'OrientationToGoal', 'Pose', 'SubgoalDirection', 'Velocity']
class ObservationRobot:
    """
    Abstract base class for observations computed per robot (e.g. pose, velocity, lidar). Concrete subclasses can be called with a single robot, a list of robots, a single simulation, or a list of simulations (flattened to their combined robot list), and can be collected in an ObservationSet.
    """
class Pose(ObservationRobot):
    """
    Robot pose in world coordinates.

    Position (metres) plus orientation, either as an angle in radians or as a
    unit vector (cos, sin). Feature size is 3 per stacked state (x, y, angle),
    or 4 (x, y, cos, sin) when orientation_as_unit_vector is set. Values are
    not normalized.

    Args:
        orientation_as_unit_vector: Encode orientation as (cos, sin) instead
            of a single angle.
        stack_size: Number of stacked past poses per robot; 0 and 1 are
            equivalent (no stacking).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> Pose:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> Pose:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, orientation_as_unit_vector: bool = False, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            orientation_as_unit_vector: Encode orientation as (cos, sin) instead of a single angle.
            stack_size: Number of stacked past poses per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def orientation_as_unit_vector(self) -> bool:
        """
        Whether orientation is encoded as a (cos, sin) unit vector instead of an angle.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class Velocity(ObservationRobot):
    """
    The robot's actual (simulated) velocity.

    Reads the actuator velocity of the most recent (and stacked past) state(s).
    Feature size per stacked state equals the actuator's axis count: 2 for
    X_ANGULAR/X_Y actuators, 3 for X_Y_ANGULAR. With normalize, each value is
    scaled to [-1, 1] against the robot's own actuator velocity limits, so
    fleets sharing an actuator type but different limits still normalize
    correctly per robot.

    Args:
        normalize: Scale each value to [-1, 1] against the robot's own
            actuator velocity limits.
        stack_size: Number of stacked past values per robot; 0 and 1 are
            equivalent (no stacking).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> Velocity:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> Velocity:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, normalize: bool = False, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            normalize: Scale each value to [-1, 1] against the robot's own actuator velocity limits.
            stack_size: Number of stacked past values per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def normalize(self) -> bool:
        """
        Whether values are scaled to [-1, 1] against the robot's own velocity limits.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class Goal(ObservationRobot):
    """
    Robot's goal in world coordinates.

    Center (x, y) in metres and radius in metres, per stacked state. Feature
    size is 3 per stacked state. Not normalized.

    Args:
        stack_size: Number of stacked past goals per robot; 0 and 1 are
            equivalent (no stacking).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> Goal:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> Goal:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            stack_size: Number of stacked past goals per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class DistanceToGoal(ObservationRobot):
    """
    Euclidean distance from the robot's pose to its goal, in metres.

    Feature size is 1 per stacked state. If normalize_distance is positive,
    the value is divided by it (so a distance of normalize_distance maps to
    1.0); otherwise the raw metre distance is reported.

    Args:
        normalize_distance: Distance (metres) that maps to 1.0; 0 or negative
            disables normalization. Clamped to >= 0.
        stack_size: Number of stacked past distances per robot; 0 and 1 are
            equivalent (no stacking).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> DistanceToGoal:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> DistanceToGoal:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, normalize_distance: typing.SupportsFloat | typing.SupportsIndex = 0.0, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            normalize_distance: Distance (metres) that maps to 1.0; 0 or negative disables normalization. Clamped to >= 0.
            stack_size: Number of stacked past distances per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def normalize_distance(self) -> float:
        """
        The configured normalization distance, in metres (0 means "no normalization").
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class OrientationToGoal(ObservationRobot):
    """
    Direction from the robot towards its goal, in the robot's local frame.

    Reported as a 2D vector (x, y); feature size is 2 per stacked state. Unit
    length (i.e. each component in [-1, 1]) once the robot is further than
    threshold_radius from the goal; inside that radius the vector is scaled
    down linearly by distance / threshold_radius so it shrinks to zero exactly
    on the goal, avoiding a discontinuous direction flip right at the goal.

    Args:
        threshold_radius: Distance (metres) inside which the direction vector
            is scaled down towards zero. Clamped to >= 0.
        stack_size: Number of stacked past values per robot; 0 and 1 are
            equivalent (no stacking).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> OrientationToGoal:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> OrientationToGoal:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, threshold_radius: typing.SupportsFloat | typing.SupportsIndex = 0.0, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            threshold_radius: Distance (metres) inside which the direction vector is scaled down towards zero. Clamped to >= 0.
            stack_size: Number of stacked past values per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
    def threshold_radius(self) -> float:
        """
        The configured threshold radius, in metres.
        """
class CommandVelocity(ObservationRobot):
    """
    The velocity command last sent to the robot's actuator.

    Same feature shape, units and normalization as Velocity, but reads the
    commanded velocity instead of the actual (simulated) one.

    Args:
        normalize: Scale each value to [-1, 1] against the robot's own
            actuator velocity limits.
        stack_size: Number of stacked past values per robot; 0 and 1 are
            equivalent (no stacking).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> CommandVelocity:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> CommandVelocity:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, normalize: bool = False, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            normalize: Scale each value to [-1, 1] against the robot's own actuator velocity limits.
            stack_size: Number of stacked past values per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def normalize(self) -> bool:
        """
        Whether values are scaled to [-1, 1] against the robot's own velocity limits.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class DeltaPose(ObservationRobot):
    """
    Robot motion between consecutive recorded states.

    Expressed in the robot's own frame at the earlier of the two states:
    (dx, dy) translation in metres and rotation angle in radians. Feature size
    is 3 per stacked state. Not normalized. At the oldest recorded state, both
    reads clamp to it and the delta is zero.

    Args:
        stack_size: Number of stacked deltas per robot; 0 and 1 are equivalent
            (no stacking).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> DeltaPose:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> DeltaPose:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, stack_size: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Args:
            stack_size: Number of stacked deltas per robot; 0 and 1 are equivalent (no stacking).
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class Lidar(ObservationRobot):
    """
    Lidar range readings for one lidar sensor, in metres.

    Feature shape per stacked state is (num_lasers,). Readings are clamped to
    [lower, upper], where lower is the robot's own lidar range_min and upper is
    range_max if positive, else the robot's own lidar range_max; so +inf (no
    return) reads upper, -inf (too close) reads lower, and the output is always
    finite. With normalize, (reading - lower) / (upper - lower) maps lower to 0
    and upper to 1 per robot, so fleets with different lidar ranges still
    normalize to [0, 1]. All robots must have the same lidar configuration at
    index_lidar.

    Args:
        index_lidar: Index into each robot's lidar list.
        range_max: Upper clamp distance in metres; 0 uses each robot's own
            lidar range_max. Clamped to >= 0; a positive value must exceed the
            lidar's range_min.
        normalize: Scale readings to [0, 1]: the lidar's range_min maps to 0,
            the upper clamp distance to 1.
        stack_size: Number of stacked past readings per robot; 0 and 1 are
            equivalent (no stacking).
        num_threads: Worker threads used inside the direct compute path (not
            the ObservationSet range path); 0 uses hardware_concurrency().
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> Lidar:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> Lidar:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, index_lidar: typing.SupportsInt | typing.SupportsIndex = 0, range_max: typing.SupportsFloat | typing.SupportsIndex = 10.0, normalize: bool = True, stack_size: typing.SupportsInt | typing.SupportsIndex = 0, num_threads: typing.SupportsInt | typing.SupportsIndex = 1) -> None:
        """
        Args:
            index_lidar: Index into each robot's lidar list.
            range_max: Upper clamp distance in metres; 0 uses each robot's own lidar range_max. Clamped to >= 0; a positive value must exceed the lidar's range_min.
            normalize: Scale readings to [0, 1]: the lidar's range_min maps to 0, the upper clamp distance to 1.
            stack_size: Number of stacked past readings per robot; 0 and 1 are equivalent (no stacking).
            num_threads: Worker threads used internally; 0 uses hardware_concurrency().
        """
    def index_lidar(self) -> int:
        """
        Index into each robot's lidar list this observation reads.
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def normalize(self) -> bool:
        """
        Whether readings are scaled to [0, 1].
        """
    def num_threads(self) -> int:
        """
        Configured worker thread count for the internal compute path.
        """
    def range_max(self) -> float:
        """
        Configured upper clamp distance in metres (0 means "each robot's own lidar range_max").
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class LidarPoint(ObservationRobot):
    """
    Cartesian lidar hit points (x, y) for one lidar sensor, in metres.

    Feature shape per stacked state is (2, num_lasers) (x row, then y row).
    Not normalized. Readings are clamped to the lidar's [range_min, range_max]
    first, so a ray without a return (+inf) gives the point at range_max and
    every point is finite. When `local` is true, points are given as distance times
    unit beam direction in the lidar's own frame (NOT the robot frame unless
    the lidar is mounted at the robot origin with zero rotation); otherwise
    points are transformed through the lidar's mounting pose and the robot's
    pose into world coordinates. All robots must have the same lidar
    configuration at index_lidar.

    Args:
        index_lidar: Index into each robot's lidar list.
        local: Report points in the lidar's own frame instead of world
            coordinates.
        stack_size: Number of stacked past readings per robot; 0 and 1 are
            equivalent (no stacking).
        num_threads: Worker threads used inside the direct compute path (not
            the ObservationSet range path); 0 uses hardware_concurrency().
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> LidarPoint:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> LidarPoint:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, index_lidar: typing.SupportsInt | typing.SupportsIndex = 0, local: bool = False, stack_size: typing.SupportsInt | typing.SupportsIndex = 0, num_threads: typing.SupportsInt | typing.SupportsIndex = 1) -> None:
        """
        Args:
            index_lidar: Index into each robot's lidar list.
            local: Report points in the lidar's own frame instead of world coordinates.
            stack_size: Number of stacked past readings per robot; 0 and 1 are equivalent (no stacking).
            num_threads: Worker threads used internally; 0 uses hardware_concurrency().
        """
    def index_lidar(self) -> int:
        """
        Index into each robot's lidar list this observation reads.
        """
    def local(self) -> bool:
        """
        Whether points are reported in the lidar's own frame instead of world coordinates.
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def num_threads(self) -> int:
        """
        Configured worker thread count for the internal compute path.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
class SubgoalDirection(ObservationRobot):
    """
    Chained directions, in the robot's local frame, towards subgoals along its path.

    Subgoals are spaced evenly along the robot's planned path up to
    lookahead_distance ahead. Feature size is 2 * num_subgoals (no stacking:
    always evaluated at the current state). Subgoal s sits at arc-length
    distance ((s + 1) / num_subgoals) * lookahead_distance from the robot
    along its path, clamping to the goal once the path is exhausted; direction
    s is measured from subgoal (s - 1) to subgoal s (from the robot's pose for
    s == 0), so the vectors trace a polyline rather than all pointing from the
    robot. If normalize is set, each direction is unit length (zero if
    degenerate); otherwise it carries the raw metre offset between the two
    points. A subgoal that could not be found (only possible for an empty
    path) contributes a zero vector.

    Args:
        lookahead_distance: Arc-length distance (metres) covered by the last
            subgoal. Clamped to >= 0.1.
        num_subgoals: Number of evenly spaced subgoals. Clamped to >= 1.
        normalize: Report unit-length directions instead of raw metre offsets.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for each of `robots` and return it as one numpy array (see shape() for its layout).
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the observation for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> SubgoalDirection:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> SubgoalDirection:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, lookahead_distance: typing.SupportsFloat | typing.SupportsIndex = 2.5, num_subgoals: typing.SupportsInt | typing.SupportsIndex = 1, normalize: bool = True) -> None:
        """
        Args:
            lookahead_distance: Arc-length distance (metres) covered by the last subgoal. Clamped to >= 0.1.
            num_subgoals: Number of evenly spaced subgoals. Clamped to >= 1.
            normalize: Report unit-length directions instead of raw metre offsets.
        """
    def lookahead_distance(self) -> float:
        """
        The configured lookahead distance, in metres.
        """
    def name(self) -> str:
        """
        The observation's name, as given at construction.
        """
    def normalize(self) -> bool:
        """
        Whether directions are reported as unit vectors.
        """
    def num_subgoals(self) -> int:
        """
        The configured number of subgoals.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for observing `robots`.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for observing a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for observing the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for observing the robots of a single simulation.
        """
    def stack_size(self) -> int:
        """
        The configured stack size (may be 0, meaning "no stacking").
        """
