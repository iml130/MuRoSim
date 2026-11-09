# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Robot-level rewards.
"""
from __future__ import annotations
import murosimpy.containers
import murosimpy.core
import numpy
import numpy.typing
import typing
__all__: list[str] = ['ActionRate', 'DeltaDistanceAlongPath', 'DeltaDistanceToGoal', 'Done', 'ForwardDriving', 'MinimumLidarDistance', 'OnGoal', 'OrientationToGoal', 'RewardRobot']
class RewardRobot:
    """
    Abstract base class for rewards computed per robot. Concrete subclasses can be called with a single robot, a list of robots, a single simulation, or a list of simulations (flattened to their combined robot list), and can be collected in a RewardSet.
    """
class OnGoal(RewardRobot):
    """
    Rewards being close to the goal.

    Scales `positive` by how far inside a radius the robot's current distance
    to goal is, normalized to [0, 1] (1 exactly on the goal, 0 at or beyond
    the radius). If threshold_radius is positive, that radius is used
    directly; if it is <= 0, the robot's own goal radius is used instead (the
    same radius the simulation uses for its goal-reached check, so reward and
    termination agree), and the reward is zero when that radius is not
    positive.

    Args:
        threshold_radius: Radius (metres) the reward ramps up over; <= 0 uses
            each robot's own goal radius instead.
        positive: Maximum reward, earned exactly on the goal.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> OnGoal:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> OnGoal:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, threshold_radius: typing.SupportsFloat | typing.SupportsIndex, positive: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Args:
            threshold_radius: Radius (metres) the reward ramps up over; <= 0 uses each robot's own goal radius instead.
            positive: Maximum reward, earned exactly on the goal.
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def positive(self) -> float:
        """
        The configured maximum reward.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
    def threshold_radius(self) -> float:
        """
        The configured threshold radius (<= 0 means "use the robot's own goal radius").
        """
class DeltaDistanceAlongPath(RewardRobot):
    """
    Rewards progress towards a lookahead point on the robot's planned path.

    Compares, between the two most recent states, how the straight-line
    distance from the robot to the point lookahead_distance ahead on the path
    changed: a decrease (progress) is scaled by `positive`, an increase
    (regression) by `negative` (expected to be <= 0). Zero if the robot has
    fewer than two recorded states, has no path, or the lookahead point could
    not be found for either state.

    Args:
        positive: Reward scale applied to a decrease in lookahead distance
            (progress).
        negative: Reward scale applied to an increase in lookahead distance
            (regression); expected to be <= 0.
        lookahead_distance: Arc-length distance (metres) ahead on the path to
            track. Clamped to >= 0.1.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> DeltaDistanceAlongPath:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> DeltaDistanceAlongPath:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, positive: typing.SupportsFloat | typing.SupportsIndex, negative: typing.SupportsFloat | typing.SupportsIndex, lookahead_distance: typing.SupportsFloat | typing.SupportsIndex = 0.5) -> None:
        """
        Args:
            positive: Reward scale applied to a decrease in lookahead distance (progress).
            negative: Reward scale applied to an increase in lookahead distance (regression); expected to be <= 0.
            lookahead_distance: Arc-length distance (metres) ahead on the path to track. Clamped to >= 0.1.
        """
    def lookahead_distance(self) -> float:
        """
        The configured lookahead distance, in metres.
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def negative(self) -> float:
        """
        The configured regression penalty scale.
        """
    def positive(self) -> float:
        """
        The configured progress reward scale.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
class DeltaDistanceToGoal(RewardRobot):
    """
    Rewards progress towards the goal.

    Compares, between the two most recent states, the Euclidean distance from
    the robot's pose to its goal. A decrease (progress) is scaled by
    `positive`, an increase (regression) by `negative` (expected to be <= 0).
    Zero if the robot has fewer than two recorded states.

    Args:
        positive: Reward scale applied to a decrease in distance to goal
            (progress).
        negative: Reward scale applied to an increase in distance to goal
            (regression); expected to be <= 0.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> DeltaDistanceToGoal:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> DeltaDistanceToGoal:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, positive: typing.SupportsFloat | typing.SupportsIndex, negative: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Args:
            positive: Reward scale applied to a decrease in distance to goal (progress).
            negative: Reward scale applied to an increase in distance to goal (regression); expected to be <= 0.
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def negative(self) -> float:
        """
        The configured regression penalty scale.
        """
    def positive(self) -> float:
        """
        The configured progress reward scale.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
class ActionRate(RewardRobot):
    """
    Penalizes large, sudden changes in commanded velocity.

    For each axis (x, y, angular), the absolute difference between the latest
    commanded and actual actuator velocity is compared against that axis's
    threshold, and any excess beyond the threshold is multiplied by `negative`
    (expected to be <= 0) and summed across axes. Zero if the robot has no
    recorded state yet, or if all per-axis differences stay within their
    thresholds.

    Args:
        threshold_x: Tolerated absolute difference on the x axis before the
            penalty kicks in.
        threshold_y: Tolerated absolute difference on the y axis before the
            penalty kicks in.
        threshold_angular: Tolerated absolute difference on the angular axis
            before the penalty kicks in.
        negative: Penalty scale applied to the excess beyond each threshold;
            expected to be <= 0.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> ActionRate:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> ActionRate:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, threshold_x: typing.SupportsFloat | typing.SupportsIndex, threshold_y: typing.SupportsFloat | typing.SupportsIndex, threshold_angular: typing.SupportsFloat | typing.SupportsIndex, negative: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Args:
            threshold_x: Tolerated absolute difference on the x axis before the penalty kicks in.
            threshold_y: Tolerated absolute difference on the y axis before the penalty kicks in.
            threshold_angular: Tolerated absolute difference on the angular axis before the penalty kicks in.
            negative: Penalty scale applied to the excess beyond each threshold; expected to be <= 0.
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def negative(self) -> float:
        """
        The configured penalty scale.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
    def threshold_angular(self) -> float:
        """
        The configured angular-axis threshold.
        """
    def threshold_x(self) -> float:
        """
        The configured x-axis threshold.
        """
    def threshold_y(self) -> float:
        """
        The configured y-axis threshold.
        """
class ForwardDriving(RewardRobot):
    """
    Rewards driving forward with low sideways velocity.

    Scales `positive` by the fraction of max forward (x) velocity commanded
    (clamped to >= 0, so reverse motion earns nothing) times the fraction of
    threshold_y_velocity NOT used sideways (clamped to >= 0, so exceeding the
    threshold earns nothing). Zero if threshold_y_velocity is not positive or
    the robot's max x velocity is 0 (disabling the reward, e.g. for
    non-forward-driving actuators).

    Args:
        positive: Maximum reward, earned at full forward velocity with zero
            sideways velocity.
        threshold_y_velocity: Sideways velocity magnitude at or above which
            the reward drops to zero; <= 0 disables the reward.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> ForwardDriving:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> ForwardDriving:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, positive: typing.SupportsFloat | typing.SupportsIndex, threshold_y_velocity: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Args:
            positive: Maximum reward, earned at full forward velocity with zero sideways velocity.
            threshold_y_velocity: Sideways velocity magnitude at or above which the reward drops to zero; <= 0 disables the reward.
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def positive(self) -> float:
        """
        The configured maximum reward.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
    def threshold_y_velocity(self) -> float:
        """
        The configured sideways-velocity threshold.
        """
class Done(RewardRobot):
    """
    One-shot terminal reward for a robot's episode outcome.

    Zero unless the robot has finished (Robot.is_finished()); a finished robot
    receives positive_reached_goal if it reached its goal (checked before
    collision, since another robot can cause a collision at goal arrival),
    otherwise negative_collision_world or negative_collision_robot depending
    on the collision status, or zero if finished for another reason.

    Args:
        positive_reached_goal: Reward given when the robot's episode ended by
            reaching its goal.
        negative_collision_world: Reward given when the episode ended by
            colliding with the world (expected to be <= 0).
        negative_collision_robot: Reward given when the episode ended by
            colliding with another robot (expected to be <= 0).
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> Done:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> Done:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, positive_reached_goal: typing.SupportsFloat | typing.SupportsIndex, negative_collision_world: typing.SupportsFloat | typing.SupportsIndex, negative_collision_robot: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Args:
            positive_reached_goal: Reward given when the robot's episode ended by reaching its goal.
            negative_collision_world: Reward given when the episode ended by colliding with the world (expected to be <= 0).
            negative_collision_robot: Reward given when the episode ended by colliding with another robot (expected to be <= 0).
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def negative_collision_robot(self) -> float:
        """
        The configured robot-collision penalty.
        """
    def negative_collision_world(self) -> float:
        """
        The configured world-collision penalty.
        """
    def positive_reached_goal(self) -> float:
        """
        The configured goal-reached reward.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
class MinimumLidarDistance(RewardRobot):
    """
    Penalizes getting close to obstacles.

    Takes each lidar's minimum reading, clamped to its [range_min, range_max]
    (so -inf, too close, reads range_min and +inf, no return, reads
    range_max), and if it is below threshold_distance, scales `negative`
    (expected to be <= 0) by how far below the threshold it is, normalized to
    [0, 1] (0 exactly at the threshold, 1 at the lidar's range_min); the
    closest lidar decides. A lidar whose range_min is at or above the
    threshold does not contribute. Zero (reward disabled) if
    threshold_distance is not positive, since that would otherwise divide by
    zero.

    Args:
        threshold_distance: Distance (metres) below which the penalty kicks
            in; <= 0 disables the reward.
        negative: Penalty scale applied at the lidar's range_min (or closer);
            expected to be <= 0.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> MinimumLidarDistance:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> MinimumLidarDistance:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, threshold_distance: typing.SupportsFloat | typing.SupportsIndex, negative: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Args:
            threshold_distance: Distance (metres) below which the penalty kicks in; <= 0 disables the reward.
            negative: Penalty scale applied at the lidar's range_min (or closer); expected to be <= 0.
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def negative(self) -> float:
        """
        The configured penalty scale.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
    def threshold_distance(self) -> float:
        """
        The configured threshold distance, in metres.
        """
class OrientationToGoal(RewardRobot):
    """
    Rewards facing towards the goal.

    Takes the forward-axis alignment (cosine of the angle between the robot's
    heading and the direction to its goal) and scales it by `positive` when
    facing towards the goal (component >= 0) or by `negative` (expected to be
    <= 0) when facing away. A robot exactly on its goal has a zero orientation
    vector and scores neutral (0).

    Args:
        positive: Reward scale applied when facing towards the goal.
        negative: Reward scale applied when facing away from the goal;
            expected to be <= 0.
    """
    @typing.overload
    def __call__(self, robots: murosimpy.containers.Robots) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for each of `robots` and return it as one numpy array (see shape() for its layout). No-op (empty array) if `robots` is empty.
        """
    @typing.overload
    def __call__(self, robot: murosimpy.core.Robot) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for a single robot and return it as a numpy array.
        """
    @typing.overload
    def __call__(self, simulations: murosimpy.containers.Simulations) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the combined robots of `simulations` and return it as one numpy array.
        """
    @typing.overload
    def __call__(self, simulation: murosimpy.core.Simulation) -> numpy.typing.NDArray[numpy.float32]:
        """
        Compute the reward for the robots of a single simulation and return it as a numpy array.
        """
    def __copy__(self) -> OrientationToGoal:
        """
        Return a copy of this object (equivalent to a deep copy, since it holds no shared mutable state).
        """
    def __deepcopy__(self, arg0: dict) -> OrientationToGoal:
        """
        Return a deep copy of this object. The memo dict is accepted for protocol compatibility but is unused, since there is no shared mutable state to alias.
        """
    def __init__(self, positive: typing.SupportsFloat | typing.SupportsIndex, negative: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Args:
            positive: Reward scale applied when facing towards the goal.
            negative: Reward scale applied when facing away from the goal; expected to be <= 0.
        """
    def name(self) -> str:
        """
        The reward's name, as given at construction.
        """
    def negative(self) -> float:
        """
        The configured "facing away" penalty scale.
        """
    def positive(self) -> float:
        """
        The configured "facing towards" reward scale.
        """
    @typing.overload
    def shape(self, robots: murosimpy.containers.Robots) -> list[int]:
        """
        Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.
        """
    @typing.overload
    def shape(self, robot: murosimpy.core.Robot) -> list[int]:
        """
        Output shape for evaluating a single robot.
        """
    @typing.overload
    def shape(self, simulations: murosimpy.containers.Simulations) -> list[int]:
        """
        Output shape for evaluating the combined robots of `simulations`.
        """
    @typing.overload
    def shape(self, simulation: murosimpy.core.Simulation) -> list[int]:
        """
        Output shape for evaluating the robots of a single simulation.
        """
