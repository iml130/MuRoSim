# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
"""murosimrlpy: observations, rewards, actions and done flags on live robots."""

import numpy as np
import pytest

import murosimrlpy
from conftest import LIDAR_RAYS, LIDAR_RANGE_MAX, VELOCITY_MAX

ROBOT_OBSERVATIONS = {
    "pose": lambda: murosimrlpy.observation.robot.Pose(),
    "pose_unit_vector": lambda: murosimrlpy.observation.robot.Pose(orientation_as_unit_vector=True),
    "velocity": lambda: murosimrlpy.observation.robot.Velocity(),
    "command_velocity": lambda: murosimrlpy.observation.robot.CommandVelocity(normalize=True),
    "delta_pose": lambda: murosimrlpy.observation.robot.DeltaPose(),
    "goal": lambda: murosimrlpy.observation.robot.Goal(),
    "distance_to_goal": lambda: murosimrlpy.observation.robot.DistanceToGoal(),
    "orientation_to_goal": lambda: murosimrlpy.observation.robot.OrientationToGoal(),
    "lidar": lambda: murosimrlpy.observation.robot.Lidar(0, LIDAR_RANGE_MAX),
    "lidar_point": lambda: murosimrlpy.observation.robot.LidarPoint(0, local=True),
    "stacked_pose": lambda: murosimrlpy.observation.robot.Pose(stack_size=3),
}

ROBOT_REWARDS = {
    "delta_distance_to_goal": lambda: murosimrlpy.reward.robot.DeltaDistanceToGoal(1.0, -1.0),
    "on_goal": lambda: murosimrlpy.reward.robot.OnGoal(0.2, 1.0),
    "minimum_lidar_distance": lambda: murosimrlpy.reward.robot.MinimumLidarDistance(0.5, -1.0),
    "forward_driving": lambda: murosimrlpy.reward.robot.ForwardDriving(0.1, 0.1),
    "action_rate": lambda: murosimrlpy.reward.robot.ActionRate(0.5, 0.5, 0.5, -1.0),
    "done": lambda: murosimrlpy.reward.robot.Done(1.0, -1.0, -1.0),
    "orientation_to_goal": lambda: murosimrlpy.reward.robot.OrientationToGoal(0.1, -0.1),
}


@pytest.fixture
def stepped(simulation):
    for robot in simulation.robots:
        robot.command_velocity = [VELOCITY_MAX / 2.0, 0.0, 0.0]
    for _ in range(3):
        simulation.step()
    return simulation


@pytest.mark.parametrize("make", ROBOT_OBSERVATIONS.values(), ids=ROBOT_OBSERVATIONS.keys())
def test_robot_observation_shape_matches_its_output(make, stepped):
    observation = make()
    robots = stepped.robots
    for target in (robots, robots[0], stepped):
        output = observation(target)
        assert output.dtype == np.float32
        assert list(output.shape) == observation.shape(target)
        assert np.all(np.isfinite(output))
    assert observation(robots).shape[0] == len(robots)
    assert isinstance(observation.name(), str) and observation.name()


def test_lidar_observation_is_normalized(stepped):
    output = murosimrlpy.observation.robot.Lidar(0, LIDAR_RANGE_MAX)(stepped.robots)
    assert output.shape == (2, LIDAR_RAYS)
    assert 0.0 < output.min() and output.max() <= 1.0


def test_stack_size_adds_a_dimension(stepped):
    stacked = murosimrlpy.observation.robot.Pose(stack_size=3)(stepped.robots)
    assert stacked.shape == (2, 3, 3)
    np.testing.assert_allclose(stacked[:, 0, :], murosimrlpy.observation.robot.Pose()(stepped.robots))


def test_observation_set_computes_every_member(stepped, thread_pool):
    observation_set = murosimrlpy.observation.ObservationSet()
    observation_set.add(murosimrlpy.observation.robot.Pose())
    observation_set.add(murosimrlpy.observation.robot.Lidar(0, LIDAR_RANGE_MAX))
    robots = stepped.robots
    outputs = observation_set.compute(thread_pool, robots)
    assert len(observation_set) == 2
    assert [list(output.shape) for output in outputs] == observation_set.shapes(robots)
    np.testing.assert_array_equal(outputs[0], murosimrlpy.observation.robot.Pose()(robots))


def test_simulation_map_observations(stepped):
    map_size = 128
    for make in (murosimrlpy.observation.simulation.OccupancyMap, murosimrlpy.observation.simulation.PathMap):
        observation = make(map_size)
        output = observation(stepped)
        assert output.dtype == np.float32
        assert list(output.shape) == observation.shape(stepped)
    with pytest.raises(RuntimeError):
        murosimrlpy.observation.simulation.OccupancyMap(16)(stepped)  # smaller than the grid map


@pytest.mark.parametrize("make", ROBOT_REWARDS.values(), ids=ROBOT_REWARDS.keys())
def test_robot_reward_is_one_value_per_robot(make, stepped):
    reward = make()
    output = reward(stepped.robots)
    assert output.dtype == np.float32
    assert output.shape == (2, 1)
    assert reward(stepped.robots[0]).shape == (1, 1)
    assert np.all(np.isfinite(output))


def test_reward_set_sums_its_members(stepped, thread_pool):
    reward_set = murosimrlpy.reward.RewardSet()
    reward_set.add(murosimrlpy.reward.robot.DeltaDistanceToGoal(1.0, -1.0))
    reward_set.add(murosimrlpy.reward.robot.OnGoal(0.2, 1.0))
    robots = stepped.robots
    total = reward_set.compute(thread_pool, robots)
    expected = (murosimrlpy.reward.robot.DeltaDistanceToGoal(1.0, -1.0)(robots)
                + murosimrlpy.reward.robot.OnGoal(0.2, 1.0)(robots))
    assert total.dtype == np.float32
    np.testing.assert_allclose(total, expected)
    assert total[0, 0] > 0.0  # robot 0 drove towards its goal


def test_action_writes_command_velocities(simulation):
    action = murosimrlpy.core.Action(normalize=True)
    robots = simulation.robots
    assert action.shape(robots) == [2, 3]
    assert action.size(robots[0]) == 3
    np.testing.assert_array_equal(action.get_neutral_point(robots[0]), [0.0, 0.0, 0.0])
    commands = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.5]], dtype=np.float32)
    action(robots, commands)
    np.testing.assert_allclose(robots[0].command_velocity, [VELOCITY_MAX, 0.0, 0.0])
    np.testing.assert_allclose(robots[1].command_velocity, [0.0, -VELOCITY_MAX, 0.5 * VELOCITY_MAX])
    with pytest.raises((RuntimeError, ValueError, TypeError)):
        action(robots, np.zeros((2, 2), dtype=np.float32))


def test_done_flags_are_booleans(stepped):
    done = murosimrlpy.core.Done()
    output = done(stepped.robots)
    assert output.dtype == np.bool_
    assert output.shape == (2, 1)
    assert not output.any()
    assert done(stepped).shape == tuple(done.shape(stepped))
