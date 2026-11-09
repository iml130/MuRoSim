// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosimrl/reward/reward_robot.hpp"
#include "murosimrl/util/conversion.hpp"

// Robot rewards
#include "murosimrl/reward/robot/on_goal.hpp"
#include "murosimrl/reward/robot/delta_distance_along_path.hpp"
#include "murosimrl/reward/robot/delta_distance_to_goal.hpp"
#include "murosimrl/reward/robot/action_rate.hpp"
#include "murosimrl/reward/robot/forward_driving.hpp"
#include "murosimrl/reward/robot/done.hpp"
#include "murosimrl/reward/robot/minimum_lidar_distance.hpp"
#include "murosimrl/reward/robot/orientation_to_goal.hpp"

#include "murosim/robot/robot.hpp"
#include "murosim/simulation/simulation.hpp"

namespace murosimrlpy {

namespace {

template<typename RewardType>
py::array_t<float> callReward(
    const RewardType &reward,
    const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots)
{
    std::vector<size_t> shape = reward.shape(robots);

    if (shape.empty())
    {
        return py::array_t<float>();
    }

    py::array_t<float> array(shape);
    float* data = array.mutable_data();

    {
        py::gil_scoped_release gil;
        reward(robots, data);
    }

    return array;
}

template<typename RewardType>
py::array_t<float> callRewardSingleRobot(
    const RewardType &reward,
    const std::shared_ptr<murosim::robot::Robot> &robot)
{
    return callReward(reward, std::vector<std::shared_ptr<murosim::robot::Robot>>{robot});
}

template<typename RewardType>
py::array_t<float> callRewardSimulations(
    const RewardType &reward,
    const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations)
{
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots =
        murosimrl::util::conversion::fromVectorSimulationToVectorRobot(simulations);
    return callReward(reward, robots);
}

template<typename RewardType>
py::array_t<float> callRewardSingleSimulation(
    const RewardType &reward,
    const std::shared_ptr<murosim::simulation::Simulation> &simulation)
{
    return callRewardSimulations(reward, std::vector<std::shared_ptr<murosim::simulation::Simulation>>{simulation});
}

// Shared binding surface for the robot-scope reward classes: __call__/shape
// over a robot, a robot list, a simulation, or a list of simulations
// (flattened to their combined robot list), plus name.
template<typename RewardType, typename... Extra>
void bindRewardRobot(py::class_<RewardType, Extra...> &pyClass)
{
    pyClass.def("__call__", [](const RewardType &self, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) {
        return callReward(self, robots);
    }, "Compute the reward for each of `robots` and return it as one numpy "
       "array (see shape() for its layout). No-op (empty array) if "
       "`robots` is empty.",
       py::arg("robots"));

    pyClass.def("__call__", [](const RewardType &self, const std::shared_ptr<murosim::robot::Robot> &robot) {
        return callRewardSingleRobot(self, robot);
    }, "Compute the reward for a single robot and return it as a numpy array.",
       py::arg("robot"));

    pyClass.def("__call__", [](const RewardType &self, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        return callRewardSimulations(self, simulations);
    }, "Compute the reward for the combined robots of `simulations` and "
       "return it as one numpy array.",
       py::arg("simulations"));

    pyClass.def("__call__", [](const RewardType &self, const std::shared_ptr<murosim::simulation::Simulation> &simulation) {
        return callRewardSingleSimulation(self, simulation);
    }, "Compute the reward for the robots of a single simulation and "
       "return it as a numpy array.",
       py::arg("simulation"));

    pyClass.def("shape", [](const RewardType &self, const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) {
        return self.shape(robots);
    }, "Output shape for evaluating `robots`: (robots.size(), 1) unless overridden.",
       py::arg("robots"));

    pyClass.def("shape", [](const RewardType &self, const std::shared_ptr<murosim::robot::Robot> &robot) {
        return self.shape(std::vector<std::shared_ptr<murosim::robot::Robot>>{robot});
    }, "Output shape for evaluating a single robot.",
       py::arg("robot"));

    pyClass.def("shape", [](const RewardType &self, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        return self.shape(simulations);
    }, "Output shape for evaluating the combined robots of `simulations`.",
       py::arg("simulations"));

    pyClass.def("shape", [](const RewardType &self, const std::shared_ptr<murosim::simulation::Simulation> &simulation) {
        return self.shape(simulation);
    }, "Output shape for evaluating the robots of a single simulation.",
       py::arg("simulation"));

    pyClass.def("name", &RewardType::name,
        "The reward's name, as given at construction.");
}

} // anonymous namespace

void initRewardRobot(py::module_ &m)
{
    // Abstract base — registered so concrete rewards can be passed around
    // (and collected in RewardSet) as RewardRobot handles.
    py::class_<murosimrl::reward::RewardRobot,
               std::shared_ptr<murosimrl::reward::RewardRobot>>(m, "RewardRobot",
        "Abstract base class for rewards computed per robot. Concrete "
        "subclasses can be called with a single robot, a list of robots, "
        "a single simulation, or a list of simulations (flattened to "
        "their combined robot list), and can be collected in a RewardSet.");

    // OnGoal reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::OnGoal, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::OnGoal>>(m, "OnGoal", R"doc(Rewards being close to the goal.

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
)doc");
        pyClass.def(py::init<float, float>(),
            "Args:\n"
            "    threshold_radius: Radius (metres) the reward ramps up "
            "over; <= 0 uses each robot's own goal radius instead.\n"
            "    positive: Maximum reward, earned exactly on the goal.",
            py::arg("threshold_radius"),
            py::arg("positive"));
        pyClass.def("threshold_radius", &murosimrl::reward::robot::OnGoal::thresholdRadius,
            "The configured threshold radius (<= 0 means \"use the robot's own goal radius\").");
        pyClass.def("positive", &murosimrl::reward::robot::OnGoal::positive,
            "The configured maximum reward.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }

    // DeltaDistanceAlongPath reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::DeltaDistanceAlongPath, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::DeltaDistanceAlongPath>>(m, "DeltaDistanceAlongPath", R"doc(Rewards progress towards a lookahead point on the robot's planned path.

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
)doc");
        pyClass.def(py::init<float, float, float>(),
            "Args:\n"
            "    positive: Reward scale applied to a decrease in lookahead "
            "distance (progress).\n"
            "    negative: Reward scale applied to an increase in "
            "lookahead distance (regression); expected to be <= 0.\n"
            "    lookahead_distance: Arc-length distance (metres) ahead on "
            "the path to track. Clamped to >= 0.1.",
            py::arg("positive"),
            py::arg("negative"),
            py::arg("lookahead_distance") = 0.5f);
        pyClass.def("positive", &murosimrl::reward::robot::DeltaDistanceAlongPath::positive,
            "The configured progress reward scale.");
        pyClass.def("negative", &murosimrl::reward::robot::DeltaDistanceAlongPath::negative,
            "The configured regression penalty scale.");
        pyClass.def("lookahead_distance", &murosimrl::reward::robot::DeltaDistanceAlongPath::lookaheadDistance,
            "The configured lookahead distance, in metres.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }

    // DeltaDistanceToGoal reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::DeltaDistanceToGoal, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::DeltaDistanceToGoal>>(m, "DeltaDistanceToGoal", R"doc(Rewards progress towards the goal.

Compares, between the two most recent states, the Euclidean distance from
the robot's pose to its goal. A decrease (progress) is scaled by
`positive`, an increase (regression) by `negative` (expected to be <= 0).
Zero if the robot has fewer than two recorded states.

Args:
    positive: Reward scale applied to a decrease in distance to goal
        (progress).
    negative: Reward scale applied to an increase in distance to goal
        (regression); expected to be <= 0.
)doc");
        pyClass.def(py::init<float, float>(),
            "Args:\n"
            "    positive: Reward scale applied to a decrease in distance "
            "to goal (progress).\n"
            "    negative: Reward scale applied to an increase in "
            "distance to goal (regression); expected to be <= 0.",
            py::arg("positive"),
            py::arg("negative"));
        pyClass.def("positive", &murosimrl::reward::robot::DeltaDistanceToGoal::positive,
            "The configured progress reward scale.");
        pyClass.def("negative", &murosimrl::reward::robot::DeltaDistanceToGoal::negative,
            "The configured regression penalty scale.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }

    // ActionRate reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::ActionRate, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::ActionRate>>(m, "ActionRate", R"doc(Penalizes large, sudden changes in commanded velocity.

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
)doc");
        pyClass.def(py::init<float, float, float, float>(),
            "Args:\n"
            "    threshold_x: Tolerated absolute difference on the x axis "
            "before the penalty kicks in.\n"
            "    threshold_y: Tolerated absolute difference on the y axis "
            "before the penalty kicks in.\n"
            "    threshold_angular: Tolerated absolute difference on the "
            "angular axis before the penalty kicks in.\n"
            "    negative: Penalty scale applied to the excess beyond each "
            "threshold; expected to be <= 0.",
            py::arg("threshold_x"),
            py::arg("threshold_y"),
            py::arg("threshold_angular"),
            py::arg("negative"));
        pyClass.def("threshold_x", &murosimrl::reward::robot::ActionRate::thresholdX,
            "The configured x-axis threshold.");
        pyClass.def("threshold_y", &murosimrl::reward::robot::ActionRate::thresholdY,
            "The configured y-axis threshold.");
        pyClass.def("threshold_angular", &murosimrl::reward::robot::ActionRate::thresholdAngular,
            "The configured angular-axis threshold.");
        pyClass.def("negative", &murosimrl::reward::robot::ActionRate::negative,
            "The configured penalty scale.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }

    // ForwardDriving reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::ForwardDriving, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::ForwardDriving>>(m, "ForwardDriving", R"doc(Rewards driving forward with low sideways velocity.

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
)doc");
        pyClass.def(py::init<float, float>(),
            "Args:\n"
            "    positive: Maximum reward, earned at full forward velocity "
            "with zero sideways velocity.\n"
            "    threshold_y_velocity: Sideways velocity magnitude at or "
            "above which the reward drops to zero; <= 0 disables the "
            "reward.",
            py::arg("positive"),
            py::arg("threshold_y_velocity"));
        pyClass.def("positive", &murosimrl::reward::robot::ForwardDriving::positive,
            "The configured maximum reward.");
        pyClass.def("threshold_y_velocity", &murosimrl::reward::robot::ForwardDriving::thresholdYVelocity,
            "The configured sideways-velocity threshold.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }

    // Done reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::Done, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::Done>>(m, "Done", R"doc(One-shot terminal reward for a robot's episode outcome.

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
)doc");
        pyClass.def(py::init<float, float, float>(),
            "Args:\n"
            "    positive_reached_goal: Reward given when the robot's "
            "episode ended by reaching its goal.\n"
            "    negative_collision_world: Reward given when the episode "
            "ended by colliding with the world (expected to be <= 0).\n"
            "    negative_collision_robot: Reward given when the episode "
            "ended by colliding with another robot (expected to be <= 0).",
            py::arg("positive_reached_goal"),
            py::arg("negative_collision_world"),
            py::arg("negative_collision_robot"));
        pyClass.def("positive_reached_goal", &murosimrl::reward::robot::Done::positiveReachedGoal,
            "The configured goal-reached reward.");
        pyClass.def("negative_collision_world", &murosimrl::reward::robot::Done::negativeCollisionWorld,
            "The configured world-collision penalty.");
        pyClass.def("negative_collision_robot", &murosimrl::reward::robot::Done::negativeCollisionRobot,
            "The configured robot-collision penalty.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }

    // MinimumLidarDistance reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::MinimumLidarDistance, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::MinimumLidarDistance>>(m, "MinimumLidarDistance", R"doc(Penalizes getting close to obstacles.

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
)doc");
        pyClass.def(py::init<float, float>(),
            "Args:\n"
            "    threshold_distance: Distance (metres) below which the "
            "penalty kicks in; <= 0 disables the reward.\n"
            "    negative: Penalty scale applied at the lidar's range_min "
            "(or closer); expected to be <= 0.",
            py::arg("threshold_distance"),
            py::arg("negative"));
        pyClass.def("threshold_distance", &murosimrl::reward::robot::MinimumLidarDistance::thresholdDistance,
            "The configured threshold distance, in metres.");
        pyClass.def("negative", &murosimrl::reward::robot::MinimumLidarDistance::negative,
            "The configured penalty scale.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }

    // OrientationToGoal reward
    {
        auto pyClass = py::class_<murosimrl::reward::robot::OrientationToGoal, murosimrl::reward::RewardRobot, std::shared_ptr<murosimrl::reward::robot::OrientationToGoal>>(m, "OrientationToGoal", R"doc(Rewards facing towards the goal.

Takes the forward-axis alignment (cosine of the angle between the robot's
heading and the direction to its goal) and scales it by `positive` when
facing towards the goal (component >= 0) or by `negative` (expected to be
<= 0) when facing away. A robot exactly on its goal has a zero orientation
vector and scores neutral (0).

Args:
    positive: Reward scale applied when facing towards the goal.
    negative: Reward scale applied when facing away from the goal;
        expected to be <= 0.
)doc");
        pyClass.def(py::init<float, float>(),
            "Args:\n"
            "    positive: Reward scale applied when facing towards the "
            "goal.\n"
            "    negative: Reward scale applied when facing away from the "
            "goal; expected to be <= 0.",
            py::arg("positive"),
            py::arg("negative"));
        pyClass.def("positive", &murosimrl::reward::robot::OrientationToGoal::positive,
            "The configured \"facing towards\" reward scale.");
        pyClass.def("negative", &murosimrl::reward::robot::OrientationToGoal::negative,
            "The configured \"facing away\" penalty scale.");
        bindRewardRobot(pyClass);
        addCopySupport(pyClass);
    }
}

} // namespace murosimrlpy
