// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "binding.hpp"

#include "murosimrl/observation/observation_robot.hpp"

// Robot observations
#include "murosimrl/observation/robot/pose.hpp"
#include "murosimrl/observation/robot/velocity.hpp"
#include "murosimrl/observation/robot/goal.hpp"
#include "murosimrl/observation/robot/distance_to_goal.hpp"
#include "murosimrl/observation/robot/orientation_to_goal.hpp"
#include "murosimrl/observation/robot/command_velocity.hpp"
#include "murosimrl/observation/robot/delta_pose.hpp"
#include "murosimrl/observation/robot/lidar.hpp"
#include "murosimrl/observation/robot/lidar_point.hpp"
#include "murosimrl/observation/robot/subgoal_direction.hpp"

namespace murosimrlpy {

using observation_binding::bindRobotObservation;

void initObservationRobot(py::module_ &m)
{
    // Abstract base — registered so concrete observations can be passed
    // around (and collected in ObservationSet) as ObservationRobot handles.
    py::class_<murosimrl::observation::ObservationRobot,
               std::shared_ptr<murosimrl::observation::ObservationRobot>>(m, "ObservationRobot",
        "Abstract base class for observations computed per robot (e.g. "
        "pose, velocity, lidar). Concrete subclasses can be called with a "
        "single robot, a list of robots, a single simulation, or a list of "
        "simulations (flattened to their combined robot list), and can be "
        "collected in an ObservationSet.");

    // Pose observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::Pose, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::Pose>>(m, "Pose", R"doc(Robot pose in world coordinates.

Position (metres) plus orientation, either as an angle in radians or as a
unit vector (cos, sin). Feature size is 3 per stacked state (x, y, angle),
or 4 (x, y, cos, sin) when orientation_as_unit_vector is set. Values are
not normalized.

Args:
    orientation_as_unit_vector: Encode orientation as (cos, sin) instead
        of a single angle.
    stack_size: Number of stacked past poses per robot; 0 and 1 are
        equivalent (no stacking).
)doc");
        pyClass.def(py::init<bool, size_t>(),
            "Args:\n"
            "    orientation_as_unit_vector: Encode orientation as (cos, sin) "
            "instead of a single angle.\n"
            "    stack_size: Number of stacked past poses per robot; 0 and 1 "
            "are equivalent (no stacking).",
            py::arg("orientation_as_unit_vector") = false,
            py::arg("stack_size") = 0);
        pyClass.def("orientation_as_unit_vector", &murosimrl::observation::robot::Pose::orientationAsUnitVector,
            "Whether orientation is encoded as a (cos, sin) unit vector instead of an angle.");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // Velocity observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::Velocity, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::Velocity>>(m, "Velocity", R"doc(The robot's actual (simulated) velocity.

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
)doc");
        pyClass.def(py::init<bool, size_t>(),
            "Args:\n"
            "    normalize: Scale each value to [-1, 1] against the robot's "
            "own actuator velocity limits.\n"
            "    stack_size: Number of stacked past values per robot; 0 and "
            "1 are equivalent (no stacking).",
            py::arg("normalize") = false,
            py::arg("stack_size") = 0);
        pyClass.def("normalize", &murosimrl::observation::robot::Velocity::normalize,
            "Whether values are scaled to [-1, 1] against the robot's own velocity limits.");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // Goal observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::Goal, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::Goal>>(m, "Goal", R"doc(Robot's goal in world coordinates.

Center (x, y) in metres and radius in metres, per stacked state. Feature
size is 3 per stacked state. Not normalized.

Args:
    stack_size: Number of stacked past goals per robot; 0 and 1 are
        equivalent (no stacking).
)doc");
        pyClass.def(py::init<size_t>(),
            "Args:\n"
            "    stack_size: Number of stacked past goals per robot; 0 and "
            "1 are equivalent (no stacking).",
            py::arg("stack_size") = 0);
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // DistanceToGoal observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::DistanceToGoal, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::DistanceToGoal>>(m, "DistanceToGoal", R"doc(Euclidean distance from the robot's pose to its goal, in metres.

Feature size is 1 per stacked state. If normalize_distance is positive,
the value is divided by it (so a distance of normalize_distance maps to
1.0); otherwise the raw metre distance is reported.

Args:
    normalize_distance: Distance (metres) that maps to 1.0; 0 or negative
        disables normalization. Clamped to >= 0.
    stack_size: Number of stacked past distances per robot; 0 and 1 are
        equivalent (no stacking).
)doc");
        pyClass.def(py::init<float, size_t>(),
            "Args:\n"
            "    normalize_distance: Distance (metres) that maps to 1.0; 0 "
            "or negative disables normalization. Clamped to >= 0.\n"
            "    stack_size: Number of stacked past distances per robot; 0 "
            "and 1 are equivalent (no stacking).",
            py::arg("normalize_distance") = 0.0f,
            py::arg("stack_size") = 0);
        pyClass.def("normalize_distance", &murosimrl::observation::robot::DistanceToGoal::normalizeDistance,
            "The configured normalization distance, in metres (0 means \"no normalization\").");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // OrientationToGoal observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::OrientationToGoal, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::OrientationToGoal>>(m, "OrientationToGoal", R"doc(Direction from the robot towards its goal, in the robot's local frame.

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
)doc");
        pyClass.def(py::init<float, size_t>(),
            "Args:\n"
            "    threshold_radius: Distance (metres) inside which the "
            "direction vector is scaled down towards zero. Clamped to >= 0.\n"
            "    stack_size: Number of stacked past values per robot; 0 and "
            "1 are equivalent (no stacking).",
            py::arg("threshold_radius") = 0.0f,
            py::arg("stack_size") = 0);
        pyClass.def("threshold_radius", &murosimrl::observation::robot::OrientationToGoal::thresholdRadius,
            "The configured threshold radius, in metres.");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // CommandVelocity observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::CommandVelocity, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::CommandVelocity>>(m, "CommandVelocity", R"doc(The velocity command last sent to the robot's actuator.

Same feature shape, units and normalization as Velocity, but reads the
commanded velocity instead of the actual (simulated) one.

Args:
    normalize: Scale each value to [-1, 1] against the robot's own
        actuator velocity limits.
    stack_size: Number of stacked past values per robot; 0 and 1 are
        equivalent (no stacking).
)doc");
        pyClass.def(py::init<bool, size_t>(),
            "Args:\n"
            "    normalize: Scale each value to [-1, 1] against the robot's "
            "own actuator velocity limits.\n"
            "    stack_size: Number of stacked past values per robot; 0 and "
            "1 are equivalent (no stacking).",
            py::arg("normalize") = false,
            py::arg("stack_size") = 0);
        pyClass.def("normalize", &murosimrl::observation::robot::CommandVelocity::normalize,
            "Whether values are scaled to [-1, 1] against the robot's own velocity limits.");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // DeltaPose observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::DeltaPose, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::DeltaPose>>(m, "DeltaPose", R"doc(Robot motion between consecutive recorded states.

Expressed in the robot's own frame at the earlier of the two states:
(dx, dy) translation in metres and rotation angle in radians. Feature size
is 3 per stacked state. Not normalized. At the oldest recorded state, both
reads clamp to it and the delta is zero.

Args:
    stack_size: Number of stacked deltas per robot; 0 and 1 are equivalent
        (no stacking).
)doc");
        pyClass.def(py::init<size_t>(),
            "Args:\n"
            "    stack_size: Number of stacked deltas per robot; 0 and 1 "
            "are equivalent (no stacking).",
            py::arg("stack_size") = 0);
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // Lidar observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::Lidar, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::Lidar>>(m, "Lidar", R"doc(Lidar range readings for one lidar sensor, in metres.

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
)doc");
        pyClass.def(py::init<size_t, float, bool, size_t, size_t>(),
            "Args:\n"
            "    index_lidar: Index into each robot's lidar list.\n"
            "    range_max: Upper clamp distance in metres; 0 uses each "
            "robot's own lidar range_max. Clamped to >= 0; a positive value "
            "must exceed the lidar's range_min.\n"
            "    normalize: Scale readings to [0, 1]: the lidar's range_min "
            "maps to 0, the upper clamp distance to 1.\n"
            "    stack_size: Number of stacked past readings per robot; 0 "
            "and 1 are equivalent (no stacking).\n"
            "    num_threads: Worker threads used internally; 0 uses "
            "hardware_concurrency().",
            py::arg("index_lidar") = 0,
            py::arg("range_max") = 10.0f,
            py::arg("normalize") = true,
            py::arg("stack_size") = 0,
            py::arg("num_threads") = 1);
        pyClass.def("index_lidar", &murosimrl::observation::robot::Lidar::indexLidar,
            "Index into each robot's lidar list this observation reads.");
        pyClass.def("range_max", &murosimrl::observation::robot::Lidar::rangeMax,
            "Configured upper clamp distance in metres (0 means \"each robot's own lidar range_max\").");
        pyClass.def("normalize", &murosimrl::observation::robot::Lidar::normalize,
            "Whether readings are scaled to [0, 1].");
        pyClass.def("num_threads", &murosimrl::observation::robot::Lidar::numThreads,
            "Configured worker thread count for the internal compute path.");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // LidarPoint observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::LidarPoint, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::LidarPoint>>(m, "LidarPoint", R"doc(Cartesian lidar hit points (x, y) for one lidar sensor, in metres.

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
)doc");
        pyClass.def(py::init<size_t, bool, size_t, size_t>(),
            "Args:\n"
            "    index_lidar: Index into each robot's lidar list.\n"
            "    local: Report points in the lidar's own frame instead of "
            "world coordinates.\n"
            "    stack_size: Number of stacked past readings per robot; 0 "
            "and 1 are equivalent (no stacking).\n"
            "    num_threads: Worker threads used internally; 0 uses "
            "hardware_concurrency().",
            py::arg("index_lidar") = 0,
            py::arg("local") = false,
            py::arg("stack_size") = 0,
            py::arg("num_threads") = 1);
        pyClass.def("index_lidar", &murosimrl::observation::robot::LidarPoint::indexLidar,
            "Index into each robot's lidar list this observation reads.");
        pyClass.def("local", &murosimrl::observation::robot::LidarPoint::local,
            "Whether points are reported in the lidar's own frame instead of world coordinates.");
        pyClass.def("num_threads", &murosimrl::observation::robot::LidarPoint::numThreads,
            "Configured worker thread count for the internal compute path.");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }

    // SubgoalDirection observation
    {
        auto pyClass = py::class_<murosimrl::observation::robot::SubgoalDirection, murosimrl::observation::ObservationRobot, std::shared_ptr<murosimrl::observation::robot::SubgoalDirection>>(m, "SubgoalDirection", R"doc(Chained directions, in the robot's local frame, towards subgoals along its path.

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
)doc");
        pyClass.def(py::init<float, size_t, bool>(),
            "Args:\n"
            "    lookahead_distance: Arc-length distance (metres) covered "
            "by the last subgoal. Clamped to >= 0.1.\n"
            "    num_subgoals: Number of evenly spaced subgoals. Clamped "
            "to >= 1.\n"
            "    normalize: Report unit-length directions instead of raw "
            "metre offsets.",
            py::arg("lookahead_distance") = 2.5f,
            py::arg("num_subgoals") = 1,
            py::arg("normalize") = true);
        pyClass.def("lookahead_distance", &murosimrl::observation::robot::SubgoalDirection::lookaheadDistance,
            "The configured lookahead distance, in metres.");
        pyClass.def("num_subgoals", &murosimrl::observation::robot::SubgoalDirection::numSubgoals,
            "The configured number of subgoals.");
        pyClass.def("normalize", &murosimrl::observation::robot::SubgoalDirection::normalize,
            "Whether directions are reported as unit vectors.");
        bindRobotObservation(pyClass);
        addCopySupport(pyClass);
    }
}

} // namespace murosimrlpy
