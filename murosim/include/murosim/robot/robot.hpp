// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ROBOT_ROBOT_HPP_
#define MUROSIM_ROBOT_ROBOT_HPP_

#include <cstddef>
#include <string>
#include <Eigen/Geometry>

#include "state_buffer.hpp"
#include "murosim/geometry/shape.hpp"
#include "murosim/actuator/actuator.hpp"
#include "murosim/sensor/lidar.hpp"
#include "murosim/navigation/node.hpp"
#include "murosim/visualization/color.hpp"
#include "murosim/planning/planner.hpp"

namespace murosim::robot
{

/// A single simulated robot: its actuator (kinematics/velocity model), body
/// and lidar shapes, mounted lidar sensors, pose, goal, and a ring buffer of
/// recorded states used for observations. Simulation drives the robot each
/// step via move()/makeState(); the many setters below are the interface
/// used to place robots, command actuators and configure visualization.
class Robot
{
public:
    /// Collision status against the environment, updated by Simulation each step.
    enum class STATUS_COLLISION{
        NONE,           ///< No collision detected.
        WORLD,          ///< Colliding with static world geometry.
        DYNAMIC_ENTITY, ///< Colliding with a non-robot dynamic entity.
        ROBOT           ///< Colliding with another robot.
    };
    /// Goal-seeking status, updated by Simulation::checkGoalStatus() each step.
    enum class STATUS_STATE{
        IDLE,           ///< Initial state, not yet evaluated against a goal.
        SEARCH_GOAL,    ///< Outside the goal radius, still travelling toward it.
        ON_GOAL,        ///< Inside the goal radius but time_on_goal_ threshold not yet reached.
        REACHED_GOAL,   ///< Goal considered reached (time-on-goal threshold met, or instant finish).
        UNUSED          ///< Robot excluded from this simulation run (see initialisation).
    };

    /// Constructs a robot from a name, actuator (cloned), robot/lidar collision shapes
    /// and the lidar sensors mounted on it. Deep-copies shape_robot/shape_lidar/lidars.
    Robot(const std::string name, const std::unique_ptr<murosim::actuator::Actuator> &actuator, const murosim::geometry::Shape &shape_robot, const murosim::geometry::Shape &shape_lidar, const std::vector<murosim::sensor::Lidar> &lidars);
    /// Deep copy, including cloning the actuator.
    Robot(const Robot &other);
    // Unified copy-and-swap assignment: handles both copy and move assignment
    // (rvalue arguments construct the by-value parameter via the move ctor).
    // A separate operator=(Robot&&) would be ambiguous with this overload.
    /// Copy- or move-assigns via copy-and-swap.
    Robot &operator=(Robot other);
    /// Defaulted move constructor.
    Robot(Robot&& other) noexcept;
    ~Robot() = default;
    /// Member-wise swap, used by the copy-and-swap assignment operator.
    friend void swap(Robot& first, Robot& second) noexcept;

    /// Lidar sensors mounted on the robot, in mounting order.
    const std::vector<murosim::sensor::Lidar>& getLidars() const;
    /// Number of lidar sensors mounted on the robot.
    size_t getLidarSize() const;
    /// @param index Lidar index, must be < getLidarSize().
    const sensor::Lidar& getLidarAt(const size_t index) const;
    /// @param index Lidar index, must be < getLidarSize().
    sensor::Lidar& getLidarAt(const size_t index);
    /// Lidar sensors mounted on the robot, in mounting order.
    std::vector<murosim::sensor::Lidar>& getLidars();
    /// Collision shape of the robot body, in the robot's local frame.
    const murosim::geometry::Shape& getShapeRobot() const;
    /// Collision shape used for lidar occlusion/ray tests, in the robot's local frame.
    const murosim::geometry::Shape& getShapeLidar() const;
    /// Recorded state at `index` steps in the past (0 = most recent).
    /// @param index Must be < getStateSize().
    const State &getStateAt(const size_t index) const;
    /** \brief getStateAt clamped to the oldest recorded state. Observations
     * routinely request stack indices past the recorded history and expect
     * the oldest state repeated; getStateAt would be out of bounds (assert
     * in debug, OOB read in release). Requires hasStates(). */
    const State &getStateAtOrOldest(const size_t index) const;
    /// Number of states currently recorded (grows up to the buffer size, then stays constant).
    size_t getStateSize() const;
    /// Sets the step counter used by getSimulatedStepCount(); does not affect state recording.
    void setSimulatedStepCount(const size_t simulated_step_count);
    /// Number of simulation steps this robot has been advanced through since the last reset().
    size_t getSimulatedStepCount() const;

    /// Advances the actuator by `time` seconds and integrates the resulting velocity into the
    /// pose; accumulates the travelled distance. Does not record a state (see makeState()).
    /// @param time Timestep in seconds, must be >= 0.
    void move(const float time);
    /// Resets the actuator, state buffer, step count, collision/goal status, travelled
    /// distance and lidars (Lidar::reset) to their initial values. Pose and goal are
    /// left untouched.
    void reset();

    /// Records the current pose/actuator/lidar/goal into the next slot of the state ring buffer.
    void makeState();

    /// Sets the planned path points (e.g. produced by planning()), used for visualization/observations.
    void setPathPoints(const std::vector<Eigen::Vector2f>& path_points);
    /// Places the robot at the node's start position/angle deterministically.
    void setPoseFromNode(const murosim::navigation::Node & node);
    /// Places the robot at a random position within the node's start circle and a random angle
    /// within the node's start angle bias.
    void setPoseFromNode(const murosim::navigation::Node & node, murosim::random::RandomGenerator &random_generator);
    /// Places the robot at a uniformly random position within `circle` (shrunk by the robot's
    /// collision radius so the whole body fits) and a uniformly random orientation.
    void setPoseFromCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator);
    /// Sets the full pose directly.
    void setPose(const Eigen::Affine2f &pose);
    /// Sets position and rotation from separate components.
    void setPose(const Eigen::Vector2f &position, const Eigen::Rotation2Df &rotation);
    /// Sets position (metres) and rotation (radians).
    void setPose(const float x, const float y, const float rotation);
    /// Sets the position, keeping the current rotation.
    void setPosition(const Eigen::Vector2f &position);
    /// Sets the position (metres), keeping the current rotation.
    void setPosition(const float x, const float y);
    /// Sets the rotation, keeping the current position.
    void setRotation(const Eigen::Rotation2Df &rotation);
    /// Sets the rotation (radians), keeping the current position.
    void setRotation(const float rotation);
    /// Sets the commanded body-frame velocity (x, y, angular); only used when the actuator's
    /// control mode is VELOCITY.
    void setActuatorCommandVelocity(const float x, const float y, const float angular);
    /// @copydoc setActuatorCommandVelocity(const float,const float,const float)
    void setActuatorCommandVelocity(const Eigen::Vector3f &cmd_vel);
    /// Sets the commanded body-frame acceleration (x, y, angular); only used when the actuator's
    /// control mode is ACCELERATION.
    void setActuatorCommandAcceleration(const float x, const float y, const float angular);
    /// @copydoc setActuatorCommandAcceleration(const float,const float,const float)
    void setActuatorCommandAcceleration(const Eigen::Vector3f &cmd_acc);
    /// Switches the actuator between commanding velocity and commanding acceleration.
    void setActuatorControlMode(const murosim::actuator::CONTROL_MODE mode);
    /// Sets the collision status; updated by Simulation's collision detection.
    void setStatusCollision(const STATUS_COLLISION status);
    /// Sets the goal-seeking status; updated by Simulation's goal check.
    void setStatusState(const STATUS_STATE status);
    /// Sets the goal node and overrides its goal radius.
    void setGoal(const murosim::navigation::Node &node, const float goal_radius);
    /// Sets the goal node, keeping its own goal radius.
    void setGoal(const murosim::navigation::Node &node);
    /// Sets the goal to a point (metres) with the given radius (metres).
    void setGoal(const float x, const float y, const float radius);
    /// Sets the goal to the given circle's centre and radius.
    void setGoal(const murosim::geometry::Circle &circle);
    /// Resizes the state ring buffer, discarding any recorded history.
    void setStateBufferSize(const size_t size);
    /// Sets the accumulated time-on-goal counter (seconds).
    void setTimeOnGoal(const float time);
    /// Sets the robot body's visualization color from 8-bit RGB channels.
    void setColorRobot(const uint8_t r, const uint8_t g, const uint8_t b);
    /// Sets the robot body's visualization color from normalized [0,1] RGB channels.
    void setColorRobot(const float r, const float g, const float b);
    /// Sets the robot body's visualization color from normalized [0,1] RGB components.
    void setColorRobot(const Eigen::Vector3f color);
    /// Sets the robot body's visualization color from 8-bit RGB components.
    void setColorRobot(const Eigen::Vector3i color);
    /// Sets the lidar visualization color from 8-bit RGB channels.
    void setColorLidar(const uint8_t r, const uint8_t g, const uint8_t b);
    /// Sets the lidar visualization color from normalized [0,1] RGB channels.
    void setColorLidar(const float r, const float g, const float b);
    /// Sets the lidar visualization color from normalized [0,1] RGB components.
    void setColorLidar(const Eigen::Vector3f color);
    /// Sets the lidar visualization color from 8-bit RGB components.
    void setColorLidar(const Eigen::Vector3i color);
    /// Sets the path visualization color from 8-bit RGB channels.
    void setColorPath(const uint8_t r, const uint8_t g, const uint8_t b);
    /// Sets the path visualization color from normalized [0,1] RGB channels.
    void setColorPath(const float r, const float g, const float b);
    /// Sets the path visualization color from normalized [0,1] RGB components.
    void setColorPath(const Eigen::Vector3f color);
    /// Sets the path visualization color from 8-bit RGB components.
    void setColorPath(const Eigen::Vector3i color);

    /// Planned path points (see planning()), in world coordinates (metres).
    const std::vector<Eigen::Vector2f>& getPathPoints() const;
    /// Number of planned path points.
    size_t getPathPointSize() const;
    /// @param index Must be < getPathPointSize().
    Eigen::Vector2f getPathPointAt(const size_t index) const;
    /// Robot name, as given at construction.
    const std::string &getName() const;
    /// Current pose (position in metres, rotation in radians).
    const Eigen::Affine2f &getPose() const;
    /// Current position (metres).
    Eigen::Vector2f getPosition() const;
    /// Current rotation as a 2D rotation matrix.
    Eigen::Rotation2Df getRotationMatrix() const;
    /// Current rotation angle (radians).
    float getRotationAngle() const;
    /// Commanded body-frame velocity (x, y, angular).
    Eigen::Vector3f getActuatorCommandVelocity() const;
    /// Commanded body-frame acceleration (x, y, angular).
    Eigen::Vector3f getActuatorCommandAcceleration() const;
    /// Actuator's current realized body-frame velocity (x, y, angular).
    Eigen::Vector3f getActuatorVelocity() const;
    /// Actuator's maximum body-frame velocity (x, y, angular).
    Eigen::Vector3f getActuatorVelocityMax() const;
    /// Actuator's minimum body-frame velocity (x, y, angular).
    Eigen::Vector3f getActuatorVelocityMin() const;
    /// Actuator's maximum body-frame acceleration (x, y, angular).
    Eigen::Vector3f getActuatorAccelerationMax() const;
    /// Whether the actuator is currently commanded by velocity or by acceleration.
    murosim::actuator::CONTROL_MODE getActuatorControlMode() const;
    /// Kinematic type of the mounted actuator.
    murosim::actuator::Actuator::TYPE getActuatorType() const;
    /// Radius (metres) of the smallest circle enclosing the robot body shape; cached at construction.
    float getCollisionRadius() const;
    /// Current collision status.
    STATUS_COLLISION getStatusCollision() const;
    /// Current goal-seeking status.
    STATUS_STATE getStatusState() const;
    /// Human-readable name of the current collision status.
    std::string getStatusCollisionAsString() const;
    /// Human-readable name of the given collision status.
    static std::string getStatusCollisionAsString(const STATUS_COLLISION collision);
    /// Human-readable name of the current goal-seeking status.
    std::string getStatusStateAsString() const;
    /// Human-readable name of the given goal-seeking status.
    static std::string getStatusStateAsString(const STATUS_STATE state);
    /// Current goal node.
    const murosim::navigation::Node &getGoal() const;
    /// Group id of the current goal node.
    size_t getGroup() const;
    /// Whether the robot has any mounted lidar sensors.
    bool hasLidar() const;
    /// Whether at least one state has been recorded via makeState().
    bool hasStates() const;
    /// Whether any path points have been set.
    bool hasPathPoints() const;
    /// Accumulated time (seconds) spent within the goal radius (see SimulationConfig::getPersistentGoalTimer()).
    float getTimeOnGoal() const;
    /// Total distance (metres) travelled by the robot since the last reset().
    float getTraveledDistance() const;
    /// Robot body's visualization color.
    murosim::visualization::Color getColorRobot() const;
    /// Lidar's visualization color.
    murosim::visualization::Color getColorLidar() const;
    /// Path's visualization color.
    murosim::visualization::Color getColorPath() const;

    /// Plans a path from the current position to the current goal on `map` and stores the
    /// result as this robot's path points.
    void planning(const std::unique_ptr<murosim::planning::Planner>& planner, const murosim::grid::Map& map);

    /// Whether the robot has been marked finished (excluded from further simulation updates).
    bool isFinished() const;
    /// Marks the robot as finished.
    void finished();
    /// Prints position, goal and status to `os` for debugging/logging.
    friend std::ostream& operator<<(std::ostream &os, const Robot& robot);

private:
    /** \brief Random position inside the circle shrunk by the robot's
     * collision radius, so the whole body fits; shared by the random
     * setPoseFrom* overloads. */
    Eigen::Vector2f randomPositionInCircle(const murosim::geometry::Circle &circle, murosim::random::RandomGenerator &random_generator) const;

    std::string name_;
    Eigen::Affine2f pose_;
    std::unique_ptr<murosim::actuator::Actuator> actuator_;

    murosim::geometry::Shape shape_robot_;
    murosim::geometry::Shape shape_lidar_;

    std::vector<murosim::sensor::Lidar> lidars_;
    murosim::navigation::Node goal_;
    murosim::robot::StateBuffer state_buffer_;
    std::vector<Eigen::Vector2f> path_points_;

    murosim::visualization::Color color_robot_;
    murosim::visualization::Color color_lidar_;
    murosim::visualization::Color color_path_;

    size_t simulated_step_count_;
    float time_on_goal_;
    float traveled_distance_;
    // cached: shape_robot_ is fixed after construction and
    // Shape::getCollisionRadius() recomputes on every call
    float collision_radius_;
    bool is_finished_;
    STATUS_COLLISION status_collision_;
    STATUS_STATE status_state_;
};



} // namespace murosim::robot

#endif // MUROSIM_ROBOT_ROBOT_HPP_
