// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SIMULATION_SIMULATION_HPP_
#define MUROSIM_SIMULATION_SIMULATION_HPP_

#include "murosim/robot/robot.hpp"
#include "murosim/grid/map.hpp"
#include "murosim/navigation/container.hpp"
#include "murosim/planning/planner.hpp"
#include "murosim/random/random_generator.hpp"
#include "murosim/io/data_container.hpp"
#include "murosim/simulation/simulation_config.hpp"




namespace murosim::simulation
{

/// Owns and advances one multi-robot scenario: the static world, the
/// navigation graph, the derived occupancy grid, the set of robots, and the
/// per-simulation RNG and planner. `simulate()` steps the whole scenario by
/// one `SimulationConfig::getTimeStep()`, handling actuation, collision
/// detection, lidar updates, goal bookkeeping and (re-)initialisation. Not
/// thread-safe: concurrent simulations must each own their own instance.
class Simulation
{
public:
    // Type alias for backward compatibility
    using INIT_MODE = SimulationConfig::INIT_MODE;

    // ========== Existing Constructors (unchanged for backward compatibility) ==========
    /// Build from a loaded/generated world description, using scalar
    /// convenience parameters instead of a full SimulationConfig.
    /// @param grid_scale metres per occupancy grid cell.
    /// @param buffer_scale extra clearance (metres) added around obstacles in the grid.
    /// @param seed RNG seed; defaults to a hardware-derived random value.
    Simulation(const murosim::io::DataContainer &data_container, const float grid_scale=0.1f, const float buffer_scale=0.2f, const INIT_MODE init_mode=INIT_MODE::MANUAL, const murosim::planning::Planner::TYPE planner_type=murosim::planning::Planner::TYPE::NONE, const uint32_t seed=std::random_device()());
    /// Build from explicit world/navigation objects and scalar parameters (see other overload).
    Simulation(const murosim::geometry::World &world, const murosim::navigation::Container &nodes, const std::string &name, const float grid_scale=0.1f, const float buffer_scale=0.2f, const INIT_MODE init_mode=INIT_MODE::MANUAL, const murosim::planning::Planner::TYPE planner_type=murosim::planning::Planner::TYPE::NONE, const uint32_t seed=std::random_device()());

    // ========== New Config-based Constructors ==========
    /// Build from a loaded/generated world description and a validated SimulationConfig.
    Simulation(const murosim::io::DataContainer &data_container, const SimulationConfig &config);
    /// Build from explicit world/navigation objects and a validated SimulationConfig.
    Simulation(const murosim::geometry::World &world, const murosim::navigation::Container &nodes, const std::string &name, const SimulationConfig &config);
    /// Deep-copies the robot list and clones the planner; the world, nodes and grid are value-copied.
    Simulation(const Simulation &other);
    Simulation& operator=(const Simulation &other);
    Simulation(Simulation&& other) noexcept;
    Simulation& operator=(Simulation&& other) noexcept;
    friend void swap(Simulation& first, Simulation& second) noexcept;

    [[nodiscard]] const murosim::geometry::World &getWorld() const;
    [[nodiscard]] const murosim::navigation::Container &getNodes() const;
    /// Occupancy grid derived from the world at construction/setConfig time.
    [[nodiscard]] const murosim::grid::Map &getGridMap() const;

    [[nodiscard]] std::vector<std::shared_ptr<murosim::robot::Robot>>& getRobots();
    [[nodiscard]] const std::vector<std::shared_ptr<murosim::robot::Robot>>& getRobots() const;
    [[nodiscard]] size_t getRobotSize() const;
    /// @param index must be < getRobotSize().
    [[nodiscard]] const std::shared_ptr<murosim::robot::Robot>& getRobotPtrAt(const size_t index) const;
    /// @param index must be < getRobotSize().
    [[nodiscard]] std::shared_ptr<murosim::robot::Robot>& getRobotPtrAt(const size_t index);
    /// @param index must be < getRobotSize().
    [[nodiscard]] const murosim::robot::Robot& getRobotAt(const size_t index) const;
    /// @param index must be < getRobotSize().
    [[nodiscard]] murosim::robot::Robot& getRobotAt(const size_t index);

    /// Adds the robot if it is not already present (by pointer identity).
    /// Invalidates readiness: the next simulate() call re-initialises positions.
    /// @return false if the robot was already registered.
    bool addRobot(const std::shared_ptr<murosim::robot::Robot> &robot);
    /// Clears all robots and marks the simulation not ready.
    void removeAllRobots();
    /// Advances the simulation by one time step (SimulationConfig::getTimeStep()
    /// seconds), sub-stepping actuators/collision/sensors at their configured
    /// intervals. If the simulation is not ready, performs reset() instead
    /// (which itself invokes one step).
    void simulate();
    /// Resets every robot (including its lidars), re-initialises robot
    /// positions per the configured INIT_MODE and runs one simulate() step to
    /// populate the initial lidar/state, then rewinds step counters to 0. In
    /// CONTINUOUS mode every lidar first takes a complete scan of the new
    /// pose, so no reading from before the reset survives into the episode.
    /// @throws std::runtime_error if position initialisation fails.
    void reset();

    [[nodiscard]] const SimulationConfig& getConfig() const { return config_; }
    /// Replaces the configuration, rebuilds the grid map and planner from it,
    /// re-seeds the RNG only if the seed actually changed, and marks the
    /// simulation not ready (robots are re-initialised on the next simulate()).
    void setConfig(const SimulationConfig &config);

    [[nodiscard]] const std::string& getName() const;

    /// True once every robot has finished (collided, reached its goal with no
    /// new goal to assign, or was parked as unused).
    [[nodiscard]] bool isFinished() const;
    /// True if robot positions have been successfully initialised for the current config.
    [[nodiscard]] bool isReady() const;

    /// Number of navigation nodes available for random placement.
    [[nodiscard]] size_t getPossibleRobotSize() const;
    /// Number of distinct navigation node groups.
    [[nodiscard]] size_t getGroupSize() const;
    [[nodiscard]] size_t getSimulatedStepCount() const;

    /// Runs the configured planner (if any) for every unfinished robot.
    void plan();

private:
    murosim::geometry::World world_;
    murosim::navigation::Container nodes_;
    /// Occupancy grid derived from world_/nodes_/config_; rebuilt in setConfig().
    murosim::grid::Map map_;
    std::string name_;
    std::vector<std::shared_ptr<murosim::robot::Robot>> robots_;

    murosim::random::RandomGenerator random_generator_;
    SimulationConfig config_;

    bool is_ready_;
    bool is_finished_;

    size_t simulated_step_count_;
    std::unique_ptr<murosim::planning::Planner> planner_;

    /// Places all robots' start/goal poses according to config_.getInitMode(); a no-op returning true for MANUAL.
    bool initRobotPositions();
    /// @return true if the pointer is already present in robots_.
    bool containsRobot(const std::shared_ptr<const murosim::robot::Robot> &robot);

    /// Advances every unfinished, non-colliding robot's actuator by `time` seconds.
    void moveRobots(const float time);
    /// Robot-vs-robot then robot-vs-world narrow-phase collision checks; updates each robot's STATUS_COLLISION.
    void detectCollisions();
    /// Performs one full synchronized lidar sweep for every robot.
    void updateLidar();
    /// Updates each robot's goal-proximity state (SEARCH_GOAL/ON_GOAL/REACHED_GOAL) and time-on-goal counter.
    void checkGoalStatus();
    /// Marks robots finished() if they collided, or reached their goal with no new goal to assign.
    void checkRobotsFinished();
    /// Records a new State snapshot for every unfinished robot.
    void updateRobotState();
    /// Stamps the current simulated_step_count_ onto every unfinished robot.
    void updateRobotStepCounts();
    /// If useNewGoal is enabled, assigns a fresh goal to every robot that just reached one.
    /// @throws std::runtime_error if a new goal cannot be found for a robot.
    void assignNewGoals();
    /// Recomputes is_finished_ as true iff every robot is finished.
    void checkSimulationFinished();
};


} // namespace murosim::simulation

#endif // MUROSIM_SIMULATION_SIMULATION_HPP_
