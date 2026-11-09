// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SIMULATION_SIMULATION_CONFIG_HPP_
#define MUROSIM_SIMULATION_SIMULATION_CONFIG_HPP_

#include "murosim/grid/map.hpp"
#include "murosim/planning/planner.hpp"
#include <random>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <string>

namespace murosim::simulation
{

/**
 * @brief Configuration parameters for Simulation
 *
 * This class holds all configurable parameters for creating and running
 * a simulation. All parameters are validated when set to ensure simulation
 * stability. It can be reused across multiple simulation instances,
 * making it ideal for factory patterns and batch simulation creation.
 */
class SimulationConfig
{
public:
    // ========== Enums (public) ==========
    /// How robot start/goal positions are (re-)chosen on reset()/assignNewGoals().
    enum class INIT_MODE {
        MANUAL,             ///< Manual placement
        FREE_ALL_RANDOM,    ///< Random free positions
        NODE_ALL_RANDOM,    ///< Random node positions
        NODE_GROUP_RANDOM   ///< Random positions within groups
    };

    /// When lidar sensors are updated relative to the simulation step loop.
    enum class SENSOR_UPDATE_MODE {
        SYNCHRONIZED,       ///< All sensors update synchronized at end of simulation step
        CONTINUOUS          ///< Sensors update continuously during simulation step
    };

    // ========== Constants ==========
    /// Minimum allowed value (seconds) for any of the time-interval setters.
    static constexpr float MINIMUM_TIME_INTERVAL = 1e-6f;
    // shared with the map, so a validated config never disagrees with the
    // Map actually built from it (Map clamps to the same bound)
    /// Minimum allowed grid scale (metres/cell), mirrors murosim::grid::Map::GRID_SCALE_MIN.
    static constexpr float MINIMUM_GRID_SCALE = murosim::grid::Map::GRID_SCALE_MIN;

    // ========== Constructor ==========
    /// Constructs with default parameter values (see the private member initializers below) and a random seed.
    SimulationConfig() = default;

    // ========== Time Parameter Getters ==========
    /// Simulation step duration in seconds.
    [[nodiscard]] float getTimeStep() const { return time_step_; }
    /// Sub-step interval (seconds) between collision checks.
    [[nodiscard]] float getTimeIntervalCollision() const { return time_interval_collision_; }
    /// Sub-step interval (seconds) between actuator updates.
    [[nodiscard]] float getTimeIntervalActuator() const { return time_interval_actuator_; }
    /// Sub-step interval (seconds) between sensor updates in CONTINUOUS mode.
    [[nodiscard]] float getTimeIntervalSensor() const { return time_interval_sensor_; }
    /// Seconds a robot must dwell on its goal before being considered REACHED_GOAL; -1.0 means instant finish.
    [[nodiscard]] float getTimeOnGoal() const { return time_on_goal_; }
    /// If true, the on-goal timer keeps counting after the robot leaves the goal instead of resetting to 0.
    [[nodiscard]] bool getPersistentGoalTimer() const { return latch_goal_timer_; }

    // ========== Grid Parameter Getters ==========
    /// Metres per occupancy grid cell.
    [[nodiscard]] float getGridScale() const { return grid_scale_; }
    /// Extra clearance (metres) added around obstacles when building the occupancy grid.
    [[nodiscard]] float getBufferScale() const { return buffer_scale_; }

    // ========== Goal Parameter Getters ==========
    /// Goal acceptance radius in metres; -1.0 means derive it automatically (e.g. from node radius).
    [[nodiscard]] float getGoalRadius() const { return goal_radius_; }
    /// Whether a robot that reaches its goal is assigned a new one instead of finishing.
    [[nodiscard]] bool getUseNewGoal() const { return use_new_goal_; }

    // ========== Mode Getters ==========
    [[nodiscard]] INIT_MODE getInitMode() const { return init_mode_; }
    [[nodiscard]] SENSOR_UPDATE_MODE getSensorUpdateMode() const { return sensor_update_mode_; }

    // ========== INIT_MODE String Conversion ==========
    /// @return the enumerator name, or "UNKNOWN" for an out-of-range value.
    [[nodiscard]] static std::string initModeToString(INIT_MODE mode) {
        switch (mode) {
            case INIT_MODE::MANUAL:           return "MANUAL";
            case INIT_MODE::FREE_ALL_RANDOM:  return "FREE_ALL_RANDOM";
            case INIT_MODE::NODE_ALL_RANDOM:  return "NODE_ALL_RANDOM";
            case INIT_MODE::NODE_GROUP_RANDOM: return "NODE_GROUP_RANDOM";
            default:                          return "UNKNOWN";
        }
    }

    /// @return the name of this config's current INIT_MODE.
    [[nodiscard]] std::string initModeToString() const { return initModeToString(init_mode_); }

    /// @return the matching INIT_MODE for a name produced by initModeToString(), or MANUAL if unrecognized.
    [[nodiscard]] static INIT_MODE initModeFromString(const std::string &mode) {
        if (mode == "MANUAL")           return INIT_MODE::MANUAL;
        if (mode == "FREE_ALL_RANDOM")  return INIT_MODE::FREE_ALL_RANDOM;
        if (mode == "NODE_ALL_RANDOM")  return INIT_MODE::NODE_ALL_RANDOM;
        if (mode == "NODE_GROUP_RANDOM") return INIT_MODE::NODE_GROUP_RANDOM;
        return INIT_MODE::MANUAL;
    }

    // ========== Planning Getters ==========
    [[nodiscard]] murosim::planning::Planner::TYPE getPlannerType() const { return planner_type_; }

    // ========== Seed Getter ==========
    /// RNG seed used to construct the simulation's RandomGenerator.
    [[nodiscard]] uint32_t getSeed() const { return seed_; }

    // ========== Time Parameter Setters (with validation) ==========

    /// Set simulation time step (must be positive)
    void setTimeStep(float time_step) {
        if (time_step <= 0.0f) {
            throw std::invalid_argument("time_step must be positive");
        }
        time_step_ = time_step;
    }

    /// Set collision detection interval (minimum: 1e-6)
    void setTimeIntervalCollision(float interval) {
        if (interval < MINIMUM_TIME_INTERVAL) {
            throw std::invalid_argument("time_interval_collision too small (min: 1e-6)");
        }
        time_interval_collision_ = interval;
    }

    /// Set actuator update interval (minimum: 1e-6)
    void setTimeIntervalActuator(float interval) {
        if (interval < MINIMUM_TIME_INTERVAL) {
            throw std::invalid_argument("time_interval_actuator too small (min: 1e-6)");
        }
        time_interval_actuator_ = interval;
    }

    /// Set sensor update interval (minimum: 1e-6)
    void setTimeIntervalSensor(float interval) {
        if (interval < MINIMUM_TIME_INTERVAL) {
            throw std::invalid_argument("time_interval_sensor too small (min: 1e-6)");
        }
        time_interval_sensor_ = interval;
    }

    /// Set time on goal (any value allowed, -1.0 = instant finish)
    void setTimeOnGoal(float time) {
        time_on_goal_ = time;
    }

    /// Set persistent goal timer (true = timer keeps counting after leaving goal)
    void setPersistentGoalTimer(bool persistent) {
        latch_goal_timer_ = persistent;
    }

    // ========== Grid Parameter Setters (with validation) ==========

    /// Set grid scale (must be at least MINIMUM_GRID_SCALE)
    void setGridScale(float scale) {
        if (scale < MINIMUM_GRID_SCALE) {
            throw std::invalid_argument("grid_scale must be at least 0.01");
        }
        grid_scale_ = scale;
    }

    /// Set buffer scale (cannot be negative)
    void setBufferScale(float scale) {
        if (scale < 0.0f) {
            throw std::invalid_argument("buffer_scale cannot be negative");
        }
        buffer_scale_ = scale;
    }

    // ========== Goal Parameter Setters (with validation) ==========

    /// Set goal radius (-1.0 for auto, or positive value)
    void setGoalRadius(float radius) {
        if (radius < -1.0f || (radius > -1.0f && radius <= 0.0f)) {
            throw std::invalid_argument("goal_radius must be -1.0 (auto) or positive");
        }
        goal_radius_ = radius;
    }

    /// Set whether to use new goals
    void setUseNewGoal(bool use_new_goal) {
        use_new_goal_ = use_new_goal;
    }

    // ========== Mode Setters ==========

    /// Set initialization mode
    void setInitMode(INIT_MODE mode) {
        init_mode_ = mode;
    }

    /// Set sensor update mode
    void setSensorUpdateMode(SENSOR_UPDATE_MODE mode) {
        sensor_update_mode_ = mode;
    }

    // ========== Planning Setters ==========

    /// Set planner type
    void setPlannerType(murosim::planning::Planner::TYPE type) {
        planner_type_ = type;
    }

    // ========== Seed Setter ==========

    /// Set random seed
    void setSeed(uint32_t seed) {
        seed_ = seed;
    }

private:
    // ========== Time Parameters ==========
    float time_step_ = 0.1f;
    float time_interval_collision_ = 1e-2f;
    float time_interval_actuator_ = 1e-2f;
    float time_interval_sensor_ = 1e-2f;
    float time_on_goal_ = -1.0f;
    bool latch_goal_timer_ = false;

    // ========== Grid Parameters ==========
    float grid_scale_ = 0.1f;
    float buffer_scale_ = 0.2f;

    // ========== Goal Parameters ==========
    float goal_radius_ = -1.0f;
    bool use_new_goal_ = false;

    // ========== Modes ==========
    INIT_MODE init_mode_ = INIT_MODE::MANUAL;
    SENSOR_UPDATE_MODE sensor_update_mode_ = SENSOR_UPDATE_MODE::SYNCHRONIZED;

    // ========== Planning ==========
    murosim::planning::Planner::TYPE planner_type_ = murosim::planning::Planner::TYPE::NONE;

    // ========== Random Seed ==========
    uint32_t seed_ = std::random_device{}();
};

} // namespace murosim::simulation

#endif // MUROSIM_SIMULATION_SIMULATION_CONFIG_HPP_
