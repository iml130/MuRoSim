// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/simulation/simulation.hpp"

#include "murosim/simulation/initialisation.hpp"
#include "murosim/robot/util.hpp"
#include "murosim/planning/factory.hpp"
#include "murosim/collision/circle.hpp"
#include "murosim/collision/shape.hpp"
#include "murosim/collision/lidar.hpp"

namespace murosim::simulation
{

namespace
{

// Build the config the scalar convenience constructors describe; all other
// fields keep their SimulationConfig defaults.
SimulationConfig makeConfig(const float grid_scale, const float buffer_scale, const SimulationConfig::INIT_MODE init_mode, const murosim::planning::Planner::TYPE planner_type, const uint32_t seed)
{
    SimulationConfig config;
    config.setGridScale(grid_scale);
    config.setBufferScale(buffer_scale);
    config.setInitMode(init_mode);
    config.setPlannerType(planner_type);
    config.setSeed(seed);
    return config;
}

} // namespace

Simulation::Simulation(const murosim::io::DataContainer &data_container, const float grid_scale, const float buffer_scale, const INIT_MODE init_mode, const murosim::planning::Planner::TYPE planner_type, const uint32_t seed)
    : Simulation(data_container.createWorld(), data_container.createNodeContainer(), data_container.getName(), grid_scale, buffer_scale, init_mode, planner_type, seed)
{
}

Simulation::Simulation(const murosim::geometry::World &world, const murosim::navigation::Container &nodes, const std::string &name, const float grid_scale, const float buffer_scale, const INIT_MODE init_mode, const murosim::planning::Planner::TYPE planner_type, const uint32_t seed)
    : Simulation(world, nodes, name, makeConfig(grid_scale, buffer_scale, init_mode, planner_type, seed))
{
}

Simulation::Simulation(const murosim::io::DataContainer &data_container, const SimulationConfig &config)
    : Simulation(data_container.createWorld(), data_container.createNodeContainer(), data_container.getName(), config)
{
}

Simulation::Simulation(const murosim::geometry::World &world, const murosim::navigation::Container &nodes, const std::string &name, const SimulationConfig &config)
    : world_(world),
      nodes_(nodes),
      map_(world_, nodes_.getNodePoints(), config.getGridScale(), config.getBufferScale()),
      name_(name),
      robots_(),
      random_generator_(config.getSeed()),
      config_(config),
      is_ready_(false),
      is_finished_(false),
      simulated_step_count_(0),
      planner_(murosim::planning::factory::createPlanner(config.getPlannerType()))
{
}

Simulation::Simulation(const Simulation& other):
    world_(other.world_),
    nodes_(other.nodes_),
    map_(other.map_),
    name_(other.name_),
    random_generator_(other.random_generator_),
    config_(other.config_),
    is_ready_(other.is_ready_),
    is_finished_(other.is_finished_),
    simulated_step_count_(other.simulated_step_count_)
{
    // Make deep copy from robots
    robots_.reserve(other.robots_.size());
    for (const std::shared_ptr<murosim::robot::Robot>& robot : other.robots_) {
        robots_.emplace_back(std::make_shared<murosim::robot::Robot>(*robot));
    }

    if (other.planner_ != nullptr) {
        planner_ = other.planner_->clone();
    }
}

Simulation& Simulation::operator=(const Simulation &other)
{
    Simulation temp(other);
    swap(*this, temp);
    return *this;
}

// Move constructor
Simulation::Simulation(Simulation&& other) noexcept :
    world_(std::move(other.world_)),
    nodes_(std::move(other.nodes_)),
    map_(std::move(other.map_)),
    name_(std::move(other.name_)),
    robots_(std::move(other.robots_)),
    random_generator_(std::move(other.random_generator_)),
    config_(std::move(other.config_)),
    is_ready_(other.is_ready_),
    is_finished_(other.is_finished_),
    simulated_step_count_(other.simulated_step_count_),
    planner_(std::move(other.planner_))
{
}

Simulation& Simulation::operator=(Simulation&& other) noexcept
{
    if (this != &other) {
        swap(*this, other);
    }
    return *this;
}

void swap(Simulation& first, Simulation& second) noexcept {
    std::swap(first.world_, second.world_);
    std::swap(first.nodes_, second.nodes_);
    std::swap(first.map_, second.map_);
    std::swap(first.name_, second.name_);
    std::swap(first.robots_, second.robots_);
    std::swap(first.random_generator_, second.random_generator_);
    std::swap(first.config_, second.config_);
    std::swap(first.is_ready_, second.is_ready_);
    std::swap(first.is_finished_, second.is_finished_);
    std::swap(first.simulated_step_count_, second.simulated_step_count_);
    std::swap(first.planner_, second.planner_);
}

bool Simulation::containsRobot(const std::shared_ptr<const murosim::robot::Robot> &robot)
{
    return std::find(robots_.begin(), robots_.end(), robot) != robots_.end();
}

bool Simulation::initRobotPositions()
{
    switch (config_.getInitMode())
    {
    case INIT_MODE::MANUAL:
        return true;

    case INIT_MODE::FREE_ALL_RANDOM:
        return initialisation::selectFreeAllRandom(robots_, random_generator_, world_, map_, config_.getGoalRadius());

    case INIT_MODE::NODE_ALL_RANDOM:
        return initialisation::selectNodeAllRandom(robots_, random_generator_, nodes_, world_, config_.getGoalRadius());

    case INIT_MODE::NODE_GROUP_RANDOM:
        return initialisation::selectNodeGroupRandom(robots_, random_generator_, nodes_, world_, config_.getGoalRadius());

    default:
        return false;
    }
}

bool Simulation::addRobot(const std::shared_ptr<murosim::robot::Robot> &robot)
{
    if (!containsRobot(robot))
    {
        is_ready_ = false;
        robots_.emplace_back(robot);
        return true;
    }
    else
    {
        return false;
    }
}

void Simulation::removeAllRobots()
{
    is_ready_ = false;
    robots_.clear();
}

void Simulation::moveRobots(const float time)
{
    for (auto &robot_ptr : robots_)
    {
        if (!robot_ptr->isFinished() && robot_ptr->getStatusCollision() == murosim::robot::Robot::STATUS_COLLISION::NONE)
        {
            robot_ptr->move(time);
        }
    }
}

void Simulation::detectCollisions()
{
    // robot-vs-robot first: each unordered pair is tested once and a hit
    // flags both robots, so the world pass below can skip them (ROBOT
    // overrides WORLD anyway). Finished robots (parked on their goal or
    // hidden at FLT_MAX by hideNotNeededRobots) and robots that already
    // carry a collision status are never re-flagged, but a still-active
    // robot hitting them is.
    for (size_t i = 0; i < robots_.size(); i++)
    {
        murosim::robot::Robot &robot = *robots_[i];
        const bool robot_active = !robot.isFinished();
        const Eigen::Vector2f position = robot.getPosition();
        const float radius = robot.getCollisionRadius();

        for (size_t j = i + 1; j < robots_.size(); j++)
        {
            murosim::robot::Robot &robot_other = *robots_[j];
            const bool flag_robot = robot_active && robot.getStatusCollision() == murosim::robot::Robot::STATUS_COLLISION::NONE;
            const bool flag_other = !robot_other.isFinished() && robot_other.getStatusCollision() == murosim::robot::Robot::STATUS_COLLISION::NONE;
            if (!flag_robot && !flag_other)
            {
                continue;
            }

            // broad phase: skip the expensive shape test when even the
            // bounding circles do not touch
            const Eigen::Vector2f position_other = robot_other.getPosition();
            if (!murosim::collision::circleWithCircle(position.x(), position.y(), radius, position_other.x(), position_other.y(), robot_other.getCollisionRadius()))
            {
                continue;
            }

            if (murosim::collision::shapeWithShape(robot.getShapeRobot(), robot.getPose(), robot_other.getShapeRobot(), robot_other.getPose()))
            {
                if (flag_robot)
                {
                    robot.setStatusCollision(murosim::robot::Robot::STATUS_COLLISION::ROBOT);
                }
                if (flag_other)
                {
                    robot_other.setStatusCollision(murosim::robot::Robot::STATUS_COLLISION::ROBOT);
                }
            }
        }
    }

    for (auto &robot_ptr : robots_)
    {
        if (!robot_ptr->isFinished() &&
            robot_ptr->getStatusCollision() == murosim::robot::Robot::STATUS_COLLISION::NONE &&
            murosim::collision::shapeWithWorld(robot_ptr->getShapeRobot(), robot_ptr->getPose(), world_))
        {
            robot_ptr->setStatusCollision(murosim::robot::Robot::STATUS_COLLISION::WORLD);
        }
    }
}

void Simulation::updateLidar()
{
    murosim::collision::lidarSweepComplete(robots_, world_, random_generator_);
}

void Simulation::checkGoalStatus()
{
    const float time_on_goal = config_.getTimeOnGoal();
    const float time_step = config_.getTimeStep();
    const bool latch = config_.getPersistentGoalTimer();

    for (auto &robot_ptr : robots_)
    {
        murosim::robot::Robot &robot = *robot_ptr;
        if (!robot.isFinished())
        {
            const float distance_to_goal = murosim::robot::util::distanceToGoal(robot.getPose(), robot.getGoal());

            if (distance_to_goal < robot.getGoal().getGoalRadius())
            {
                if (time_on_goal <= robot.getTimeOnGoal())
                {
                    robot.setStatusState(murosim::robot::Robot::STATUS_STATE::REACHED_GOAL);
                }
                else
                {
                    robot.setStatusState(murosim::robot::Robot::STATUS_STATE::ON_GOAL);
                }

                robot.setTimeOnGoal(time_step + robot.getTimeOnGoal());
            }
            else
            {
                if (!latch || robot.getTimeOnGoal() <= 0.0f)
                {
                    robot.setTimeOnGoal(0.0f);
                }
                else
                {
                    // Persistent mode: keep counting after first touch
                    robot.setTimeOnGoal(time_step + robot.getTimeOnGoal());

                    if (time_on_goal <= robot.getTimeOnGoal())
                    {
                        robot.setStatusState(murosim::robot::Robot::STATUS_STATE::REACHED_GOAL);
                        continue;
                    }
                }
                robot.setStatusState(murosim::robot::Robot::STATUS_STATE::SEARCH_GOAL);
            }
        }
    }
}

void Simulation::updateRobotState()
{
    for (auto &robot_ptr : robots_)
    {
        if (!robot_ptr->isFinished())
        {
            robot_ptr->makeState();
        }
    }
}

void Simulation::checkRobotsFinished()
{
    const bool use_new_goal = config_.getUseNewGoal();

    for (auto &robot_ptr : robots_)
    {
        // collision
        if (robot_ptr->getStatusCollision() != murosim::robot::Robot::STATUS_COLLISION::NONE)
        {
            robot_ptr->finished();
        }

        // When reached goal and no new goal is set
        if (!use_new_goal && robot_ptr->getStatusState() == murosim::robot::Robot::STATUS_STATE::REACHED_GOAL)
        {
            robot_ptr->finished();
        }
    }
}

void Simulation::updateRobotStepCounts()
{
    for (auto &robot_ptr : robots_)
    {
        if (!robot_ptr->isFinished())
        {
            robot_ptr->setSimulatedStepCount(simulated_step_count_);
        }
    }
}

void Simulation::assignNewGoals()
{
    if (config_.getUseNewGoal())
    {
        const INIT_MODE init_mode = config_.getInitMode();

        for (auto &robot_ptr : robots_)
        {
            murosim::robot::Robot &robot = *robot_ptr;

            if (!robot.isFinished() && robot.getStatusState() == murosim::robot::Robot::STATUS_STATE::REACHED_GOAL)
            {
                // a failed reassignment leaves the robot parked on a reached
                // goal forever (the episode never terminates), so escalate it
                // like reset() and Planner::plan() do
                switch (init_mode)
                {
                case INIT_MODE::MANUAL:
                    break;

                case INIT_MODE::FREE_ALL_RANDOM:
                    if (!initialisation::selectNewFree(robot, random_generator_, world_, map_, config_.getGoalRadius()))
                    {
                        throw std::runtime_error("[Simulation::assignNewGoals]: could not select a new free goal position for robot \"" + robot.getName() + "\"");
                    }
                    break;

                case INIT_MODE::NODE_ALL_RANDOM:
                    if (!initialisation::selectNewNodeAll(robot, random_generator_, nodes_, config_.getGoalRadius()))
                    {
                        throw std::runtime_error("[Simulation::assignNewGoals]: could not select a new goal node for robot \"" + robot.getName() + "\"");
                    }
                    break;

                case INIT_MODE::NODE_GROUP_RANDOM:
                    if (!initialisation::selectNewNodeGroup(robot, random_generator_, nodes_, config_.getGoalRadius()))
                    {
                        throw std::runtime_error("[Simulation::assignNewGoals]: could not select a new goal node in group " + std::to_string(robot.getGoal().getGroup()) + " for robot \"" + robot.getName() + "\"");
                    }
                    break;

                default:
                    break;
                }
            }
        }
    }
}

void Simulation::simulate()
{
    // when the simulation is not ready try to reset it
    if (!is_ready_)
    {
        reset();
        return; // Return because reset already performs a simulation step
    }

    // accumulate in double: in float, once time grows past ~interval/ULP the
    // addition becomes a no-op and the loop never terminates (the per-update
    // accumulators below stay float because they are reset every interval)
    double time = 0.0;
    float time_since_last_actuator_update = 0.0f;
    float time_since_last_collision_update = 0.0f;

    const double time_step = static_cast<double>(config_.getTimeStep());
    const float time_interval_actuator = config_.getTimeIntervalActuator();
    const float time_interval_collision = config_.getTimeIntervalCollision();
    const bool continuous_sensor = config_.getSensorUpdateMode() == SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS;
    const float time_interval_sensor = config_.getTimeIntervalSensor();

    // Base loop rate driven by actuator and collision only
    float min_interval = std::min(time_interval_actuator, time_interval_collision);

    // For CONTINUOUS mode, track sensor timing for partial ray updates. Each
    // lidar keeps its own scan phase, so only the elapsed interval is passed.
    float time_since_last_sensor_update = 0.0f;

    while (time < time_step)
    {
        const float current_interval = static_cast<float>(std::min(static_cast<double>(min_interval), time_step - time));

        time_since_last_actuator_update += current_interval;
        time_since_last_collision_update += current_interval;

        // Update actuator if enough time has passed
        if (time_since_last_actuator_update >= time_interval_actuator)
        {
            moveRobots(time_since_last_actuator_update);
            time_since_last_actuator_update = 0.0f;
        }

        // Update collision if enough time has passed
        if (time_since_last_collision_update >= time_interval_collision)
        {
            detectCollisions();
            time_since_last_collision_update = 0.0f;
        }

        // CONTINUOUS mode: partial lidar sweep based on rotation timing
        if (continuous_sensor)
        {
            time_since_last_sensor_update += current_interval;
            if (time_since_last_sensor_update >= time_interval_sensor)
            {
                murosim::collision::lidarSweepTime(robots_, world_, random_generator_, time_since_last_sensor_update);
                time_since_last_sensor_update = 0.0f;
            }
        }

        time += static_cast<double>(current_interval);
    }

    // Perform final updates if there's remaining time
    if (time_since_last_actuator_update > 0.0f)
    {
        moveRobots(time_since_last_actuator_update);
    }

    if (time_since_last_collision_update > 0.0f)
    {
        detectCollisions();
    }

    if (continuous_sensor)
    {
        // Final partial sweep for any remaining sensor time
        if (time_since_last_sensor_update > 0.0f)
        {
            murosim::collision::lidarSweepTime(robots_, world_, random_generator_, time_since_last_sensor_update);
        }
    }
    else
    {
        // SYNCHRONIZED: one complete sweep at the end of the step
        updateLidar();
    }

    checkGoalStatus();
    updateRobotState();

    simulated_step_count_++;

    updateRobotStepCounts();
    checkRobotsFinished();
    assignNewGoals();
    checkSimulationFinished();

}

void Simulation::reset()
{
    for (auto &robot_ptr : robots_)
    {
        robot_ptr->reset();
    }

    is_ready_ = initRobotPositions();

    if (is_ready_)
    {
        // Robot::reset() cleared every lidar, and CONTINUOUS mode builds a scan
        // up over several steps: start the episode with a complete scan of the
        // new poses, so the first observation is never empty or stale.
        // (SYNCHRONIZED mode sweeps completely at the end of simulate() anyway.)
        if (config_.getSensorUpdateMode() == SimulationConfig::SENSOR_UPDATE_MODE::CONTINUOUS)
        {
            murosim::collision::lidarSweepComplete(robots_, world_, random_generator_);
        }

        // Run simulate() to compute initial lidar/state for RL algorithms.
        // Then reset counters to 0 since this is the initial state, not a simulation step.
        simulate();

        is_finished_ = false;
        simulated_step_count_ = 0;

        for (auto &robot_ptr : robots_)
        {
            robot_ptr->setSimulatedStepCount(simulated_step_count_);
        }
    }
    else
    {
        throw std::runtime_error("Simulation reset failed: Robot position initialization unsuccessful.");
    }
}

void Simulation::setConfig(const SimulationConfig &config)
{
    // only re-seed when the seed actually changed, otherwise tweaking an
    // unrelated parameter via setConfig(getConfig()) would rewind the RNG
    // and silently replay the initial random stream
    const bool seed_changed = config.getSeed() != config_.getSeed();

    config_ = config;

    // re-derive the state the constructors build from the config, so the
    // stored config never disagrees with the live objects
    map_ = murosim::grid::Map(world_, nodes_.getNodePoints(), config_.getGridScale(), config_.getBufferScale());
    if (seed_changed)
    {
        random_generator_ = murosim::random::RandomGenerator(config_.getSeed());
    }
    planner_ = murosim::planning::factory::createPlanner(config_.getPlannerType());

    // robots must be re-initialised under the new config; like after
    // construction, the next simulate() performs the reset
    is_ready_ = false;
}

void Simulation::checkSimulationFinished()
{
    is_finished_ = std::all_of(robots_.begin(), robots_.end(),
        [](const auto &robot_ptr) { return robot_ptr->isFinished(); });
}

const murosim::geometry::World &Simulation::getWorld() const
{
    return world_;
}

const murosim::navigation::Container &Simulation::getNodes() const
{
    return nodes_;
}

std::vector<std::shared_ptr<murosim::robot::Robot>>& Simulation::getRobots()
{
    return robots_;
}

const std::vector<std::shared_ptr<murosim::robot::Robot>>& Simulation::getRobots() const
{
    return robots_;
}

size_t Simulation::getRobotSize() const
{
    return robots_.size();
}

const std::shared_ptr<murosim::robot::Robot>& Simulation::getRobotPtrAt(const size_t index) const
{
    assert(index < robots_.size());
    return robots_[index];
}

std::shared_ptr<murosim::robot::Robot>& Simulation::getRobotPtrAt(const size_t index)
{
    assert(index < robots_.size());
    return robots_[index];
}

const murosim::robot::Robot& Simulation::getRobotAt(const size_t index) const
{
    assert(index < robots_.size());
    return *robots_[index];
}

murosim::robot::Robot& Simulation::getRobotAt(const size_t index)
{
    assert(index < robots_.size());
    return *robots_[index];
}
size_t Simulation::getPossibleRobotSize() const
{
    return nodes_.getNodes().size();
}

size_t Simulation::getGroupSize() const
{
    return nodes_.getGroupIndices().size();
}

size_t Simulation::getSimulatedStepCount() const
{
    return simulated_step_count_;
}

const std::string& Simulation::getName() const
{
    return name_;
}

const murosim::grid::Map& Simulation::getGridMap() const
{
    return map_;
}

bool Simulation::isReady() const
{
    return is_ready_;
}

bool Simulation::isFinished() const
{
    return is_finished_;
}

void Simulation::plan()
{
    if (planner_ != nullptr)
    {
        for(std::shared_ptr<murosim::robot::Robot>& robot : robots_)
        {
            if(!robot->isFinished())
            {
                robot->planning(planner_, map_);
            }
        }
    }
}

} // namespace murosim::simulation
