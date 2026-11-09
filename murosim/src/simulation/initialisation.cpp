// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/simulation/initialisation.hpp"

#include "murosim/collision/shape.hpp"
#include "murosim/collision/circle.hpp"
#include "murosim/math/constants.hpp"

#include <iostream>
#include <limits>
#include <numeric>

constexpr size_t RANDOM_POSE_TRIES = 25;
constexpr size_t RANDOM_FREE_MAP_TRIES = 1000;
constexpr size_t MAX_NEW_NODE_TRIES = 1000;

bool checkCircleCollision(const murosim::geometry::Circle &circle, const std::vector<murosim::geometry::Circle> &circles, const murosim::geometry::World &world)
{
    for (const auto &other : circles)
    {
        if (murosim::collision::circleWithCircle(circle, other))
        {
            return true;
        }
    }

    return murosim::collision::circleWithWorld(circle, world);
}

size_t randomIndexFromSize(murosim::random::RandomGenerator &random_generator, const size_t size)
{
    return std::uniform_int_distribution<size_t>(0, size - 1)(random_generator);
}

size_t randomIndexFromGroup(murosim::random::RandomGenerator &random_generator, const std::vector<size_t>& group)
{
    return group[std::uniform_int_distribution<size_t>(0, group.size() - 1)(random_generator)];
}

void hideNotNeededRobots(std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const size_t index)
{
    // Set all not needed robots out of world area and set the done flag.
    // finished() takes them out of the step loop (goal checks, movement) and
    // lets checkSimulationFinished complete; without it the UNUSED status is
    // overwritten on the next step and the simulation can never finish.
    for (size_t i = index; i < robots.size(); i++)
    {
        robots[i]->setPose(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0f);
        robots[i]->setStatusState(murosim::robot::Robot::STATUS_STATE::UNUSED);
        robots[i]->finished();
    }
}

bool hasStartAndGoalSameIndexPosition(const std::vector<size_t> &start,const std::vector<size_t> &goal)
{
    for (size_t i = 0; i < start.size(); i++)
    {
        if (start[i] == goal[i])
        {
            return true;
        }
    }

    return false;
}

void shuffleStartAndGoalIndices(std::vector<size_t> &start, std::vector<size_t> &goal, murosim::random::RandomGenerator &random_generator)
{
    std::shuffle(std::begin(start), std::end(start), random_generator);
    std::shuffle(std::begin(goal), std::end(goal), random_generator);

    // shuffle goal until there is no same index position for start and goal node
    while (hasStartAndGoalSameIndexPosition(start, goal))
    {
        std::shuffle(std::begin(goal), std::end(goal), random_generator);
    }
}

bool setRobotNodePoseRandom(murosim::robot::Robot &robot, murosim::random::RandomGenerator &random_generator,const murosim::navigation::Node &node, const murosim::geometry::World &world)
{
    size_t tries = 0;

    // tries to set the random pose for the robot in the node area.
    while(tries < RANDOM_POSE_TRIES)
    {
        robot.setPoseFromNode(node, random_generator);

        if(murosim::collision::shapeWithWorld(robot.getShapeRobot(), robot.getPose(), world))
        {
            tries++;
        }
        else
        {
            return true;
        }
    }

    // random Pose failed. Set the normal goal pose and try.
    robot.setPoseFromNode(node);

    return !murosim::collision::shapeWithWorld(robot.getShapeRobot(), robot.getPose(), world);
}

namespace murosim::simulation::initialisation
{

bool selectFreeAllRandom(std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, murosim::random::RandomGenerator &random_generator, const murosim::geometry::World &world, const murosim::grid::Map &map, const float goal_radius)
{
    if (robots.empty())
    {
        std::cerr << "[initialisation::selectFreeAllRandom]: ERROR no robots to initialised!" << std::endl;
        return false;
    }

    // get the collision radius from the robot for the node radius
    float radius = -std::numeric_limits<float>::max();

    for (const auto &robot_ptr : robots)
    {
        radius = std::max(radius, robot_ptr->getCollisionRadius());
    }

    // add a little space to the circle radius
    radius *= 1.1f;

    std::uniform_real_distribution<float>dis_start_angle{-murosim::math::PI, murosim::math::PI};
    std::uniform_int_distribution<size_t>dis_map_free(0, map.size() - 1);

    std::vector<murosim::geometry::Circle> circles;

    // try to place two circles (start + goal) per robot
    for (size_t attempt = 0; attempt < robots.size() * 2; attempt++)
    {
        for (size_t tries = 0; tries < RANDOM_FREE_MAP_TRIES; tries++)
        {
            // select free map place
            const size_t index_free_place = dis_map_free(random_generator);

            if (map.isFree(index_free_place))
            {
                Eigen::Vector2f position;
                map.fromMapIntoWorld(map.indexToPoint(index_free_place), position);

                const murosim::geometry::Circle circle(position, radius);

                // check if the circle collides with other circles and the world.
                if (!checkCircleCollision(circle, circles, world))
                {
                    circles.emplace_back(circle);
                    break;
                }
            }
        }
    }

    if (circles.size() < 2)
    {
        std::cerr << "[initialisation::selectFreeAllRandom]: ERROR no circles could be place as starts ans goals!" << std::endl;
        return false;
    }

    const size_t number = std::min(robots.size(), circles.size());

    // Place the robots for the start and goal circle
    std::vector<size_t> indices_start(circles.size());
    std::iota(indices_start.begin(), indices_start.end(), 0);

    std::vector<size_t> indices_goal = indices_start;

    shuffleStartAndGoalIndices(indices_start, indices_goal, random_generator);

    // assign the indices to the robots and set the pose and goal
    for (size_t i = 0; i < number; i++)
    {
        if (!setRobotNodePoseRandom(*robots[i], random_generator, murosim::navigation::Node(circles[indices_start[i]], dis_start_angle(random_generator), 0.0f, indices_start[i]), world))
        {
            return false;
        }

        robots[i]->setGoal(murosim::navigation::Node(circles[indices_goal[i]], dis_start_angle(random_generator), 0.0f, indices_goal[i]), goal_radius);
        robots[i]->setStatusState(murosim::robot::Robot::STATUS_STATE::SEARCH_GOAL);
    }

    hideNotNeededRobots(robots, number);

    return true;
}

bool selectNodeAllRandom(std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const murosim::geometry::World &world, const float goal_radius)
{
    const auto& node_list = nodes.getNodes();

    if (node_list.size() < 2)
    {
        std::cerr << "[initialisation::selectNodeAllRandom]: ERROR Need at least 2 nodes!" << std::endl;
        return false;
    }

    const size_t number = std::min(robots.size(), node_list.size());

    // create and fill the indices vector for random initialisation
    std::vector<size_t> indices_start(node_list.size());
    std::iota(indices_start.begin(), indices_start.end(), 0);

    std::vector<size_t> indices_goal = indices_start;

    shuffleStartAndGoalIndices(indices_start, indices_goal, random_generator);

    // assign the indices to the robots and set the pose and goal
    for (size_t i = 0; i < number; i++)
    {
        if (!setRobotNodePoseRandom(*robots[i], random_generator, node_list[indices_start[i]], world))
        {
            return false;
        }

        robots[i]->setGoal(node_list[indices_goal[i]], goal_radius);
        robots[i]->setStatusState(murosim::robot::Robot::STATUS_STATE::SEARCH_GOAL);
    }

    hideNotNeededRobots(robots, number);

    return true;
}

bool selectNodeGroupRandom(std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const murosim::geometry::World &world, const float goal_radius)
{
    const auto& group_indices = nodes.getGroupIndices();
    const auto& node_list = nodes.getNodes();

    // check if every group has at least two nodes.
    for (size_t i = 0; i < group_indices.size(); i++)
    {
        if (group_indices[i].size() < 2)
        {
            std::cerr << "[initialisation::selectNodeGroupRandom]: ERROR The Group '" <<  nodes.lookUpGroup(i) << "' has only " << group_indices[i].size() << " nodes. Need at least 2 nodes per group!" << std::endl;
            return false;
        }
    }

    const size_t number = std::min(robots.size(), node_list.size());

    std::vector<std::pair<size_t, size_t>> start_goal_combination;

    // Iterate through the groups and select the random nodes within the group nodes
    for (size_t i = 0; i < group_indices.size(); i++)
    {
        std::vector<size_t> indices_start = group_indices[i];
        std::vector<size_t> indices_goal = group_indices[i];

        shuffleStartAndGoalIndices(indices_start, indices_goal, random_generator);

        for (size_t j = 0; j < indices_start.size(); j++)
        {
            start_goal_combination.emplace_back(std::make_pair(indices_start[j], indices_goal[j]));
        }
    }

    // shuffle the combination
    std::shuffle(std::begin(start_goal_combination), std::end(start_goal_combination), random_generator);

    // assign the indices to the robots and set the pose and goal
    for (size_t i = 0; i < number; i++)
    {
        if (!setRobotNodePoseRandom(*robots[i], random_generator, node_list[start_goal_combination[i].first], world))
        {
            return false;
        }

        robots[i]->setGoal(node_list[start_goal_combination[i].second], goal_radius);
        robots[i]->setStatusState(murosim::robot::Robot::STATUS_STATE::SEARCH_GOAL);
    }

    hideNotNeededRobots(robots, number);

    return true;
}

bool selectNewNodeAll(murosim::robot::Robot &robot, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const float goal_radius)
{
    const auto& node_list = nodes.getNodes();
    size_t index = 0;
    float distance;
    size_t tries = 0;

    do
    {
        index = randomIndexFromSize(random_generator, node_list.size());
        distance = (node_list[index].getPoint() - robot.getGoal().getPoint()).norm();
        tries++;

        if (tries >= MAX_NEW_NODE_TRIES)
        {
            std::cerr << "[initialisation::selectNewNodeAll]: ERROR Could not find valid node within " << MAX_NEW_NODE_TRIES << " tries!" << std::endl;
            return false;
        }
    // resample while the candidate lies within the old goal's circle,
    // i.e. until a different node is found
    } while (distance < robot.getGoal().getRadius());

    robot.setGoal(node_list[index], goal_radius);

    return true;
}

bool selectNewNodeGroup(murosim::robot::Robot &robot, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const float goal_radius)
{
    const auto& group_indices = nodes.getGroupIndices();
    const auto& node_list = nodes.getNodes();
    size_t index = 0;
    float distance;
    size_t tries = 0;

    do
    {
        index = randomIndexFromGroup(random_generator, group_indices[robot.getGoal().getGroup()]);
        distance = (node_list[index].getPoint() - robot.getGoal().getPoint()).norm();
        tries++;

        if (tries >= MAX_NEW_NODE_TRIES)
        {
            std::cerr << "[initialisation::selectNewNodeGroup]: ERROR Could not find valid node within " << MAX_NEW_NODE_TRIES << " tries!" << std::endl;
            return false;
        }
    // resample while the candidate lies within the old goal's circle,
    // i.e. until a different node is found
    } while (distance < robot.getGoal().getRadius());

    robot.setGoal(node_list[index], goal_radius);

    return true;
}

bool selectNewFree(murosim::robot::Robot &robot, murosim::random::RandomGenerator &random_generator, const murosim::geometry::World &world, const murosim::grid::Map &map, const float goal_radius)
{
    // mirror selectFreeAllRandom: the goal circle is the robot's collision
    // radius with a little space
    const float radius = robot.getCollisionRadius() * 1.1f;

    std::uniform_real_distribution<float> dis_start_angle{-murosim::math::PI, murosim::math::PI};
    std::uniform_int_distribution<size_t> dis_map_free(0, map.size() - 1);

    const auto tryPlaceGoalAt = [&](const size_t index_free_place) -> bool
    {
        if (!map.isFree(index_free_place))
        {
            return false;
        }

        Eigen::Vector2f position;
        map.fromMapIntoWorld(map.indexToPoint(index_free_place), position);

        // reject candidates within the old goal's circle, i.e. require a
        // different place (same rule as selectNewNodeAll/Group)
        if ((position - robot.getGoal().getPoint()).norm() < robot.getGoal().getRadius())
        {
            return false;
        }

        const murosim::geometry::Circle circle(position, radius);

        if (murosim::collision::circleWithWorld(circle, world))
        {
            return false;
        }

        robot.setGoal(murosim::navigation::Node(circle, dis_start_angle(random_generator), 0.0f, index_free_place), goal_radius);
        return true;
    };

    for (size_t tries = 0; tries < RANDOM_FREE_MAP_TRIES; tries++)
    {
        if (tryPlaceGoalAt(dis_map_free(random_generator)))
        {
            return true;
        }
    }

    // Random draws can stochastically miss on crowded maps, and the caller
    // escalates a false return to an exception; scan every cell (from a
    // random offset to avoid biasing the goal towards low indices) so failure
    // means there is genuinely no acceptable free cell.
    const size_t offset = dis_map_free(random_generator);
    for (size_t i = 0; i < map.size(); i++)
    {
        if (tryPlaceGoalAt((offset + i) % map.size()))
        {
            return true;
        }
    }

    std::cerr << "[initialisation::selectNewFree]: ERROR No acceptable free goal position exists on the map!" << std::endl;
    return false;
}

} // namespace murosim::simulation::initialisation
