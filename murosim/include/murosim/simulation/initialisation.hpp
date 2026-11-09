// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SIMULATION_INITIALISATION_HPP_
#define MUROSIM_SIMULATION_INITIALISATION_HPP_

#include "murosim/robot/robot.hpp"
#include "murosim/geometry/world.hpp"
#include "murosim/navigation/container.hpp"
#include "murosim/grid/map.hpp"
#include "murosim/random/random_generator.hpp"

#include <random>

/// Helper routines used by Simulation to place robot start/goal poses
/// according to a SimulationConfig::INIT_MODE. The "select*" functions
/// initialise the whole robot set (called from reset()); the "selectNew*"
/// functions pick a single replacement goal for one robot (called from
/// assignNewGoals() when useNewGoal is enabled). None of these are
/// thread-safe with respect to the shared RandomGenerator.
namespace murosim::simulation::initialisation
{

/// Places up to robots.size() robots at random free (non-node) map cells,
/// pairing distinct start/goal circles sized from the largest robot
/// collision radius; any surplus robots are hidden and marked finished.
/// @param goal_radius override for the goal acceptance radius, -1.0 for automatic.
/// @return false if there are no robots, or fewer than 2 valid free circles could be placed.
bool selectFreeAllRandom(std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, murosim::random::RandomGenerator &random_generator, const murosim::geometry::World &world, const murosim::grid::Map &map, const float goal_radius=-1.0f);
/// Places up to robots.size() robots at random distinct navigation nodes for
/// start and goal; any surplus robots are hidden and marked finished.
/// @return false if nodes contains fewer than 2 nodes.
bool selectNodeAllRandom(std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const murosim::geometry::World &world, const float goal_radius=-1.0f);
/// Like selectNodeAllRandom, but start/goal pairs are drawn from within the same navigation node group.
/// @return false if any group has fewer than 2 nodes.
bool selectNodeGroupRandom(std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const murosim::geometry::World &world, const float goal_radius=-1.0f);
/// Assigns robot a new goal node different from its current one (resampled while within the current goal's radius).
/// @return false if no distinct node is found within the internal retry budget.
bool selectNewNodeAll(murosim::robot::Robot &robot, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const float goal_radius=-1.0f);
/// Like selectNewNodeAll, but restricted to nodes in the robot's current goal group.
bool selectNewNodeGroup(murosim::robot::Robot &robot, murosim::random::RandomGenerator &random_generator, const murosim::navigation::Container &nodes, const float goal_radius=-1.0f);
/// Assigns robot a new free-map goal position distinct from and outside its current goal's radius,
/// falling back to an exhaustive scan of the map if random sampling fails.
/// @return false if no acceptable free cell exists on the map.
bool selectNewFree(murosim::robot::Robot &robot, murosim::random::RandomGenerator &random_generator, const murosim::geometry::World &world, const murosim::grid::Map &map, const float goal_radius=-1.0f);

}

#endif // MUROSIM_SIMULATION_INITIALISATION_HPP_
