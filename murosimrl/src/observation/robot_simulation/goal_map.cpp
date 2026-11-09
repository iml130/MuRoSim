// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot_simulation/goal_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "murosimrl/util/simulation.hpp"

namespace murosimrl::observation::robot_simulation
{

GoalMap::GoalMap(const size_t patch_dim, const bool with_buffer)
    : ObservationRobotSimulation("goal_map", 1)
    , patch_dim_(patch_dim)
    , with_buffer_(with_buffer)
{
}

std::vector<size_t> GoalMap::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const size_t batch_size = util::simulation::getNumberOfRobots(simulations);
    return ObservationBase::shape(batch_size, {patch_dim_, patch_dim_});
}

void GoalMap::compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    fillObservationGoalMap(output, simulations);
}

void GoalMap::fillObservationGoalMap(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const float* lut = util::grid::gridValueLut(with_buffer_);
    const size_t step = patch_dim_ * patch_dim_;

    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        const murosim::grid::Map &map = sim->getGridMap();

        for (const auto& robot : sim->getRobots())
        {
            Eigen::Vector2i center;
            map.fromWorldIntoMap(robot->getGoal().getPoint(), center);

            util::grid::extractMapPatch(map, center, patch_dim_, lut, array);
            array += step;
        }
    }
}

} // namespace murosimrl::observation::robot_simulation
