// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot_simulation/pose_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "murosimrl/util/simulation.hpp"


namespace murosimrl::observation::robot_simulation
{

PoseMap::PoseMap(const size_t patch_dim, const bool with_buffer, const size_t stack_size)
    : ObservationRobotSimulation("pose_map", stack_size)
    , patch_dim_(patch_dim)
    , with_buffer_(with_buffer)
{
}

std::vector<size_t> PoseMap::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const size_t batch_size = util::simulation::getNumberOfRobots(simulations);
    return ObservationBase::shape(batch_size, {patch_dim_, patch_dim_});
}

void PoseMap::compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    fillObservationPoseMap(output, simulations);
}

void PoseMap::fillObservationPoseMap(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const float* lut = util::grid::gridValueLut(with_buffer_);
    const size_t step = patch_dim_ * patch_dim_;
    const size_t stack_size = effectiveStackSize();

    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        const murosim::grid::Map &map = sim->getGridMap();

        for (const auto& robot : sim->getRobots())
        {
            for (size_t stack_index = 0; stack_index < stack_size; ++stack_index)
            {
                const Eigen::Vector2f position = robot->getStateAtOrOldest(stack_index).getPose().translation();
                Eigen::Vector2i center;
                map.fromWorldIntoMap(position, center);

                util::grid::extractMapPatch(map, center, patch_dim_, lut, array);
                array += step;
            }
        }
    }
}

} // namespace murosimrl::observation::robot_simulation
