// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/simulation/path_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "murosimrl/util/simulation.hpp"

namespace murosimrl::observation::simulation
{

PathMap::PathMap(const size_t map_size, const bool with_buffer)
    : ObservationSimulation("path_map", 1)
    , map_size_(map_size)
    , with_buffer_(with_buffer)
{
    if (map_size_ == 0)
    {
        throw std::runtime_error("Argument map_size must not be zero!");
    }
}

std::vector<size_t> PathMap::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    return ObservationBase::shape(simulations.size(), {map_size_, map_size_});
}

void PathMap::compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    util::simulation::requireGridMapFitSize(simulations, map_size_, "PathMap");
    fillObservationPathMap(output, simulations);
}

void PathMap::fillObservationPathMap(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const float* lut = util::grid::gridValueLut(with_buffer_);
    const size_t step = map_size_ * map_size_;

    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        const murosim::grid::Map &map = sim->getGridMap();
        const Eigen::Vector2i offset = util::grid::embedMapCentered(map, map_size_, lut, array);

        for (const auto& robot : sim->getRobots())
        {
            for (const Eigen::Vector2f& point : robot->getPathPoints())
            {
                Eigen::Vector2i in_map;
                map.fromWorldIntoMap(point, in_map);

                if (map.isInMap(in_map))
                {
                    array[(static_cast<size_t>(in_map.y() + offset.y())) * map_size_ + static_cast<size_t>(in_map.x() + offset.x())] = -1.0f;
                }
            }
        }

        array += step;
    }
}

} // namespace murosimrl::observation::simulation
