// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/simulation/occupancy_map.hpp"
#include "murosimrl/util/grid.hpp"
#include "murosimrl/util/simulation.hpp"
#include <stdexcept>

namespace murosimrl::observation::simulation
{

OccupancyMap::OccupancyMap(const size_t map_size, const bool with_buffer)
    : ObservationSimulation("occupancy_map", 1)
    , map_size_(map_size)
    , with_buffer_(with_buffer)
{
    if (map_size_ == 0)
    {
        throw std::runtime_error("Argument map_size must not be zero!");
    }
}

std::vector<size_t> OccupancyMap::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    return ObservationBase::shape(simulations.size(), {map_size_, map_size_});
}

void OccupancyMap::compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    util::simulation::requireGridMapFitSize(simulations, map_size_, "OccupancyMap");
    fillObservationOccupancyMap(output, simulations);
}

void OccupancyMap::fillObservationOccupancyMap(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const float* lut = util::grid::gridValueLut(with_buffer_);
    const size_t step = map_size_ * map_size_;

    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        util::grid::embedMapCentered(sim->getGridMap(), map_size_, lut, array);
        array += step;
    }
}

} // namespace murosimrl::observation::simulation
