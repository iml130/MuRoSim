// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/simulation/velocity_map.hpp"
#include "murosimrl/util/simulation.hpp"
#include "murosim/grid/drawing.hpp"
#include <algorithm>
#include <cmath>

namespace
{

Eigen::Vector2f fromLocalToGlobalVelocity(const Eigen::Affine2f &pose, const Eigen::Vector3f velocity, const float time_step)
{
    return pose.rotation() * Eigen::Rotation2Df(velocity.z() * time_step * 0.5f) * velocity.head(2);
}

} // anonymous namespace

namespace murosimrl::observation::simulation
{

VelocityMap::VelocityMap(const size_t map_size, const size_t history_size, const bool use_collision_radius, const size_t stack_size)
    : ObservationSimulation("velocity_map", stack_size)
    , map_size_(map_size)
    , history_size_(history_size)
    , use_collision_radius_(use_collision_radius)
{
    if (map_size_ == 0)
    {
        throw std::runtime_error("Argument map_size must not be zero!");
    }
}

std::vector<size_t> VelocityMap::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const size_t channels = (history_size_ == 0) ? 2 : 3;
    return ObservationBase::shape(simulations.size(), {channels, map_size_, map_size_});
}

void VelocityMap::compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    util::simulation::requireGridMapFitSize(simulations, map_size_, "VelocityMap");

    fillObservationVelocity(output, simulations);
}

void VelocityMap::fillIndicesForRobotCollisionRadius(const murosim::grid::Map &map, const Eigen::Vector2f &robot_position, const float robot_collision_radius, const size_t start_x, const size_t start_y, std::vector<size_t> &indices) const
{
    indices.clear();

    if (0.0f < robot_collision_radius)
    {
        // drawing::circleCells is the same rasterization the grid map uses, so
        // robot footprints in the observation match the occupancy map exactly
        // (including the half-cell overlap inflation and the center-cell
        // guarantee for sub-cell radii).
        murosim::grid::drawing::circleCells(map, robot_position, robot_collision_radius,
            [this, start_x, start_y, &indices](const Eigen::Vector2i pixel) {
                indices.push_back((static_cast<size_t>(pixel.y()) + start_y) * map_size_ + static_cast<size_t>(pixel.x()) + start_x);
            });
        return;
    }

    // use_collision_radius_ disabled: mark only the robot's center cell
    Eigen::Vector2i p;
    map.fromWorldIntoMap(robot_position, p);

    if (map.isInMap(p))
    {
        indices.push_back((static_cast<size_t>(p.y()) + start_y) * map_size_ + static_cast<size_t>(p.x()) + start_x);
    }
}

void VelocityMap::fillObservationVelocity(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const size_t stack_size = effectiveStackSize();
    const size_t map_step = map_size_ * map_size_;
    // history_size_ == 0 means no history channel: the history loop degenerates
    // to a single newest-state iteration and the third channel is not written
    const size_t channels = (history_size_ == 0) ? 2 : 3;
    const size_t array_step = channels * map_step;
    const size_t history_size = std::max<size_t>(1, history_size_);
    std::vector<size_t> indices;

    for (const std::shared_ptr<murosim::simulation::Simulation> &sim : simulations)
    {
        const murosim::grid::Map &map = sim->getGridMap();
        const float time_step = sim->getConfig().getTimeStep();
        const size_t start_x = (map_size_ - map.sizeX()) / 2;
        const size_t start_y = (map_size_ - map.sizeY()) / 2;

        for (size_t stack_index = 0; stack_index < stack_size; stack_index++)
        {
            std::fill(array, array + array_step, 0.0f);

            for (size_t history_index = history_size; history_index-- > 0;)
            {
                const float history_value = 1.0f - static_cast<float>(history_index) / static_cast<float>(history_size);

                for (const auto& robotPtr : sim->getRobots())
                {
                    const murosim::robot::Robot &robot = *robotPtr;

                    // No state before the first simulation step - leave zeros
                    if (!robot.hasStates())
                    {
                        continue;
                    }

                    const murosim::robot::State &state = robot.getStateAtOrOldest(stack_index + history_index);
                    const float robot_collision_radius = (use_collision_radius_) ? robot.getCollisionRadius() : -1.0f;
                    const Eigen::Vector2f velocity = fromLocalToGlobalVelocity(state.getPose(), state.getActuatorVelocity(), time_step);

                    fillIndicesForRobotCollisionRadius(map, state.getPose().translation(), robot_collision_radius, start_x, start_y, indices);

                    for (const size_t index : indices)
                    {
                        // raster rows are y-flipped (Map::fromWorldIntoMap), so
                        // negate vy to keep the vector channels in the same
                        // image frame as the spatial channels
                        array[index] = velocity.x();
                        array[index + map_step] = -velocity.y();

                        if (channels == 3)
                        {
                            array[index + map_step * 2] = history_value;
                        }
                    }
                }
            }

            array += array_step;
        }
    }
}

} // namespace murosimrl::observation::simulation
