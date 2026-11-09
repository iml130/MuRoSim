// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot_simulation/pose_normalized.hpp"
#include "murosimrl/util/simulation.hpp"
#include "murosim/math/angle.hpp"
#include <algorithm>

namespace murosimrl::observation::robot_simulation
{

PoseNormalized::PoseNormalized(const float normalize_distance, const bool orientation_as_unit_vector, const size_t stack_size)
    : ObservationRobotSimulation("pose_normalized", stack_size)
    , normalize_distance_(std::max(0.0f, normalize_distance))
    , orientation_as_unit_vector_(orientation_as_unit_vector)
{
}

std::vector<size_t> PoseNormalized::shape(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const size_t batch_size = util::simulation::getNumberOfRobots(simulations);
    const size_t feature_size = orientation_as_unit_vector_ ? 4 : 3;
    return ObservationBase::shape(batch_size, {feature_size});
}

void PoseNormalized::compute(const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations, float* output) const
{
    if (orientation_as_unit_vector_)
    {
        fillObservationPoseNormalizedWithOrientationAsUnitVector(output, simulations);
    }
    else
    {
        fillObservationPoseNormalized(output, simulations);
    }
}

void PoseNormalized::fillObservationPoseNormalized(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const float normalize_inv = (0.0f < normalize_distance_) ? 1.0f / normalize_distance_ : 1.0f;
    const Eigen::Vector2f normalize_half = (0.0f < normalize_distance_) ? Eigen::Vector2f(0.5f, 0.5f) : Eigen::Vector2f::Zero();
    const size_t stack_size = effectiveStackSize();
    size_t index = 0;

    for (const auto &sim : simulations)
    {
        const murosim::geometry::World &world = sim->getWorld();
        const Eigen::Vector2f center = (world.getAabbMin() + world.getAabbMax()) * 0.5f;

        for (const auto& robotPtr : sim->getRobots())
        {
            const murosim::robot::Robot &robot = *robotPtr;

            for (size_t stack_index = 0; stack_index < stack_size; stack_index++)
            {
                const Eigen::Affine2f &pose = robot.getStateAtOrOldest(stack_index).getPose();
                const Eigen::Vector2f position = (pose.translation() - center) * normalize_inv + normalize_half;
                array[index++] = position.x();
                array[index++] = position.y();
                array[index++] = murosim::math::rotationAngle(pose);
            }
        }
    }
}

void PoseNormalized::fillObservationPoseNormalizedWithOrientationAsUnitVector(float* array, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    const float normalize_inv = (0.0f < normalize_distance_) ? 1.0f / normalize_distance_ : 1.0f;
    const Eigen::Vector2f normalize_half = (0.0f < normalize_distance_) ? Eigen::Vector2f(0.5f, 0.5f) : Eigen::Vector2f::Zero();
    const size_t stack_size = effectiveStackSize();
    size_t index = 0;

    for (const auto &sim : simulations)
    {
        const murosim::geometry::World &world = sim->getWorld();
        const Eigen::Vector2f center = (world.getAabbMin() + world.getAabbMax()) * 0.5f;

        for (const auto& robotPtr : sim->getRobots())
        {
            const murosim::robot::Robot &robot = *robotPtr;

            for (size_t stack_index = 0; stack_index < stack_size; stack_index++)
            {
                const Eigen::Affine2f &pose = robot.getStateAtOrOldest(stack_index).getPose();
                const Eigen::Vector2f position = (pose.translation() - center) * normalize_inv + normalize_half;
                const Eigen::Vector2f orientation = murosim::math::rotationVector(pose);

                array[index++] = position.x();
                array[index++] = position.y();
                array[index++] = orientation.x();
                array[index++] = orientation.y();
            }
        }
    }
}

} // namespace murosimrl::observation::robot_simulation
