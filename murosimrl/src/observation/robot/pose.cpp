// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/pose.hpp"
#include "murosim/math/angle.hpp"

#include <algorithm>

namespace murosimrl::observation::robot
{

Pose::Pose(const bool orientation_as_unit_vector, const size_t stack_size)
    : ObservationRobot("pose", stack_size)
    , orientation_as_unit_vector_(orientation_as_unit_vector)
{
}

std::vector<size_t> Pose::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    const size_t observation_size = orientation_as_unit_vector_ ? 4 : 3;
    return ObservationBase::shape(robots.size(), {observation_size});
}

bool Pose::orientationAsUnitVector() const
{
    return orientation_as_unit_vector_;
}

void Pose::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const size_t stack_size = effectiveStackSize();

    if (orientation_as_unit_vector_)
    {
        size_t index = 0;

        for (size_t i = 0; i < robots.size(); i++)
        {
            for (size_t j = 0; j < stack_size; j++)
            {
                const Eigen::Affine2f pose = robots[i]->getStateAtOrOldest(j).getPose();
                const Eigen::Vector2f position = pose.translation();
                const Eigen::Vector2f orientation = murosim::math::rotationVector(pose);

                output[index++] = position.x();
                output[index++] = position.y();
                output[index++] = orientation.x();
                output[index++] = orientation.y();
            }
        }
    }
    else
    {
        size_t index = 0;

        for (size_t i = 0; i < robots.size(); i++)
        {
            const murosim::robot::Robot &robot = *robots[i];

            for (size_t j = 0; j < stack_size; j++)
            {
                const Eigen::Affine2f pose = robot.getStateAtOrOldest(j).getPose();

                output[index++] = pose.translation().x();
                output[index++] = pose.translation().y();
                output[index++] = murosim::math::rotationAngle(pose);
            }
        }
    }
}

} // namespace murosimrl::observation::robot
