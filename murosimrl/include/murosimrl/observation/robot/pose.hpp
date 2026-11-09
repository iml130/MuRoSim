// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_POSE_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_POSE_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Robot pose in world coordinates: position (metres) plus orientation,
/// either as an angle in radians or as a unit vector (cos, sin). Feature size
/// is 3 per stacked state (x, y, angle) or 4 (x, y, cos, sin) when
/// orientation_as_unit_vector is set. Values are not normalized. Reads
/// Robot::getStateAtOrOldest(j).getPose() for each stacked state j.
class Pose : public ObservationRobot
{
private:
    bool orientation_as_unit_vector_;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

public:
    /// @param orientation_as_unit_vector Encode orientation as (cos, sin) instead of a single angle.
    /// @param stack_size Number of stacked past poses per robot; 0 and 1 are equivalent (no stacking).
    Pose(const bool orientation_as_unit_vector, const size_t stack_size);

    /// Per-robot feature size is 3 (x, y, angle) or 4 (x, y, cos, sin), times stack size.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;

    /// Whether orientation is encoded as a (cos, sin) unit vector instead of an angle.
    bool orientationAsUnitVector() const;
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_POSE_HPP_
