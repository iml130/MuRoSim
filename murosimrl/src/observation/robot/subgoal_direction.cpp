// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/subgoal_direction.hpp"
#include "murosimrl/util/path.hpp"

#include <algorithm>
#include <cmath>

namespace murosimrl::observation::robot
{

SubgoalDirection::SubgoalDirection(const float lookahead_distance, const size_t num_subgoals, const bool normalize)
    : ObservationRobot("subgoal_direction", 1)
    , lookahead_distance_(std::max(0.1f, lookahead_distance))
    , num_subgoals_(std::max(static_cast<size_t>(1), num_subgoals))
    , normalize_(normalize)
{
}

std::vector<size_t> SubgoalDirection::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    return ObservationBase::shape(robots.size(), {num_subgoals_ * 2});
}

void SubgoalDirection::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const size_t stack_size = effectiveStackSize();
    const float segment_distance = lookahead_distance_ / static_cast<float>(num_subgoals_);
    size_t index = 0;

    for (size_t i = 0; i < robots.size(); i++)
    {
        const murosim::robot::Robot &robot = *robots[i];
        const auto &path_points = robot.getPathPoints();
        const Eigen::Vector2f robot_pos = robot.getPosition();

        // Compute all subgoals for this robot
        std::vector<Eigen::Vector2f> subgoals(num_subgoals_);
        std::vector<bool> found(num_subgoals_, false);

        for (size_t s = 0; s < num_subgoals_; s++)
        {
            // subgoals beyond the path clamp to the goal (util::path), so the
            // observation keeps encoding the goal near the end of the episode
            // - the same geometry the along-path reward measures against
            const float target = segment_distance * static_cast<float>(s + 1);
            found[s] = murosimrl::util::path::interpolateAlongPath(robot_pos, path_points, target, subgoals[s]);
        }

        for (size_t j = 0; j < stack_size; j++)
        {
            const murosim::robot::State &state = robot.getStateAtOrOldest(j);
            const Eigen::Affine2f &pose = state.getPose();

            for (size_t s = 0; s < num_subgoals_; s++)
            {
                if (found[s])
                {
                    // Chained direction: robot→p0, p0→p1, p1→p2, ...
                    const Eigen::Vector2f &from = (s == 0) ? pose.translation() : subgoals[s - 1];
                    Eigen::Vector2f direction = pose.rotation().inverse() * (subgoals[s] - from);

                    if (normalize_)
                    {
                        const float norm = direction.norm();
                        if (norm > 1e-6f)
                        {
                            direction /= norm;
                        }
                        else
                        {
                            direction = Eigen::Vector2f::Zero();
                        }
                    }

                    output[index++] = direction.x();
                    output[index++] = direction.y();
                }
                else
                {
                    output[index++] = 0.0f;
                    output[index++] = 0.0f;
                }
            }
        }
    }
}

} // namespace murosimrl::observation::robot
