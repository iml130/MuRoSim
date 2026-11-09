// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/lidar_point.hpp"
#include "murosimrl/util/robot.hpp"

#include <thread>
#include <vector>
#include <algorithm>

namespace murosimrl::observation::robot
{

LidarPoint::LidarPoint(const size_t index_lidar, const bool local,
                       const size_t stack_size, const size_t num_threads)
    : ObservationRobot(std::string("lidar_point_") + std::to_string(index_lidar), stack_size)
    , index_lidar_(index_lidar)
    , local_(local)
    , num_threads_(num_threads == 0 ? std::thread::hardware_concurrency() : num_threads)
{
}

void LidarPoint::validate(const EntityList &robots) const
{
    ObservationRobot::validate(robots);
    util::robot::requireConsistentLidars(robots, index_lidar_);
}

std::vector<size_t> LidarPoint::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    if (robots.empty()) return {};

    util::robot::requireConsistentLidars(robots, index_lidar_);

    const size_t size_lidar = robots[0]->getLidars()[index_lidar_].getLaserSize();
    return ObservationBase::shape(robots.size(), {2, size_lidar});
}

void LidarPoint::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
{
    const size_t size_lidar = robots[0]->getLidars()[index_lidar_].getLaserSize();

    const size_t stack_size = effectiveStackSize();

    // Scale threads with workload, but cap at num_threads_ and robot count
    const size_t effective_threads = std::min({num_threads_, robots.size()});

    if (effective_threads > 1)
    {
        std::vector<std::thread> threads;
        threads.reserve(effective_threads - 1);

        const size_t robots_per_thread = robots.size() / effective_threads;
        const size_t remainder = robots.size() % effective_threads;

        size_t begin = 0;
        for (size_t t = 0; t < effective_threads - 1; t++)
        {
            // Distribute remainder across first threads
            const size_t chunk = robots_per_thread + (t < remainder ? 1 : 0);
            const size_t end = begin + chunk;

            threads.emplace_back(&LidarPoint::fillObservation, this,
                begin, end, output, std::cref(robots), stack_size, size_lidar);

            begin = end;
        }

        // Main thread handles the last chunk
        fillObservation(begin, robots.size(), output, robots, stack_size, size_lidar);

        // Wait for all threads
        for (auto& thread : threads)
        {
            thread.join();
        }
    }
    else
    {
        fillObservation(0, robots.size(), output, robots, stack_size, size_lidar);
    }
}

void LidarPoint::computeRange(const EntityList &robots, const size_t begin,
                              const size_t end, float* output) const
{
    const size_t size_lidar = robots[0]->getLidars()[index_lidar_].getLaserSize();
    const EntityList sub(robots.begin() + static_cast<std::ptrdiff_t>(begin),
                         robots.begin() + static_cast<std::ptrdiff_t>(end));
    fillObservation(0, sub.size(), output, sub, effectiveStackSize(), size_lidar);
}

void LidarPoint::fillObservation(const size_t begin, const size_t end, float* array,
                                 const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots,
                                 const size_t stack_size, const size_t size_lidar) const
{
    array += begin * stack_size * size_lidar * 2;

    // "Local" points are in the LIDAR frame (distance times unit beam
    // direction); the mounting pose lidar.getPose() is only applied in the
    // global branch. For a lidar mounted off-center or rotated this is NOT
    // the robot frame. Readings are clamped to the sensor range first, so
    // +inf (no return) gives a point at range_max and -inf (too close) one
    // at range_min: every point is finite.
    if (local_)
    {
        for (size_t i = begin; i < end; i++)
        {
            const murosim::robot::Robot &robot = *robots[i];
            const murosim::sensor::Lidar &lidar = robot.getLidars()[index_lidar_];
            const murosim::simd::ArrayFloat &laser_x = lidar.getArrayLaserX();
            const murosim::simd::ArrayFloat &laser_y = lidar.getArrayLaserY();

            for (size_t j = 0; j < stack_size; j++)
            {
                const murosim::simd::ArrayFloat &distance = robot.getStateAtOrOldest(j).getLidarDistanceAt(index_lidar_);

                for (size_t k = 0; k < size_lidar; k++)
                {
                    array[k] = lidar.clampDistance(distance[k]) * laser_x[k];
                }

                array += size_lidar;

                for (size_t k = 0; k < size_lidar; k++)
                {
                    array[k] = lidar.clampDistance(distance[k]) * laser_y[k];
                }

                array += size_lidar;
            }
        }
    }
    else
    {
        for (size_t i = begin; i < end; i++)
        {
            const murosim::robot::Robot &robot = *robots[i];
            const murosim::sensor::Lidar &lidar = robot.getLidars()[index_lidar_];
            const murosim::simd::ArrayFloat &laser_x = lidar.getArrayLaserX();
            const murosim::simd::ArrayFloat &laser_y = lidar.getArrayLaserY();
            const Eigen::Affine2f pose_lidar = lidar.getPose();

            for (size_t j = 0; j < stack_size; j++)
            {
                const murosim::robot::State &state = robot.getStateAtOrOldest(j);
                const murosim::simd::ArrayFloat &distance = state.getLidarDistanceAt(index_lidar_);
                const Eigen::Affine2f &pose = state.getPose() * pose_lidar;

                for (size_t k = 0; k < size_lidar; k++)
                {
                    const Eigen::Vector2f point = pose * (lidar.clampDistance(distance[k]) * Eigen::Vector2f(laser_x[k], laser_y[k]));

                    array[k] = point.x();
                    array[k + size_lidar] = point.y();
                }

                array += size_lidar * 2;
            }
        }
    }
}

} // namespace murosimrl::observation::robot
