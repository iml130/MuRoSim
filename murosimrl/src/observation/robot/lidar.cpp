// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/robot/lidar.hpp"
#include "murosimrl/util/robot.hpp"

#include <thread>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <string>

namespace murosimrl::observation::robot
{

Lidar::Lidar(const size_t index_lidar, const float range_max, const bool normalize,
             const size_t stack_size, const size_t num_threads)
    : ObservationRobot(std::string("lidar_") + std::to_string(index_lidar), stack_size)
    , index_lidar_(index_lidar)
    , range_max_(std::max(0.0f, range_max))
    , normalize_(normalize)
    , num_threads_(num_threads == 0 ? std::thread::hardware_concurrency() : num_threads)
{
}

void Lidar::validate(const EntityList &robots) const
{
    ObservationRobot::validate(robots);
    util::robot::requireConsistentLidars(robots, index_lidar_);

    // fillObservation clamps to [range_min, range_max] of the observation; the
    // lidar guarantees range_min < its own range_max, a set range_max must
    // also lie above range_min
    if (0.0f < range_max_)
    {
        for (size_t i = 0; i < robots.size(); i++)
        {
            const float range_min = robots[i]->getLidars()[index_lidar_].getRangeMin();
            if (range_max_ <= range_min)
            {
                throw std::invalid_argument("Lidar observation range_max " + std::to_string(range_max_) +
                    " must be greater than the range_min " + std::to_string(range_min) +
                    " of lidar " + std::to_string(index_lidar_) + " of robot " + std::to_string(i));
            }
        }
    }
}

std::vector<size_t> Lidar::shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const
{
    if (robots.empty()) return {};

    util::robot::requireConsistentLidars(robots, index_lidar_);

    const size_t size_lidar = robots[0]->getLidars()[index_lidar_].getLaserSize();
    return ObservationBase::shape(robots.size(), {size_lidar});
}

void Lidar::compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const
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

            threads.emplace_back(&Lidar::fillObservation, this,
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

void Lidar::computeRange(const EntityList &robots, const size_t begin,
                         const size_t end, float* output) const
{
    const size_t size_lidar = robots[0]->getLidars()[index_lidar_].getLaserSize();
    const EntityList sub(robots.begin() + static_cast<std::ptrdiff_t>(begin),
                         robots.begin() + static_cast<std::ptrdiff_t>(end));
    fillObservation(0, sub.size(), output, sub, effectiveStackSize(), size_lidar);
}

void Lidar::fillObservation(const size_t begin, const size_t end, float* array,
                            const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots,
                            const size_t stack_size, const size_t size_lidar) const
{
    array += begin * stack_size * size_lidar;

    for (size_t i = begin; i < end; i++)
    {
        const murosim::robot::Robot &robot = *robots[i];
        const murosim::sensor::Lidar &lidar = robot.getLidars()[index_lidar_];

        // Readings are +inf (no return) or -inf (too close) outside the
        // sensor range; clamping maps them onto the limits, so the output is
        // always finite. The limits come from the robot's OWN sensor (like
        // VelocityBase), so fleets with different lidars still map range_min
        // to 0 and the upper limit to 1 per robot.
        const float lower = lidar.getRangeMin();
        const float upper = (0.0f < range_max_) ? range_max_ : lidar.getRangeMax();
        const float offset = normalize_ ? lower : 0.0f;
        const float scale = normalize_ ? 1.0f / (upper - lower) : 1.0f;

        for (size_t j = 0; j < stack_size; j++)
        {
            const float* src = robot.getStateAtOrOldest(j).getLidarDistanceAt(index_lidar_).data();

            for (size_t k = 0; k < size_lidar; k++)
            {
                array[k] = (std::clamp(src[k], lower, upper) - offset) * scale;
            }

            array += size_lidar;
        }
    }
}

} // namespace murosimrl::observation::robot
