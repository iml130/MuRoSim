// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/reward/reward_set.hpp"

#include <algorithm>
#include <functional>
#include <list>
#include <stdexcept>

namespace murosimrl::reward
{

RewardSet::RewardSet(const size_t min_chunk_size)
    : min_chunk_size_(std::max<size_t>(1, min_chunk_size))
{
}

size_t RewardSet::minChunkSize() const
{
    return min_chunk_size_;
}

void RewardSet::add(const std::shared_ptr<RewardRobot> &reward)
{
    if (!reward)
    {
        throw std::invalid_argument("Cannot add a null reward to a RewardSet!");
    }
    rewards_.emplace_back(reward);
}

size_t RewardSet::size() const
{
    return rewards_.size();
}

const std::vector<std::shared_ptr<RewardRobot>> &RewardSet::operators() const
{
    return rewards_;
}

std::vector<size_t> RewardSet::shape(const RobotList &robots) const
{
    return {robots.size(), 1};
}

void RewardSet::compute(murosim::threading::ThreadPool &pool, const RobotList &robots,
                        float* summed) const
{
    if (robots.empty())
    {
        return;
    }

    if (rewards_.empty())
    {
        std::fill(summed, summed + robots.size(), 0.0f);
        return;
    }

    // Same chunk count heuristic as ObservationSet: enough chunks to keep
    // the pool busy without drowning it in tiny jobs, floored so small
    // lists don't shatter into per-robot jobs.
    const size_t threads = std::max<size_t>(1, pool.numberOfThreads());
    const size_t max_by_grain =
        std::max<size_t>(1, robots.size() / min_chunk_size_);
    const size_t target = std::max<size_t>(
        1, std::min({robots.size(), 3 * threads, max_by_grain}));
    const size_t base = robots.size() / target;
    const size_t remainder = robots.size() % target;

    std::list<std::function<void()>> jobs;

    size_t begin = 0;
    for (size_t c = 0; c < target; c++)
    {
        const size_t end = begin + base + (c < remainder ? 1 : 0);

        jobs.emplace_back([this, &robots, begin = begin, end = end, summed]() {
            const size_t length = end - begin;
            std::vector<float> value(length);
            float* const accumulated = summed + begin;

            std::fill(accumulated, accumulated + length, 0.0f);

            for (const auto &reward : rewards_)
            {
                reward->computeRange(robots, begin, end, value.data());

                for (size_t i = 0; i < length; i++)
                {
                    accumulated[i] += value[i];
                }
            }
        });

        begin = end;
    }

    pool.execute(std::move(jobs));
}

} // namespace murosimrl::reward
