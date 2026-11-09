// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/observation_set.hpp"

#include <algorithm>
#include <functional>
#include <list>
#include <stdexcept>

namespace murosimrl::observation
{

namespace
{

size_t totalSize(const std::vector<size_t> &shape)
{
    if (shape.empty()) return 0;

    size_t total = 1;
    for (const size_t dim : shape)
    {
        total *= dim;
    }
    return total;
}

// Chunk boundaries over `count` items: enough chunks that the small
// operators fill the gaps between the heavy ones, without drowning the
// pool's chunked job pickup in tiny jobs. min_chunk_size floors the items
// per chunk so small lists don't shatter into per-robot jobs.
std::vector<std::pair<size_t, size_t>> makeChunks(const size_t count, const size_t threads,
                                                  const size_t min_chunk_size)
{
    const size_t max_by_grain = std::max<size_t>(1, count / std::max<size_t>(1, min_chunk_size));
    const size_t target = std::max<size_t>(
        1, std::min({count, 3 * std::max<size_t>(1, threads), max_by_grain}));
    const size_t base = count / target;
    const size_t remainder = count % target;

    std::vector<std::pair<size_t, size_t>> chunks;
    chunks.reserve(target);

    size_t begin = 0;
    for (size_t c = 0; c < target; c++)
    {
        const size_t end = begin + base + (c < remainder ? 1 : 0);
        chunks.emplace_back(begin, end);
        begin = end;
    }
    return chunks;
}

} // anonymous namespace

ObservationSet::ObservationSet(const size_t min_chunk_size)
    : min_chunk_size_(std::max<size_t>(1, min_chunk_size))
{
}

size_t ObservationSet::minChunkSize() const
{
    return min_chunk_size_;
}

void ObservationSet::add(const std::shared_ptr<ObservationRobot> &observation)
{
    if (!observation)
    {
        throw std::invalid_argument("Cannot add a null observation to an ObservationSet!");
    }
    observations_.emplace_back(observation);
}

size_t ObservationSet::size() const
{
    return observations_.size();
}

const std::vector<std::shared_ptr<ObservationRobot>> &ObservationSet::operators() const
{
    return observations_;
}

std::vector<std::vector<size_t>> ObservationSet::shapes(const RobotList &robots) const
{
    std::vector<std::vector<size_t>> result;
    result.reserve(observations_.size());

    for (const auto &observation : observations_)
    {
        result.emplace_back(observation->shape(robots));
    }
    return result;
}

void ObservationSet::compute(murosim::threading::ThreadPool &pool, const RobotList &robots,
                             const std::vector<float*> &outputs) const
{
    if (outputs.size() != observations_.size())
    {
        throw std::invalid_argument("ObservationSet::compute expects one output per observation!");
    }

    if (observations_.empty())
    {
        return;
    }

    // Validate everything before any job is submitted, so failures surface
    // as a plain exception instead of mid-batch.
    for (const auto &observation : observations_)
    {
        observation->validateFor(robots);
    }

    const std::vector<std::pair<size_t, size_t>> chunks =
        makeChunks(robots.size(), pool.numberOfThreads(), min_chunk_size_);

    std::list<std::function<void()>> jobs;

    for (size_t i = 0; i < observations_.size(); i++)
    {
        const ObservationRobot &observation = *observations_[i];

        const size_t total = totalSize(observation.shape(robots));
        if (total == 0)
        {
            continue; // zero-size feature: nothing to write
        }

        float* const output = outputs[i];
        if (output == nullptr)
        {
            throw std::invalid_argument("ObservationSet::compute got a null output for a non-empty observation!");
        }

        const size_t stride = total / robots.size();

        for (const auto &[begin, end] : chunks)
        {
            jobs.emplace_back([&observation, &robots, first = begin, last = end, output, stride]() {
                observation.computeRange(robots, first, last, output + first * stride);
            });
        }
    }

    if (!jobs.empty())
    {
        pool.execute(std::move(jobs));
    }
}

} // namespace murosimrl::observation
