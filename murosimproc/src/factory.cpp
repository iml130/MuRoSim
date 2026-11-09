// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/factory.hpp"
#include "murosimproc/generators/generator.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

namespace murosimproc
{

Factory::Factory(size_t num_threads, size_t max_retries)
    : num_threads_(num_threads)
    , max_retries_(max_retries)
{
    if (max_retries_ == 0) [[unlikely]] {
        throw std::invalid_argument("max_retries must be > 0");
    }
}

size_t Factory::getNumThreads() const
{
    if (num_threads_ == 0)
    {
        return std::max(static_cast<size_t>(1),
            static_cast<size_t>(std::thread::hardware_concurrency()));
    }
    return num_threads_;
}

size_t Factory::getMaxRetries() const
{
    return max_retries_;
}

void Factory::validateNotEmpty() const
{
    if (generators_.empty()) [[unlikely]] {
        throw std::logic_error(
            "Cannot generate from empty factory. "
            "Add generators with addGenerator() first.");
    }
}

size_t Factory::selectGenerator(murosim::random::RandomGenerator &random_generator) const
{
    std::uniform_real_distribution<double> distribution(0.0, cumulative_weights_.back());
    const double random_value = distribution(random_generator);
    auto iter = std::lower_bound(cumulative_weights_.begin(), cumulative_weights_.end(), random_value);
    return static_cast<size_t>(std::distance(cumulative_weights_.begin(), iter));
}

murosim::io::DataContainer Factory::generateDataContainer(murosim::random::RandomGenerator &random_generator) const
{
    validateNotEmpty();

    for (size_t attempt = 0; attempt < max_retries_; ++attempt)
    {
        const size_t index = selectGenerator(random_generator);

        try
        {
            return (*generators_[index])(random_generator);
        }
        catch (const std::runtime_error &)
        {
            // Generator failed to produce enough nodes, retry with a new pick
        }
    }

    throw std::runtime_error(
        "Factory failed to generate a world with sufficient nodes after " +
        std::to_string(max_retries_) + " attempts");
}

std::shared_ptr<murosim::simulation::Simulation> Factory::generateSimulation(
    murosim::random::RandomGenerator &random_generator,
    float grid_scale,
    float buffer_scale) const
{
    murosim::io::DataContainer data_container = generateDataContainer(random_generator);

    return std::make_shared<murosim::simulation::Simulation>(
        data_container,
        grid_scale,
        buffer_scale,
        murosim::simulation::Simulation::INIT_MODE::MANUAL);
}

namespace {

template<typename T, typename Func>
std::vector<T> generateParallelImpl(size_t count, size_t num_threads, Func generate_func)
{
    murosim::random::RandomGenerator seed_generator;
    std::vector<murosim::random::RandomGenerator::result_type> seeds(count);
    for (size_t i = 0; i < count; ++i)
        seeds[i] = seed_generator();

    std::vector<std::optional<T>> results(count);
    std::atomic<size_t> next_index{0};
    std::atomic<bool> has_error{false};
    std::string error_message;
    std::mutex error_mutex;

    auto worker = [&]()
    {
        while (!has_error.load(std::memory_order_relaxed))
        {
            const size_t i = next_index.fetch_add(1, std::memory_order_relaxed);
            if (i >= count) break;

            try
            {
                murosim::random::RandomGenerator rng(seeds[i]);
                results[i].emplace(generate_func(rng));
            }
            catch (const std::exception &e)
            {
                std::lock_guard<std::mutex> lock(error_mutex);
                if (!has_error.exchange(true))
                    error_message = e.what();
            }
        }
    };

    const size_t actual_threads = std::min(count, num_threads);
    std::vector<std::thread> threads;
    threads.reserve(actual_threads);
    for (size_t t = 0; t < actual_threads; ++t)
        threads.emplace_back(worker);
    for (auto &t : threads)
        t.join();

    if (has_error)
        throw std::runtime_error(error_message);

    std::vector<T> output;
    output.reserve(count);
    for (auto &result : results)
        output.emplace_back(std::move(result.value()));
    return output;
}

} // anonymous namespace

void Factory::addGenerator(std::shared_ptr<generators::Generator> generator, double weight)
{
    if (!generator) [[unlikely]] {
        throw std::invalid_argument("Cannot add null generator to factory");
    }

    if (weight <= 0.0) [[unlikely]] {
        throw std::invalid_argument(
            "Generator weight must be > 0 (got " +
            std::to_string(weight) + ")");
    }

    generators_.emplace_back(generator);
    weights_.emplace_back(weight);

    const double cumulative_weight = cumulative_weights_.empty()
        ? weight
        : cumulative_weights_.back() + weight;
    cumulative_weights_.emplace_back(cumulative_weight);
}

void Factory::addGenerator(std::shared_ptr<generators::Generator> generator)
{
    addGenerator(generator, 1.0);
}

std::vector<murosim::io::DataContainer> Factory::generateDataContainers(const size_t count) const
{
    validateNotEmpty();
    return generateParallelImpl<murosim::io::DataContainer>(count, getNumThreads(),
        [this](murosim::random::RandomGenerator &rng) {
            return generateDataContainer(rng);
        });
}

std::vector<std::shared_ptr<murosim::simulation::Simulation>> Factory::generateSimulations(const size_t count, const float grid_scale, const float buffer_scale) const
{
    validateNotEmpty();
    return generateParallelImpl<std::shared_ptr<murosim::simulation::Simulation>>(count, getNumThreads(),
        [this, grid_scale, buffer_scale](murosim::random::RandomGenerator &rng) {
            return generateSimulation(rng, grid_scale, buffer_scale);
        });
}

} // namespace murosimproc
