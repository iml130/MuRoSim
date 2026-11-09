// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_FACTORY_HPP_
#define MUROSIMPROC_FACTORY_HPP_

#include <cstddef>
#include <memory>
#include <murosimproc/generators/generator.hpp>
#include <murosim/simulation/simulation.hpp>
#include <vector>

namespace murosimproc
{

/// Holds a weighted pool of `Generator`s and produces batches of worlds (or ready-to-use `Simulation`s)
/// from it in parallel across a thread pool, retrying a fresh random pick if a chosen generator fails to
/// meet its minimum node count. Each call to `generateDataContainers()`/`generateSimulations()` seeds
/// itself from a default-constructed `RandomGenerator`, so results are **not reproducible** from run to
/// run (no seed can be passed in) — see the "murosimproc world generation is not seed-reproducible" note
/// in the project documentation.
class Factory
{
private:
    size_t num_threads_;
    size_t max_retries_;

    std::vector<std::shared_ptr<generators::Generator>> generators_;
    std::vector<double> weights_;
    std::vector<double> cumulative_weights_;

    /// Throws std::logic_error if no generator has been added via `addGenerator()`.
    void validateNotEmpty() const;
    /// Randomly picks a generator index, weighted by the weights passed to `addGenerator()`.
    size_t selectGenerator(murosim::random::RandomGenerator &random_generator) const;
    /// Picks a weighted-random generator and invokes it, retrying with a newly-picked generator (up to `getMaxRetries()` times) if it throws std::runtime_error (insufficient nodes).
    /// @throws std::logic_error if the factory has no generators.
    /// @throws std::runtime_error if no attempt succeeds within `getMaxRetries()` tries.
    murosim::io::DataContainer generateDataContainer(murosim::random::RandomGenerator &random_generator) const;
    /// Same as `generateDataContainer()`, wrapping the result in a manually-initialized `Simulation`.
    std::shared_ptr<murosim::simulation::Simulation> generateSimulation(murosim::random::RandomGenerator &random_generator, float grid_scale, float buffer_scale) const;

public:
    /// Constructs a factory with no generators yet (add them via `addGenerator()`).
    /// @param num_threads Number of worker threads to use for batch generation; 0 (default) uses `std::thread::hardware_concurrency()` (or 1 if that is unavailable).
    /// @param max_retries Maximum number of generator picks to try per requested world before giving up; must be > 0.
    /// @throws std::invalid_argument if `max_retries == 0`.
    Factory(size_t num_threads = 0, size_t max_retries = 100);

    /// Returns the configured worker-thread count, or the resolved hardware-concurrency value if 0 was passed to the constructor.
    size_t getNumThreads() const;
    /// Returns the configured maximum number of retries per requested world.
    size_t getMaxRetries() const;

    /// Adds `generator` to the pool with relative selection `weight` (must be > 0).
    /// @throws std::invalid_argument if `generator` is null or `weight <= 0`.
    void addGenerator(std::shared_ptr<generators::Generator> generator, double weight);
    /// Same as the weighted overload with `weight = 1.0`.
    void addGenerator(std::shared_ptr<generators::Generator> generator);
    /// Generates `count` worlds in parallel (using `getNumThreads()` threads), each from an independently, non-reproducibly seeded weighted-random generator pick.
    /// @throws std::logic_error if the factory has no generators.
    /// @throws std::runtime_error if generation fails for any of the `count` worlds after `getMaxRetries()` attempts.
    std::vector<murosim::io::DataContainer> generateDataContainers(const size_t count) const;
    /// Same as `generateDataContainers()`, additionally wrapping each result in a manually-initialized `Simulation` constructed with `grid_scale` and `buffer_scale`.
    std::vector<std::shared_ptr<murosim::simulation::Simulation>> generateSimulations(const size_t count, const float grid_scale, const float buffer_scale) const;
};

} // namespace murosimproc

#endif // MUROSIMPROC_FACTORY_HPP_
