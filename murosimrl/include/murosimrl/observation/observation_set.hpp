// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_OBSERVATION_SET_HPP_
#define MUROSIMRL_OBSERVATION_OBSERVATION_SET_HPP_

#include "murosimrl/observation/observation_robot.hpp"
#include "murosim/threading/thread_pool.hpp"

#include <memory>
#include <vector>

namespace murosimrl::observation
{

/// Collection of robot-scope observations computed in one thread-pool batch.
///
/// compute() validates every operator on the full robot list up front, then
/// fans (operator x robot-chunk) jobs into a single ThreadPool::execute().
/// Each robot writes a disjoint slice of its operator's output buffer, so the
/// result is bit-identical to serial evaluation regardless of thread count.
/// Must not run concurrently with simulate()/reset()/plan() on the same
/// simulations (the callers' phase ordering already guarantees this).
class ObservationSet
{
public:
    using RobotList = std::vector<std::shared_ptr<murosim::robot::Robot>>;

    /// @param min_chunk_size Smallest robot count worth its own job: small
    /// lists collapse into fewer (down to one) chunks instead of waking every
    /// worker for a handful of robots. Clamped to at least 1.
    explicit ObservationSet(const size_t min_chunk_size = 16);

    /// The configured minimum chunk size (see constructor).
    size_t minChunkSize() const;

    /// Append an observation to the set, in evaluation order. Throws std::invalid_argument if `observation` is null.
    void add(const std::shared_ptr<ObservationRobot> &observation);

    /// Number of observations currently in the set.
    size_t size() const;

    /// The observations in add() order.
    const std::vector<std::shared_ptr<ObservationRobot>> &operators() const;

    /// One shape per operator, in add() order.
    /// @param robots Robots the shapes would be computed for.
    std::vector<std::vector<size_t>> shapes(const RobotList &robots) const;

    /// Compute all observations for `robots` in parallel using `pool`.
    /// @param pool Thread pool used to distribute (operator x robot-chunk) jobs.
    /// @param robots Robots to observe; validated against every operator before any job runs.
    /// @param outputs One buffer per operator (outputs[i] receives operator i's
    /// result, laid out identically to that operator's own operator()). An
    /// entry may be null only if the corresponding operator's total size for
    /// `robots` is zero; throws std::invalid_argument otherwise or if the
    /// size does not match the number of observations.
    void compute(murosim::threading::ThreadPool &pool, const RobotList &robots,
                 const std::vector<float*> &outputs) const;

private:
    size_t min_chunk_size_;
    std::vector<std::shared_ptr<ObservationRobot>> observations_;
};

} // namespace murosimrl::observation

#endif // MUROSIMRL_OBSERVATION_OBSERVATION_SET_HPP_
