// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_REWARD_REWARD_SET_HPP_
#define MUROSIMRL_REWARD_REWARD_SET_HPP_

#include "murosimrl/reward/reward_robot.hpp"
#include "murosim/threading/thread_pool.hpp"

#include <memory>
#include <vector>

namespace murosimrl::reward
{

/// Collection of robot-scope rewards summed in one thread-pool batch.
///
/// compute() fans robot-chunks into a single ThreadPool::execute(); inside a
/// chunk every reward is evaluated in add() order and accumulated per robot,
/// so the sum is bit-identical to serial evaluation regardless of thread
/// count. Must not run concurrently with simulate()/reset()/plan() on the
/// same simulations.
class RewardSet
{
public:
    using RobotList = std::vector<std::shared_ptr<murosim::robot::Robot>>;

    /// @param min_chunk_size Smallest robot count worth its own job: small
    /// lists collapse into fewer (down to one) chunks instead of waking every
    /// worker for a handful of robots. Clamped to at least 1.
    explicit RewardSet(const size_t min_chunk_size = 16);

    /// The configured minimum chunk size (see constructor).
    size_t minChunkSize() const;

    /// Append a reward to the set, in evaluation/summation order. Throws std::invalid_argument if `reward` is null.
    void add(const std::shared_ptr<RewardRobot> &reward);

    /// Number of rewards currently in the set.
    size_t size() const;

    /// The rewards in add() order.
    const std::vector<std::shared_ptr<RewardRobot>> &operators() const;

    /// Output shape for evaluating `robots`: {robots.size(), 1} (one scalar per robot).
    std::vector<size_t> shape(const RobotList &robots) const;

    /// Compute and sum all rewards for `robots` in parallel using `pool`.
    /// @param pool Thread pool used to distribute robot-chunk jobs.
    /// @param robots Robots to evaluate.
    /// @param summed Receives one float per robot: the sum over all rewards in
    /// add() order. With no rewards added, the output is zero-filled.
    void compute(murosim::threading::ThreadPool &pool, const RobotList &robots,
                 float* summed) const;

private:
    size_t min_chunk_size_;
    std::vector<std::shared_ptr<RewardRobot>> rewards_;
};

} // namespace murosimrl::reward

#endif // MUROSIMRL_REWARD_REWARD_SET_HPP_
