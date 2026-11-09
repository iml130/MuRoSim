// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_THREADING_THREAD_POOL_HPP_
#define MUROSIM_THREADING_THREAD_POOL_HPP_

#include "murosim/threading/batch.hpp"
#include <thread>
#include <list>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <vector>

namespace murosim::threading {

/// Fixed-size pool of worker threads that run batches of independent
/// std::function<void()> jobs. execute() wraps its job list in a Batch,
/// queues it, and blocks the calling thread until every job in that batch has
/// run (or one has thrown); other callers may queue further batches
/// concurrently, and workers process batches in FIFO order, pulling jobs a
/// few at a time (see Batch::pop) so multiple workers can drain one large
/// batch in parallel. Used by murosim::Simulation and the murosimrl
/// observation/reward sets to step many independent simulations or robots in
/// parallel (see the marlnav downstream consumer). Not copyable; the
/// destructor stops all workers and joins them, waiting for in-flight jobs to
/// finish.
class ThreadPool
{
private:
    std::list<std::shared_ptr<Batch>> batches_;
    std::vector<std::thread> workers_;
    std::atomic<bool> flag_stop_;
    std::atomic<bool> flag_work_available_;
    std::mutex mutex_batch_;
    std::mutex mutex_worker_;
    std::condition_variable cv_worker_;

    /// Worker thread body: repeatedly pulls jobs from the front batch,
    /// executing them and reporting outcomes via Batch::jobFinish/jobFailed,
    /// and blocks on cv_worker_ when no batch is queued, until flag_stop_ is
    /// set (destructor) or work becomes available again.
    void run();
    // Queue a batch, wake a worker, and block until the batch finishes.
    void submitAndWait(const std::shared_ptr<Batch> &batch);
public:
    /// Starts number_of_threads worker threads; 0 (the default) uses
    /// std::thread::hardware_concurrency(), or 1 if that reports 0. If a
    /// worker fails to start, already-started workers are stopped and joined
    /// before the exception propagates, so no ThreadPool is left half-built.
    ThreadPool(const size_t number_of_threads=0);
    /// Signals all workers to stop and joins them, waiting for any job
    /// currently executing to finish first.
    ~ThreadPool();
    ThreadPool(const ThreadPool &other) = delete;
    ThreadPool &operator=(const ThreadPool &other) = delete;
    /// Runs jobs across the pool's worker threads and blocks until all have
    /// completed. If any job throws, that exception is rethrown here once
    /// every job (failed or not) has finished; remaining jobs still run. A
    /// no-op for an empty list.
    void execute(std::list<std::function<void()>> &&jobs);
    /// @copydoc execute(std::list<std::function<void()>>&&)
    void execute(const std::list<std::function<void()>> &jobs);
    /// Number of worker threads started by the constructor.
    size_t numberOfThreads() const;
};

} // namespace murosim::threading

#endif // MUROSIM_THREADING_THREAD_POOL_HPP_
