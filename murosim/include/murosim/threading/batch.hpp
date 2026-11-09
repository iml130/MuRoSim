// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_THREADING_BATCH_HPP_
#define MUROSIM_THREADING_BATCH_HPP_

#include <list>
#include <mutex>
#include <condition_variable>
#include <exception>
#include <functional>

namespace murosim::threading {

/// A single unit of work handed to ThreadPool: a queue of jobs plus a
/// completion counter that lets one or more callers block until every job has
/// either run or failed. Worker threads drain the queue via pop() and report
/// each job's outcome through jobFinish()/jobFailed(); a caller blocks in
/// wait() until the counter reaches zero. Internally thread-safe (all methods
/// are guarded by the Batch's own mutex), so a batch may be drained
/// concurrently by several worker threads, as ThreadPool does.
class Batch
{
public:
    /// Takes ownership of jobs; the completion counter starts at jobs.size().
    Batch(std::list<std::function<void()>> &&jobs);
    /// Copies jobs; the completion counter starts at jobs.size().
    Batch(const std::list<std::function<void()>> &jobs);
    Batch(const Batch&) = delete;
    Batch& operator=(const Batch&) = delete;
    /// Removes and returns up to number jobs from the front of the queue.
    /// Returns fewer (possibly zero) if the queue holds less than number.
    /// Thread-safe: callers race to drain the same batch.
    std::list<std::function<void()>> pop(const size_t number);
    /// Records that one job completed successfully, decrementing the
    /// completion counter; wakes wait() once it reaches zero.
    void jobFinish();
    /// Records that one job threw, decrementing the completion counter like
    /// jobFinish(). Only the first exception passed across all jobFailed()
    /// calls is kept and later rethrown by wait().
    void jobFailed(std::exception_ptr exception);
    /// Blocks until every job has called jobFinish() or jobFailed(). If any
    /// job failed, rethrows the first recorded exception; otherwise returns
    /// normally. Returns immediately for an already-empty/finished batch.
    void wait();
private:
    std::list<std::function<void()>> jobs_;
    size_t counter_;
    std::exception_ptr exception_;
    std::condition_variable cv_;
    std::mutex mutex_;
};

} // namespace murosim::threading

#endif // MUROSIM_THREADING_BATCH_HPP_
