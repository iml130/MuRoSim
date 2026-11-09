// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/threading/batch.hpp"
#include <atomic>
#include <exception>
#include <functional>
#include <list>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using murosim::threading::Batch;
using Jobs = std::list<std::function<void()>>;

constexpr int LARGE_JOB_COUNT = 1000;
constexpr size_t CHUNK = 100;

/// `count` jobs that do nothing.
Jobs idleJobs(const int count)
{
    Jobs jobs;
    for (int i = 0; i < count; i++)
    {
        jobs.push_back([]() {});
    }
    return jobs;
}

/// `count` jobs that each increment `counter`.
Jobs countingJobs(std::atomic<int> &counter, const int count)
{
    Jobs jobs;
    for (int i = 0; i < count; i++)
    {
        jobs.push_back([&counter]() { counter++; });
    }
    return jobs;
}

/// Pops up to `chunk` jobs, runs them and reports each as finished.
/// @return the number of jobs run.
size_t runChunk(Batch &batch, const size_t chunk)
{
    Jobs popped = batch.pop(chunk);
    for (std::function<void()> &job : popped)
    {
        job();
        batch.jobFinish();
    }
    return popped.size();
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(BatchConstructor, RvalueConstructorTakesAllJobs)
{
    Batch batch(idleJobs(2));

    EXPECT_EQ(batch.pop(10).size(), static_cast<size_t>(2));
}

TEST(BatchConstructor, ConstRefConstructorCopiesJobs)
{
    const Jobs jobs = idleJobs(2);

    Batch batch(jobs);

    EXPECT_EQ(jobs.size(), static_cast<size_t>(2)); // source untouched
    EXPECT_EQ(batch.pop(10).size(), static_cast<size_t>(2));
}

TEST(BatchConstructor, EmptyBatchHasNothingToPopAndWaitReturns)
{
    Batch batch{Jobs()};

    EXPECT_TRUE(batch.pop(5).empty());
    EXPECT_NO_THROW(batch.wait()); // nothing outstanding, returns at once
}

// ============================================================================
// Pop Tests
// ============================================================================

TEST(BatchPop, PopSingleJob)
{
    Batch batch(idleJobs(3));

    EXPECT_EQ(batch.pop(1).size(), static_cast<size_t>(1));
}

TEST(BatchPop, PopMultipleJobs)
{
    Batch batch(idleJobs(10));

    EXPECT_EQ(batch.pop(5).size(), static_cast<size_t>(5));
}

TEST(BatchPop, PopMoreThanAvailableReturnsTheRest)
{
    Batch batch(idleJobs(2));

    EXPECT_EQ(batch.pop(10).size(), static_cast<size_t>(2));
}

TEST(BatchPop, PopZeroReturnsNothing)
{
    Batch batch(idleJobs(2));

    EXPECT_TRUE(batch.pop(0).empty());
    EXPECT_EQ(batch.pop(10).size(), static_cast<size_t>(2)); // nothing was consumed
}

TEST(BatchPop, PopDrainsInChunks)
{
    Batch batch(idleJobs(10));

    EXPECT_EQ(batch.pop(3).size(), static_cast<size_t>(3));
    EXPECT_EQ(batch.pop(3).size(), static_cast<size_t>(3));
    EXPECT_EQ(batch.pop(3).size(), static_cast<size_t>(3));
    EXPECT_EQ(batch.pop(3).size(), static_cast<size_t>(1));
    EXPECT_TRUE(batch.pop(3).empty());
}

TEST(BatchPop, PopPreservesJobOrder)
{
    std::vector<int> order;
    Jobs jobs;
    for (int i = 0; i < 5; i++)
    {
        jobs.push_back([i, &order]() { order.push_back(i); });
    }
    Batch batch(std::move(jobs));

    for (std::function<void()> &job : batch.pop(5))
    {
        job();
    }

    EXPECT_EQ(order, (std::vector<int>{0, 1, 2, 3, 4}));
}

TEST(BatchPop, PoppedJobsAreCallable)
{
    std::atomic<int> counter{0};
    Batch batch(countingJobs(counter, 3));

    EXPECT_EQ(runChunk(batch, 3), static_cast<size_t>(3));
    EXPECT_EQ(counter.load(), 3);
}

// ============================================================================
// Job Finish Tests
// ============================================================================

TEST(BatchJobFinish, WaitReturnsOnceEveryJobIsFinished)
{
    Batch batch(idleJobs(5));

    EXPECT_EQ(runChunk(batch, 5), static_cast<size_t>(5));

    EXPECT_NO_THROW(batch.wait());
}

TEST(BatchJobFinish, ExtraJobFinishIsIgnored)
{
    Batch batch(idleJobs(2));

    batch.jobFinish();
    batch.jobFinish();
    batch.jobFinish(); // one more than there are jobs

    EXPECT_NO_THROW(batch.wait());
}

TEST(BatchJobFinish, JobFinishWithoutRunningTheJobStillCountsDown)
{
    std::atomic<int> counter{0};
    Batch batch(countingJobs(counter, 2));

    batch.jobFinish(); // the batch only counts, it does not run jobs itself
    batch.jobFinish();

    EXPECT_NO_THROW(batch.wait());
    EXPECT_EQ(counter.load(), 0);
}

// ============================================================================
// Wait Tests
// ============================================================================

TEST(BatchWait, WaitAfterAllJobsFinished)
{
    std::atomic<int> counter{0};
    Batch batch(countingJobs(counter, 2));
    runChunk(batch, 2);

    batch.wait();

    EXPECT_EQ(counter.load(), 2);
}

TEST(BatchWait, WaitBlocksUntilWorkerThreadFinishes)
{
    std::atomic<int> counter{0};
    Batch batch(countingJobs(counter, 5));
    std::thread worker([&batch]() { runChunk(batch, 5); });

    batch.wait(); // returns only after the worker reported all five

    EXPECT_EQ(counter.load(), 5);
    worker.join();
}

TEST(BatchWait, WaitWithTwoWorkerThreads)
{
    std::atomic<int> counter{0};
    Batch batch(countingJobs(counter, 10));
    std::thread worker1([&batch]() { runChunk(batch, 5); });
    std::thread worker2([&batch]() { runChunk(batch, 5); });

    batch.wait();

    EXPECT_EQ(counter.load(), 10);
    worker1.join();
    worker2.join();
}

TEST(BatchWait, LargeBatchDrainedInChunks)
{
    std::atomic<int> counter{0};
    Batch batch(countingJobs(counter, LARGE_JOB_COUNT));

    size_t chunks = 0;
    while (runChunk(batch, CHUNK) > 0)
    {
        chunks++;
    }
    batch.wait();

    EXPECT_EQ(chunks, static_cast<size_t>(LARGE_JOB_COUNT) / CHUNK);
    EXPECT_EQ(counter.load(), LARGE_JOB_COUNT);
}

// ============================================================================
// Job Failed Tests
// ============================================================================

// Regression: there was no way to surface a job exception; it escaped the
// worker thread and terminated the process.
TEST(BatchJobFailed, WaitRethrowsTheException)
{
    Batch batch(idleJobs(2));

    batch.jobFinish();
    batch.jobFailed(std::make_exception_ptr(std::runtime_error("boom")));

    EXPECT_THROW(batch.wait(), std::runtime_error);
}

TEST(BatchJobFailed, JobFailedCountsAsFinished)
{
    Batch batch(idleJobs(1));

    batch.jobFailed(std::make_exception_ptr(std::runtime_error("boom")));

    EXPECT_THROW(batch.wait(), std::runtime_error); // returned (and threw) instead of blocking
}

TEST(BatchJobFailed, FirstExceptionWins)
{
    Batch batch(idleJobs(2));

    batch.jobFailed(std::make_exception_ptr(std::runtime_error("first")));
    batch.jobFailed(std::make_exception_ptr(std::logic_error("second")));

    // logic_error is not a runtime_error, so the type alone tells which one was kept
    EXPECT_THROW(batch.wait(), std::runtime_error);
}

TEST(BatchJobFailed, WaitWithoutFailureDoesNotThrow)
{
    Batch batch(idleJobs(1));

    batch.jobFinish();

    EXPECT_NO_THROW(batch.wait());
}
