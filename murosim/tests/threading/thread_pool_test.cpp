// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/threading/thread_pool.hpp"
#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using murosim::threading::ThreadPool;
using Jobs = std::list<std::function<void()>>;

constexpr size_t THREADS = 4;
constexpr int LARGE_JOB_COUNT = 1000;
constexpr int LIFECYCLES = 100; // pool constructions in the lost-wakeup regression

// A worker takes up to this many jobs per pop (NUMBER_OF_JOBS in
// thread_pool.cpp); jobs this far apart in a batch land on different workers
constexpr int JOBS_PER_WORKER_POP = 5;

// Upper bound for the rendezvous below. It only guards against hanging the
// whole test run if the pool ever serialises the two jobs; nothing is
// asserted about elapsed time.
constexpr std::chrono::seconds HANG_GUARD(10);

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

Jobs throwingJob()
{
    Jobs jobs;
    jobs.push_back([]() { throw std::runtime_error("job failed"); });
    return jobs;
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(ThreadPoolConstructor, DefaultConstructorUsesAtLeastOneThread)
{
    const ThreadPool pool;

    EXPECT_GE(pool.numberOfThreads(), static_cast<size_t>(1));
}

TEST(ThreadPoolConstructor, ZeroThreadsUsesAtLeastOneThread)
{
    const ThreadPool pool(0);

    EXPECT_GE(pool.numberOfThreads(), static_cast<size_t>(1));
}

TEST(ThreadPoolConstructor, SpecificThreadCountIsKept)
{
    const ThreadPool single(1);
    const ThreadPool four(4);
    const ThreadPool eight(8);

    EXPECT_EQ(single.numberOfThreads(), static_cast<size_t>(1));
    EXPECT_EQ(four.numberOfThreads(), static_cast<size_t>(4));
    EXPECT_EQ(eight.numberOfThreads(), static_cast<size_t>(8));
}

// ============================================================================
// Test Fixture
// ============================================================================

class ThreadPoolTest : public ::testing::Test
{
protected:
    ThreadPool pool{THREADS};
    std::atomic<int> counter{0};
};

// ============================================================================
// Execute Tests
// ============================================================================

TEST_F(ThreadPoolTest, ExecuteEmptyListRvalueIsNoOp)
{
    EXPECT_NO_THROW(pool.execute(Jobs()));

    pool.execute(countingJobs(counter, 1)); // still usable afterwards
    EXPECT_EQ(counter.load(), 1);
}

TEST_F(ThreadPoolTest, ExecuteEmptyListConstRefIsNoOp)
{
    const Jobs no_jobs;

    EXPECT_NO_THROW(pool.execute(no_jobs));

    pool.execute(countingJobs(counter, 1));
    EXPECT_EQ(counter.load(), 1);
}

TEST_F(ThreadPoolTest, ExecuteSingleJobRvalue)
{
    pool.execute(countingJobs(counter, 1));

    EXPECT_EQ(counter.load(), 1);
}

TEST_F(ThreadPoolTest, ExecuteMultipleJobsRvalue)
{
    pool.execute(countingJobs(counter, 10));

    EXPECT_EQ(counter.load(), 10);
}

TEST_F(ThreadPoolTest, ExecuteConstRefLeavesJobsReusable)
{
    const Jobs jobs = countingJobs(counter, 10);

    pool.execute(jobs);
    pool.execute(jobs); // the list is copied, so it can be run again

    EXPECT_EQ(jobs.size(), static_cast<size_t>(10));
    EXPECT_EQ(counter.load(), 20);
}

TEST_F(ThreadPoolTest, ExecuteReturnsOnlyWhenEveryJobHasRun)
{
    pool.execute(countingJobs(counter, 10));

    EXPECT_EQ(counter.load(), 10); // checked right after execute() returned
}

TEST_F(ThreadPoolTest, RepeatedExecuteCallsAccumulate)
{
    for (int batch = 0; batch < 5; batch++)
    {
        pool.execute(countingJobs(counter, 10));
    }

    EXPECT_EQ(counter.load(), 50);
}

// ============================================================================
// Parallel Execution Tests
// ============================================================================

TEST_F(ThreadPoolTest, JobsOfOneBatchRunOnDifferentWorkers)
{
    // Job 0 and job JOBS_PER_WORKER_POP are popped by different workers. Each
    // waits for the other to have started, which only completes when both run
    // at the same time; a pool that ran them one after another would stall
    // until the hang guard releases the first one.
    std::atomic<int> arrived{0};
    std::atomic<bool> hang_guard_hit{false};
    const auto rendezvous = [&arrived, &hang_guard_hit]()
    {
        arrived++;
        const auto deadline = std::chrono::steady_clock::now() + HANG_GUARD;
        while (arrived.load() < 2)
        {
            if (std::chrono::steady_clock::now() > deadline)
            {
                hang_guard_hit = true;
                return;
            }
            std::this_thread::yield();
        }
    };
    const auto count = [this]() { counter++; };
    Jobs jobs;
    for (int i = 0; i < 2 * JOBS_PER_WORKER_POP; i++)
    {
        if (i % JOBS_PER_WORKER_POP == 0)
        {
            jobs.push_back(rendezvous);
        }
        else
        {
            jobs.push_back(count);
        }
    }

    pool.execute(std::move(jobs));

    EXPECT_EQ(arrived.load(), 2);
    EXPECT_FALSE(hang_guard_hit);
    EXPECT_EQ(counter.load(), 2 * JOBS_PER_WORKER_POP - 2);
}

TEST_F(ThreadPoolTest, EveryJobRunsExactlyOnce)
{
    std::vector<int> runs(100, 0);
    std::mutex mutex;
    Jobs jobs;
    for (size_t i = 0; i < runs.size(); i++)
    {
        jobs.push_back([i, &runs, &mutex]()
        {
            const std::lock_guard<std::mutex> lock(mutex);
            runs[i]++;
        });
    }

    pool.execute(std::move(jobs));

    for (size_t i = 0; i < runs.size(); i++)
    {
        ASSERT_EQ(runs[i], 1) << "job " << i;
    }
}

TEST_F(ThreadPoolTest, JobsWriteTheirOwnResultSlot)
{
    std::vector<int> results(10, 0);
    Jobs jobs;
    for (int i = 0; i < 10; i++)
    {
        jobs.push_back([i, &results]() { results[static_cast<size_t>(i)] = i * i; }); // distinct slots, no lock needed
    }

    pool.execute(std::move(jobs));

    for (int i = 0; i < 10; i++)
    {
        EXPECT_EQ(results[static_cast<size_t>(i)], i * i) << "job " << i;
    }
}

TEST_F(ThreadPoolTest, LargeNumberOfJobs)
{
    pool.execute(countingJobs(counter, LARGE_JOB_COUNT));

    EXPECT_EQ(counter.load(), LARGE_JOB_COUNT);
}

TEST_F(ThreadPoolTest, ManySmallBatches)
{
    for (int batch = 0; batch < 100; batch++)
    {
        pool.execute(countingJobs(counter, 10));
    }

    EXPECT_EQ(counter.load(), 1000);
}

// ============================================================================
// Single Thread Tests
// ============================================================================

TEST(ThreadPoolSingleThread, RunsEveryJob)
{
    ThreadPool pool(1);
    std::atomic<int> counter{0};

    pool.execute(countingJobs(counter, 10));

    EXPECT_EQ(counter.load(), 10);
}

TEST(ThreadPoolSingleThread, RunsJobsInOrder)
{
    ThreadPool pool(1);
    std::vector<int> order;
    Jobs jobs;
    for (int i = 0; i < 10; i++)
    {
        jobs.push_back([i, &order]() { order.push_back(i); });
    }

    pool.execute(std::move(jobs));

    EXPECT_EQ(order, (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));
}

// ============================================================================
// Lifecycle Tests
// ============================================================================

TEST(ThreadPoolLifecycle, ScopedPoolCompletesJobsBeforeDestruction)
{
    std::atomic<int> counter{0};

    {
        ThreadPool pool(2);
        pool.execute(countingJobs(counter, 5));
    } // destructor joins the workers here

    EXPECT_EQ(counter.load(), 5);
}

TEST(ThreadPoolLifecycle, IdlePoolDestructsWithoutJobs)
{
    EXPECT_NO_THROW(ThreadPool(2));
}

// Regression: the work-available flag was not synchronised with the workers'
// wait predicate, occasionally losing the wakeup and hanging execute() or
// the destructor forever.
TEST(ThreadPoolLifecycle, RepeatedLifecycleDoesNotHang)
{
    for (int round = 0; round < LIFECYCLES; round++)
    {
        ThreadPool pool(2);
        std::atomic<int> counter{0};

        pool.execute(countingJobs(counter, 16));

        ASSERT_EQ(counter.load(), 16) << "round " << round;
    }
}

// ============================================================================
// Exception Tests
// ============================================================================

// Regression: exceptions used to escape the worker thread and
// std::terminate the process.
TEST_F(ThreadPoolTest, JobExceptionPropagatesToExecute)
{
    EXPECT_THROW(pool.execute(throwingJob()), std::runtime_error);
}

TEST_F(ThreadPoolTest, RemainingJobsStillRunAfterException)
{
    Jobs jobs = throwingJob();
    jobs.splice(jobs.end(), countingJobs(counter, 8));

    EXPECT_THROW(pool.execute(std::move(jobs)), std::runtime_error);

    EXPECT_EQ(counter.load(), 8);
}

TEST_F(ThreadPoolTest, PoolUsableAfterException)
{
    EXPECT_THROW(pool.execute(throwingJob()), std::runtime_error);

    pool.execute(countingJobs(counter, 8));

    EXPECT_EQ(counter.load(), 8);
}
