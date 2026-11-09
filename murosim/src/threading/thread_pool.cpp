// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT
#include "murosim/threading/thread_pool.hpp"
#include <cstddef>

namespace {
    constexpr size_t NUMBER_OF_JOBS = 5;
}

namespace murosim::threading {

ThreadPool::ThreadPool(const size_t number_of_threads): flag_stop_(false), flag_work_available_(false)
{
    const size_t number = (number_of_threads == 0)
        ? std::max(1u, std::thread::hardware_concurrency())
        : number_of_threads;

    try
    {
        for (size_t i = 0; i < number; i++)
        {
            workers_.emplace_back(&ThreadPool::run, this);
        }
    }
    catch (...)
    {
        // A worker failed to start. Stop and join the ones already running so
        // their std::thread destructors don't std::terminate on joinable
        // threads while this half-built pool unwinds.
        {
            std::lock_guard<std::mutex> lock(mutex_worker_);
            flag_stop_ = true;
        }
        cv_worker_.notify_all();
        for (std::thread &worker : workers_)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }
        throw;
    }
}

ThreadPool::~ThreadPool()
{
    {
        // mutex_worker_ must be held while setting the flag, otherwise the
        // notify can fire between a worker's wait-predicate check and its
        // block, losing the wakeup and hanging join() below.
        std::lock_guard<std::mutex> lock(mutex_worker_);
        flag_stop_ = true;
    }

    cv_worker_.notify_all();

    for (size_t i = 0; i < workers_.size(); i++)
    {
        workers_[i].join();
    }
}

void ThreadPool::run()
{
    while (true)
    {
        std::shared_ptr<Batch> batch;
        std::list<std::function<void()>> jobs;
        bool should_wait = false;

        {
            std::lock_guard<std::mutex> lock(mutex_batch_);

            if (!batches_.empty())
            {
                batch = batches_.front();
                jobs = batch->pop(NUMBER_OF_JOBS);

                if (jobs.empty())
                {
                    batches_.pop_front();
                }
                else
                {
                    cv_worker_.notify_one();
                }
            }
            else if (flag_stop_)
            {
                return;
            }
            else
            {
                flag_work_available_ = false;
                should_wait = true;
            }
        }

        if (should_wait)
        {
            std::unique_lock<std::mutex> lock(mutex_worker_);
            cv_worker_.wait(lock, [this]{ return flag_stop_.load() || flag_work_available_.load(); });
            continue;
        }

        while (!jobs.empty())
        {
            try
            {
                jobs.front()();
            }
            catch (...)
            {
                // Hand the exception to the thread blocked in Batch::wait();
                // letting it escape would std::terminate the process.
                jobs.pop_front();
                batch->jobFailed(std::current_exception());
                continue;
            }
            jobs.pop_front();
            batch->jobFinish();
        }
    }
}

void ThreadPool::submitAndWait(const std::shared_ptr<Batch> &batch)
{
    {
        std::lock_guard<std::mutex> lock(mutex_batch_);
        batches_.push_back(batch);
    }
    {
        // See ~ThreadPool: the flag write must be ordered with the workers'
        // wait-predicate check via mutex_worker_.
        std::lock_guard<std::mutex> lock(mutex_worker_);
        flag_work_available_ = true;
    }
    cv_worker_.notify_one();

    batch->wait();
}

void ThreadPool::execute(std::list<std::function<void()>> &&jobs)
{
    if (!jobs.empty())
    {
        submitAndWait(std::make_shared<Batch>(std::move(jobs)));
    }
}

void ThreadPool::execute(const std::list<std::function<void()>> &jobs)
{
    if (!jobs.empty())
    {
        submitAndWait(std::make_shared<Batch>(jobs));
    }
}

size_t ThreadPool::numberOfThreads() const
{
    return workers_.size();
}

} // namespace murosim::threading
