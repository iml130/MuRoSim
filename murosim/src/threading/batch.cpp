// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/threading/batch.hpp"

namespace murosim::threading
{

Batch::Batch(std::list<std::function<void()>> &&jobs):
    jobs_(std::move(jobs)),
    counter_(jobs_.size())
{
}

Batch::Batch(const std::list<std::function<void()>> &jobs):
    jobs_(jobs),
    counter_(jobs_.size())
{
}

std::list<std::function<void()>> Batch::pop(const size_t number)
{
    // jobs_ is guarded by the Batch's own mutex so Batch is internally
    // thread-safe: the pool already serializes pop() via mutex_batch_, but a
    // direct caller draining a batch from several threads is safe too.
    std::lock_guard<std::mutex> lk(mutex_);

    std::list<std::function<void()>> jobs;

    size_t i = 0;

    while (!jobs_.empty() && i < number)
    {
        jobs.emplace_back(std::move(jobs_.front()));
        jobs_.pop_front();
        i++;
    }

    return jobs;
}

void Batch::jobFinish()
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (counter_ > 0 && --counter_ == 0)
    {
        cv_.notify_all();
    }
}

void Batch::jobFailed(std::exception_ptr exception)
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (!exception_)
    {
        exception_ = exception;
    }
    if (counter_ > 0 && --counter_ == 0)
    {
        cv_.notify_all();
    }
}

void Batch::wait()
{
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait(lk, [this]{ return counter_ == 0; });
    if (exception_)
    {
        std::rethrow_exception(exception_);
    }
}

} // namespace murosim::threading
