// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/robot/state_buffer.hpp"

namespace {
    constexpr size_t MIN_BUFFER_STATE_SIZE = 32;
}

namespace murosim::robot
{

StateBuffer::StateBuffer()
{
    setBufferSize(MIN_BUFFER_STATE_SIZE);
}

void StateBuffer::addState(State &&state)
{
    nextState() = std::move(state);
}

State &StateBuffer::nextState()
{
    if (size_ < size_buffer_)
    {
        index_ = size_;
        size_++;
    }
    else
    {
        index_ = (index_ + 1) & size_mask_;
    }

    return buffer_[index_];
}

void StateBuffer::reset()
{
    size_ = 0;
    index_ = 0;
}

void StateBuffer::setBufferSize(const size_t size)
{
    reset();

    const size_t size_min = std::max(size, MIN_BUFFER_STATE_SIZE);

    size_buffer_ = 2;
    size_mask_ = std::numeric_limits<size_t>::max() << 1;

    while(size_buffer_ < size_min)
    {
        size_buffer_ *= 2;
        size_mask_ = size_mask_ << 1;
    }

    size_mask_ = ~size_mask_;
    buffer_.resize(size_buffer_);
    buffer_.shrink_to_fit();
}

const State &StateBuffer::operator[](const size_t i) const
{
    // check if the index is in the range of the circular buffer
    assert(i < size_);

    // - i because of the reverse order.
    return buffer_[(index_ - i) & size_mask_];
}

size_t StateBuffer::size() const
{
    return size_;
}

size_t StateBuffer::getMaxBufferSize() const
{
    return size_buffer_;
}


} // namespace murosim::robot
