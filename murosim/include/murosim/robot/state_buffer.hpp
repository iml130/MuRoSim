// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ROBOT_STATE_BUFFER_HPP_
#define MUROSIM_ROBOT_STATE_BUFFER_HPP_

#include "state.hpp"
#include <array>

namespace murosim::robot
{

/// Fixed-capacity ring buffer of State snapshots for one robot. The backing
/// storage is rounded up to the next power of two (index masking instead of
/// modulo); operator[] indexes in reverse chronological order, i.e. index 0
/// is the most recently recorded state.
class StateBuffer
{
private:
    size_t index_;
    size_t size_buffer_;
    size_t size_mask_;
    size_t size_;
    std::vector<State> buffer_;
public:
    /// Constructs a buffer with the default minimum capacity (32 states).
    StateBuffer(/* args */);
    /// Clears all recorded states without changing the allocated capacity.
    void reset();
    /// Moves `state` into the next ring-buffer slot.
    void addState(State &&state);
    /** \brief Advance the ring buffer and return the slot to fill in place. */
    State &nextState();
    /// Number of states currently recorded (0 after reset(), up to getMaxBufferSize()).
    size_t size() const;
    /// State recorded `i` steps ago; 0 is the most recent.
    /// @param i Must be < size().
    const State &operator[](const size_t i) const;
    /// Allocated capacity (a power of two, at least 32); recorded state count saturates here.
    size_t getMaxBufferSize() const;
    /// Resizes the buffer to hold at least `size` states (rounded up to a power of two, minimum
    /// 32) and clears all recorded history.
    void setBufferSize(const size_t size);
};

} // namespace murosim::robot

#endif // MUROSIM_ROBOT_STATE_BUFFER_HPP_
