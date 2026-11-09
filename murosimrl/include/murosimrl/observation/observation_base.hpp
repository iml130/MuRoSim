// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_BASE_HPP_
#define MUROSIMRL_OBSERVATION_BASE_HPP_

#include <string>
#include <vector>
#include <cstddef>

namespace murosimrl::observation
{

/// Common, entity-agnostic state and helpers shared by every observation.
/// Holds the observation's name and its temporal stack size (how many past
/// states are concatenated into one feature vector; 0 or 1 both mean "no
/// stacking"). Subclasses (via ObservationOf) implement the actual reading
/// of simulation state and use the helpers here to compute consistent
/// output shapes and buffer sizes.
class ObservationBase
{
private:
    std::string name_;
    size_t stack_size_;

protected:
    /// @param name Identifies the observation, e.g. for logging or lookup by name.
    /// @param stack_size Number of stacked past states per item; 0 and 1 are equivalent (no stacking).
    ObservationBase(const std::string &name, const size_t stack_size);

    /// Build a shape vector as [batch_size, stack_size (if set), *feature_shape], or empty if batch_size is 0.
    /// @param batch_size Number of items (robots or simulations) in the batch.
    /// @param feature_shape Shape of a single, non-stacked observation for one item.
    /// @return The full output shape, or an empty vector when batch_size is 0.
    std::vector<size_t> shape(const size_t batch_size, const std::vector<size_t> &feature_shape) const;

    /// Number of stacked states to read per item, treating an unset (0) stack
    /// size as 1. Use this in compute() so the data layout matches shape().
    size_t effectiveStackSize() const;

    /// Product of all dimensions in a shape vector; 0 for an empty shape.
    /// @param shape Shape to reduce.
    /// @return Total element count.
    size_t computeTotalSize(const std::vector<size_t> &shape) const;

public:
    virtual ~ObservationBase() = default;

    /// The observation's name, as given at construction.
    const std::string& name() const;

    /// The configured stack size (may be 0, meaning "no stacking").
    size_t stackSize() const;
};

} // namespace murosimrl::observation

#endif // MUROSIMRL_OBSERVATION_BASE_HPP_
