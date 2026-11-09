// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_OF_HPP_
#define MUROSIMRL_OBSERVATION_OF_HPP_

#include "murosimrl/observation/observation_base.hpp"

#include <memory>
#include <stdexcept>
#include <vector>

namespace murosimrl::observation
{

/// Shared observation boilerplate over a list of `Entity` (Robot or Simulation).
/// This is the contract every concrete observation implements: subclasses
/// provide compute() (write one feature vector per item into a caller-owned
/// buffer) and shape() (report the output shape for a given item list);
/// validate() is an optional readiness check run before compute() (e.g.
/// "robots must have at least one state"), defaulting to accepting anything.
/// The single-item and std::vector<float>-returning overloads, as well as
/// batched validation/range computation for ObservationSet, are provided
/// here so each concrete observation only differs in entity type and checks.
template <typename Entity>
class ObservationOf : public ObservationBase
{
protected:
    using EntityPtr = std::shared_ptr<Entity>;
    using EntityList = std::vector<EntityPtr>;

    /// @param name Identifies the observation, e.g. for logging or lookup by name.
    /// @param stack_size Number of stacked past states per item; 0 and 1 are equivalent (no stacking).
    ObservationOf(const std::string &name, const size_t stack_size)
        : ObservationBase(name, stack_size)
    {
    }

    /// Write the observation for every item in `items` into `output`, in item order.
    /// @param items Non-empty, already-validated list of entities to observe.
    /// @param output Caller-owned buffer sized to match shape(items); must not be null.
    virtual void compute(const EntityList &items, float* output) const = 0;

    /// Throw if `items` cannot produce an observation yet. Default: accept all.
    virtual void validate(const EntityList &) const {}

public:
    virtual ~ObservationOf() = default;

    /// Report the output shape for observing `items`.
    /// @param items List of entities the observation would be computed for.
    /// @return Output shape, following the [batch, stack, *feature] convention of ObservationBase::shape.
    virtual std::vector<size_t> shape(const EntityList &items) const = 0;

    /// Compute the observation into a caller-owned buffer.
    /// Contract: `items` must be non-empty (throws std::invalid_argument
    /// otherwise) and ready for observation (validate() throws otherwise).
    /// @param items Entities to observe.
    /// @param output Buffer to write into; must be sized per shape(items).
    void operator()(const EntityList &items, float* output) const
    {
        requireNonEmpty(items);
        validate(items);
        compute(items, output);
    }

    /// Public entry to the readiness checks, for callers (ObservationSet)
    /// that validate once up front and then compute in ranges.
    /// @param items Entities to check; throws if empty or not ready.
    void validateFor(const EntityList &items) const
    {
        requireNonEmpty(items);
        validate(items);
    }

    /// Compute items [begin, end) only. `output` points at the slice for
    /// `begin` — the caller offsets by begin * per-item size. No validation
    /// here (see validateFor); ranges of one validated list stay valid.
    /// Default: sub-list + compute(); ops with a cheaper range path override.
    /// @param items Full item list (already validated by the caller).
    /// @param begin Index of the first item in the range (inclusive).
    /// @param end Index one past the last item in the range (exclusive).
    /// @param output Buffer positioned at the start of this range's slice.
    virtual void computeRange(const EntityList &items, const size_t begin,
                              const size_t end, float* output) const
    {
        const EntityList sub(items.begin() + static_cast<std::ptrdiff_t>(begin),
                             items.begin() + static_cast<std::ptrdiff_t>(end));
        compute(sub, output);
    }

    /// Compute the observation for a single item.
    /// @param item Entity to observe.
    /// @param output Buffer to write into; must be sized per shape(item).
    void operator()(const EntityPtr &item, float* output) const
    {
        operator()(EntityList{item}, output);
    }

    /// Compute the observation for `items` and return it as a newly allocated vector.
    /// @param items Entities to observe; must be non-empty.
    /// @return Flat buffer sized per shape(items) (empty if the feature size is 0).
    std::vector<float> operator()(const EntityList &items) const
    {
        requireNonEmpty(items);

        std::vector<float> result(computeTotalSize(shape(items)));

        // A non-empty item list whose feature size is 0 yields an empty result;
        // skip the write so compute() is never handed a null output buffer.
        if (!result.empty())
        {
            operator()(items, result.data());
        }

        return result;
    }

    /// Compute the observation for a single item and return it as a newly allocated vector.
    /// @param item Entity to observe.
    /// @return Flat buffer sized per shape(item).
    std::vector<float> operator()(const EntityPtr &item) const
    {
        return operator()(EntityList{item});
    }

    /// Report the output shape for observing a single item.
    /// @param item Entity the observation would be computed for.
    /// @return Output shape, per the [batch, stack, *feature] convention of ObservationBase::shape.
    std::vector<size_t> shape(const EntityPtr &item) const
    {
        return shape(EntityList{item});
    }

private:
    /// Throw std::invalid_argument if `items` is empty.
    void requireNonEmpty(const EntityList &items) const
    {
        if (items.empty())
        {
            throw std::invalid_argument("Cannot create observation from an empty input!");
        }
    }
};

} // namespace murosimrl::observation

#endif // MUROSIMRL_OBSERVATION_OF_HPP_
