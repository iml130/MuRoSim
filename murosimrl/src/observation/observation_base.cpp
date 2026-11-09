// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimrl/observation/observation_base.hpp"

#include <algorithm>

namespace murosimrl::observation
{

ObservationBase::ObservationBase(const std::string &name, const size_t stack_size)
    : name_(name), stack_size_(stack_size)
{
}

size_t ObservationBase::stackSize() const
{
    return stack_size_;
}

size_t ObservationBase::effectiveStackSize() const
{
    return std::max(static_cast<size_t>(1), stack_size_);
}

size_t ObservationBase::computeTotalSize(const std::vector<size_t> &shape) const
{
    if (shape.empty())
    {
        return 0;
    }

    size_t total = 1;
    for (size_t dim : shape)
    {
        total *= dim;
    }
    return total;
}

const std::string& ObservationBase::name() const
{
    return name_;
}

std::vector<size_t> ObservationBase::shape(const size_t batch_size, const std::vector<size_t> &feature_shape) const
{
    std::vector<size_t> s;

    if (0 < batch_size)
    {
        s.push_back(batch_size);

        if (0 < stack_size_)
        {
            s.push_back(stack_size_);
        }

        for (size_t i = 0; i < feature_shape.size(); i++)
        {
            s.push_back(feature_shape[i]);
        }
    }

    return s;
}

} // namespace murosimrl::observation
