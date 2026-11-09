// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/simd/array_float.hpp"
#include "murosim/simd/util.hpp"

#include <cassert>
#include <algorithm>

namespace murosim::simd
{

void ArrayFloat::fillWithQuietNan()
{
    // fill with quiet_NaN to ignore the calculation of the simd operation like cmp.
    for (size_t i = 0; i < size_aligned_; i++)
    {
        data_[i] = ArrayFloat::QUIET_NAN;
    }
}

ArrayFloat::ArrayFloat():size_(0), size_aligned_(0), data_(nullptr)
{

}

ArrayFloat::ArrayFloat(const size_t size):size_(size), size_aligned_(0), data_(util::allocAlignedMemoryFloat(size_, size_aligned_))
{
    fillWithQuietNan();
}

ArrayFloat::ArrayFloat(const std::vector<float> &array): size_(array.size()), size_aligned_(0), data_(util::allocAlignedMemoryFloat(size_, size_aligned_))
{
    fillWithQuietNan();
    std::copy(std::begin(array), std::end(array), data_.get());
}

ArrayFloat::ArrayFloat(const ArrayFloat &other): size_(other.size_), size_aligned_(0), data_(util::allocAlignedMemoryFloat(size_, size_aligned_))
{
    std::copy(other.data_.get(), other.data_.get() + other.size_aligned_, data_.get());
}

ArrayFloat &ArrayFloat::operator=(const ArrayFloat& other)
{
    if (this != &other)
    {
        if(size_aligned_ != other.size_aligned_)
        {
            data_.reset(util::allocAlignedMemoryFloat(other.size_, size_aligned_));
        }
        // size_ must be updated even when the aligned capacity matches:
        // different logical sizes can share the same aligned size
        size_ = other.size_;
        std::copy(other.data_.get(), other.data_.get() + other.size_aligned_, data_.get());
    }

    return *this;
}


ArrayFloat::ArrayFloat(ArrayFloat&& other) noexcept
    : size_(other.size_), size_aligned_(other.size_aligned_), data_(std::move(other.data_))
{
    other.size_ = 0;
    other.size_aligned_ = 0;
}

ArrayFloat& ArrayFloat::operator=(ArrayFloat&& other) noexcept
{
    if (this != &other)
    {
        data_ = std::move(other.data_);
        size_ = other.size_;
        size_aligned_ = other.size_aligned_;

        other.size_ = 0;
        other.size_aligned_ = 0;
    }

    return *this;
}

std::vector<float> ArrayFloat::toVector() const
{
    std::vector<float> v;
    v.reserve(size_);
    v.insert(std::begin(v), data_.get(), data_.get() + size_);
    return v;
}

void ArrayFloat::copyToVector(std::vector<float> &v) const
{
    assert(v.size() >= size_);
    std::copy(data_.get(), data_.get() + size_, std::begin(v));
}

ArrayFloat ArrayFloat::subArrayFloat(const size_t begin, const size_t end, const bool backwards) const
{
    assert(begin < size_);
    assert(end <= size_);
    assert(begin <= end);

    ArrayFloat array = ArrayFloat(end - begin);
    if (backwards)
    {
        std::reverse_copy(data_.get() + begin, data_.get() + end, array.data());
    }
    else
    {
        std::copy(data_.get() + begin, data_.get() + end, array.data());
    }

    return array;
}

float ArrayFloat::min() const
{
    return (empty())? ArrayFloat::QUIET_NAN : *std::min_element(data_.get(), data_.get() + size_);
}

float ArrayFloat::max() const
{
    return (empty())? ArrayFloat::QUIET_NAN : *std::max_element(data_.get(), data_.get() + size_);
}

float ArrayFloat::sum() const
{
    return (empty())? ArrayFloat::QUIET_NAN : util::sum(data_.get(), size_);
}

float ArrayFloat::mean() const
{
    return (empty())
            ? ArrayFloat::QUIET_NAN
            : sum() / static_cast<float>(size_);
}

float ArrayFloat::variance() const
{
    return util::variance(data_.get(), size_, mean());
}

bool ArrayFloat::empty() const
{
    return size_ == 0;
}


} // namespace murosim::simd
