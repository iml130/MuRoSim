// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT
//
// Portions of this file are adapted from fast-wfc
// (https://github.com/math-fehr/fast-wfc), Copyright (c) 2018-2019 Mathieu Fehr
// and Nathanaël Courant, released under the MIT License. The upstream notice
// is reproduced in THIRD_PARTY_LICENSES.md.

#ifndef MUROSIMPROC_UTIL_ARRAY2D_HPP_
#define MUROSIMPROC_UTIL_ARRAY2D_HPP_

#include <cassert>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace murosimproc::util
{


/// Dense, row-major 2D grid of `T` backed by a single `std::vector`.
/// Used throughout murosimproc to store per-cell grid data (occupancy, tile ids, colors, ...).
/// Element access via `operator[]`/`operator()` only bounds-checks with `assert` (disabled in release builds).
template <typename T>
class Array2d {
private:
    std::vector<T> array_;
    size_t size_x_;
    size_t size_y_;

public:
    /// Constructs an empty (0x0) array.
    Array2d(): array_(), size_x_(0), size_y_(0)
    {

    }

    /// Constructs an `x` by `y` array with default-initialized elements.
    Array2d(const size_t x, const size_t y) : array_(x * y), size_x_(x), size_y_(y)
    {

    }

    /// Constructs an `x` by `y` array with every element set to `value`.
    Array2d(const size_t x, const size_t y, const T &value) : array_(x * y, value), size_x_(x), size_y_(y)
    {

    }

    /// Constructs an `x` by `y` array by copying row-major data.
    /// @param data Row-major element data; its size must equal `x * y`.
    /// @throws std::invalid_argument if `data.size() != x * y`.
    Array2d(const size_t x, const size_t y, const std::vector<T> &data) : array_(x * y), size_x_(x), size_y_(y)
    {
        if (data.size() != array_.size())
        {
            throw std::invalid_argument("Array2d data size must equal x * y!");
        }
        std::copy(std::cbegin(data), std::cend(data), std::begin(array_));
    }

    /// Accesses the element at flat `index` (row-major).
    inline T& operator[](const size_t index)
    {
        assert(index < array_.size());
        return array_[index];
    }

    /// Accesses the element at flat `index` (row-major).
    inline  const T& operator[](const size_t index) const
    {
        assert(index < array_.size());
        return array_[index];
    }

    /// Accesses the element at grid coordinates `(x, y)`.
    inline T& operator()(const size_t x, const size_t y)
    {
        assert(x < size_x_);
        assert(y < size_y_);
        return array_[x + y * size_x_];
    }

    /// Accesses the element at grid coordinates `(x, y)`.
    inline const T& operator()(const size_t x, const size_t y) const
    {
        assert(x < size_x_);
        assert(y < size_y_);
        return array_[x + y * size_x_];
    }

    /// Compares two arrays for equal size and equal elements in row-major order.
    inline bool operator== (const Array2d<T> &other) const
    {
        if (size_x_ != other.size_x_ || size_y_ != other.size_y_)
        {
            return false;
        }

        for (size_t i = 0; i < array_.size(); i++)
        {
            if (array_[i] != other.array_[i])
            {
                return false;
            }
        }

        return true;
    }

    /// Converts grid coordinates `(x, y)` to the flat row-major index used by `operator[]`.
    inline size_t toIndex(const size_t x, const size_t y) const
    {
        assert(x < size_x_);
        assert(y < size_y_);
        return x + y * size_x_;
    }

    /// Returns the total number of elements (`sizeX() * sizeY()`).
    inline size_t size() const
    {
        return array_.size();
    }

    /// Returns the width (number of columns).
    inline size_t sizeX() const
    {
        return size_x_;
    }

    /// Returns the height (number of rows).
    inline size_t sizeY() const
    {
        return size_y_;
    }

    /// Returns whether the array holds zero elements.
    inline bool empty() const
    {
        return array_.empty();
    }

    /// Copies out the `width` by `height` sub-region starting at `(x, y)` as a new array.
    /// @throws Undefined behaviour (assertion in debug builds) if the region exceeds the array bounds.
    inline Array2d<T> subArray2d(const size_t x, const size_t y, const size_t width, const size_t height) const
    {
        assert(x + width  <= size_x_);
        assert(y + height <= size_y_);

        Array2d<T> sub_array(width, height);

        for (size_t row = 0; row < height; row++)
        {
            size_t src_index = (y + row) * size_x_ + x;
            const T* src_ptr = &array_[src_index];
            T* dst_ptr = &sub_array(0, row);
            std::copy(src_ptr, src_ptr + width, dst_ptr);
        }

        return sub_array;
    }

    /// Sets every element to `value`.
    inline void fill(const T &value)
    {
        std::fill(std::begin(array_), std::end(array_), value);
    }
};

} // namespace murosimproc::util

namespace std {

/// Combines the hashes of all elements (order-dependent) so an `Array2d<T>` can be used as a hash-map/set key.
template <typename T> struct hash<murosimproc::util::Array2d<T>>
{
    std::size_t operator()(const murosimproc::util::Array2d<T>& array) const
    {
        std::size_t seed = 0;
        std::hash<T> hasher;
        for(size_t i = 0; i < array.size(); i++)
        {
            seed ^= hasher(array[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        return seed;
    }
};

}  // namespace std

#endif // MUROSIMPROC_UTIL_ARRAY2D_HPP_
