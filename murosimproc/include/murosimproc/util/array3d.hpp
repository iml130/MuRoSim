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

#ifndef MUROSIMPROC_UTIL_ARRAY3D_HPP_
#define MUROSIMPROC_UTIL_ARRAY3D_HPP_

#include <cassert>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace murosimproc::util
{

/// Dense, row-major 3D grid of `T` backed by a single `std::vector` (layout: x fastest, then y, then z).
/// Used for volumetric/layered grid data (e.g. WFC wave coefficients per cell). Element access via
/// `operator[]`/`operator()` only bounds-checks with `assert` (disabled in release builds).
template <typename T>
class Array3d {
private:
    std::vector<T> array_;
    size_t size_x_;
    size_t size_y_;
    size_t size_z_;
    size_t step_xy_;

public:
    /// Constructs an empty (0x0x0) array.
    Array3d(): array_(), size_x_(0), size_y_(0), size_z_(0), step_xy_(0)
    {

    }

    /// Constructs an `x` by `y` by `z` array with default-initialized elements.
    Array3d(const size_t x, const size_t y, const size_t z) : array_(x * y * z), size_x_(x), size_y_(y), size_z_(z), step_xy_(x*y)
    {

    }

    /// Constructs an `x` by `y` by `z` array with every element set to `value`.
    Array3d(const size_t x, const size_t y, const size_t z, const T &value) : array_(x * y * z, value), size_x_(x), size_y_(y), size_z_(z), step_xy_(x*y)
    {

    }

    /// Constructs an `x` by `y` by `z` array by copying data.
    /// @param data Element data (x fastest, then y, then z); its size must equal `x * y * z`.
    /// @throws std::invalid_argument if `data.size() != x * y * z`.
    Array3d(const size_t x, const size_t y, const size_t z, const std::vector<T> &data) : array_(x * y * z), size_x_(x), size_y_(y), size_z_(z), step_xy_(x*y)
    {
        if (data.size() != array_.size())
        {
            throw std::invalid_argument("Array3d data size must equal x * y * z!");
        }
        std::copy(std::cbegin(data), std::cend(data), std::begin(array_));
    }

    /// Accesses the element at flat `index`.
    inline T& operator[](const size_t index)
    {
        assert(index < array_.size());
        return array_[index];
    }

    /// Accesses the element at flat `index`.
    inline const T& operator[](const size_t index) const
    {
        assert(index < array_.size());
        return array_[index];
    }

    /// Accesses the element at grid coordinates `(x, y, z)`.
    inline T& operator()(const size_t x, const size_t y, const size_t z)
    {
        assert(x < size_x_);
        assert(y < size_y_);
        assert(z < size_z_);
        return array_[x + y * size_x_ + z * step_xy_];
    }

    /// Accesses the element at grid coordinates `(x, y, z)`.
    inline const T& operator()(const size_t x, const size_t y, const size_t z) const
    {
        assert(x < size_x_);
        assert(y < size_y_);
        assert(z < size_z_);
        return array_[x + y * size_x_ + z * step_xy_];
    }

    /// Compares two arrays for equal size and equal elements in storage order.
    inline bool operator== (const Array3d<T> &other) const
    {
        if (size_x_ != other.size_x_ || size_y_ != other.size_y_ || size_z_ != other.size_z_)
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

    /// Converts grid coordinates `(x, y, z)` to the flat index used by `operator[]`.
    inline size_t toIndex(const size_t x, const size_t y, const size_t z) const
    {
        assert(x < size_x_);
        assert(y < size_y_);
        assert(z < size_z_);
        return x + y * size_x_ + z * step_xy_;
    }

    /// Returns the total number of elements (`sizeX() * sizeY() * sizeZ()`).
    inline size_t size() const
    {
        return array_.size();
    }

    /// Returns the extent along x.
    inline size_t sizeX() const
    {
        return size_x_;
    }

    /// Returns the extent along y.
    inline size_t sizeY() const
    {
        return size_y_;
    }

    /// Returns the extent along z.
    inline size_t sizeZ() const
    {
        return size_z_;
    }

    /// Returns whether the array holds zero elements.
    inline bool empty() const
    {
        return array_.empty();
    }

    /// Copies out the `x_size` by `y_size` by `z_size` sub-region starting at `(x, y, z)` as a new array.
    /// @throws Undefined behaviour (assertion in debug builds) if the region exceeds the array bounds.
    inline Array3d<T> subArray3d(const size_t x, const size_t y, const size_t z, const size_t x_size, const size_t y_size, const size_t z_size) const
    {
        assert(x + x_size <= size_x_);
        assert(y + y_size <= size_y_);
        assert(z + z_size <= size_z_);

        Array3d<T> sub_array(x_size, y_size, z_size);

        for (size_t k = 0; k < z_size; k++)
        {
            for (size_t j = 0; j < y_size; j++)
            {
                size_t src_index = (x) + size_x_ * ((y + j) + size_y_ * (z + k));
                const T* src_ptr = &array_[src_index];
                T* dst_ptr = &sub_array(0, j, k);
                std::copy(src_ptr, src_ptr + x_size, dst_ptr);
            }
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

namespace std
{

/// Combines the hashes of all elements (order-dependent) so an `Array3d<T>` can be used as a hash-map/set key.
template <typename T> struct hash<murosimproc::util::Array3d<T>>
{
    std::size_t operator()(const murosimproc::util::Array3d<T>& array) const
    {
        std::size_t seed = 0;
        std::hash<T> hasher;
        for(size_t i = 0; i < array.size(); i++) {
            seed ^= hasher(array[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

}  // namespace std


#endif // MUROSIMPROC_UTIL_ARRAY3D_HPP_
