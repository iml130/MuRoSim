// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SIMD_ARRAY_FLOAT_HPP_
#define MUROSIM_SIMD_ARRAY_FLOAT_HPP_

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <vector>
#include <limits>
#include "murosim/simd/declarations.hpp"

namespace murosim::simd
{

/// std::unique_ptr deleter for buffers obtained from std::aligned_alloc;
/// frees with std::free rather than delete[] as plain new/delete cannot
/// release SIMD-aligned allocations.
struct AlignedFloatDeleter
{
    void operator()(float* ptr) const noexcept
    {
        std::free(ptr);
    }
};

/// Fixed-size buffer of floats, SIMD-aligned and padded up to a multiple of
/// the build's SIMD lane width so murosim::simd::Vec can load/store past the
/// logical end without reading uninitialized memory. The padding lanes are
/// filled with quiet NaN, which compares false under Vec::lessEqual, so
/// padding never contributes to collision or reduction results computed over
/// the whole aligned buffer. Used to back the per-robot lidar range arrays and
/// similar bulk-float data processed with the SIMD kernels in
/// src/collision/array_kernels.cpp and murosim::simd::util.
class ArrayFloat
{
private:
    size_t size_;
    size_t size_aligned_;
    std::unique_ptr<float[], AlignedFloatDeleter> data_;

    void fillWithQuietNan();
public:
    /// Sentinel value used both to fill unused padding lanes and as the
    /// return value of reductions (min/max/sum/mean) on an empty array.
    static constexpr float QUIET_NAN = std::numeric_limits<float>::quiet_NaN();
    /// Lane width of the build-selected SIMD backend; sizeAligned() is always
    /// a multiple of this.
    static constexpr size_t SIMD_SIZE = declarations::SIMD_FLOAT_SIZE;

    /// Constructs an empty array (size 0, no allocation).
    ArrayFloat();
    /// Allocates an aligned buffer for size elements; contents (including
    /// padding) start as quiet NaN.
    explicit ArrayFloat(const size_t size);
    /// Allocates an aligned buffer and copies array's contents into it; any
    /// padding beyond array.size() is quiet NaN.
    explicit ArrayFloat(const std::vector<float> &array);
    ~ArrayFloat() = default;

    ArrayFloat(const ArrayFloat &other);
    ArrayFloat &operator=(const ArrayFloat& other);

    ArrayFloat(ArrayFloat&& other) noexcept;
    ArrayFloat& operator=(ArrayFloat&& a) noexcept;

    // The accessors are inline on purpose: the collision kernels call
    // data()/sizeAligned() per lane, and as out-of-line functions they were
    // 10-14 % of a simulation step (perf, 2026-09-18).
    /// Logical element count, i.e. the size passed at construction.
    [[nodiscard]] size_t size() const
    {
        return size_;
    }
    /// Allocated element count, rounded up to a multiple of SIMD_SIZE; always
    /// >= size().
    [[nodiscard]] size_t sizeAligned() const
    {
        return size_aligned_;
    }
    /// Element access without bounds checking beyond a debug assert(i < size()).
    float& operator[](const size_t i)
    {
        assert(i < size_);
        return data_[i];
    }
    /// @copydoc operator[](const size_t)
    [[nodiscard]] float operator[](const size_t i) const
    {
        assert(i < size_);
        return data_[i];
    }

    /// Pointer to the first element of the aligned, padded buffer.
    [[nodiscard]] const float * data() const
    {
        return data_.get();
    }
    /// @copydoc data() const
    float * data()
    {
        return data_.get();
    }

    /// Copies the logical (unpadded) elements into a new std::vector.
    [[nodiscard]] std::vector<float> toVector() const;
    /// Copies the logical elements into an existing vector, which must
    /// already hold at least size() elements.
    void copyToVector(std::vector<float> &v) const;
    /// Returns a new ArrayFloat holding elements [begin, end). If backwards
    /// is true, the elements are copied in reverse order.
    /// @param begin inclusive start index, must be < size().
    /// @param end exclusive end index, must be <= size() and >= begin.
    [[nodiscard]] ArrayFloat subArrayFloat(const size_t begin, const size_t end, const bool backwards=false) const;

    /// Minimum over the logical elements; QUIET_NAN if empty.
    [[nodiscard]] float min() const;
    /// Maximum over the logical elements; QUIET_NAN if empty.
    [[nodiscard]] float max() const;
    /// Sum over the logical elements, computed via murosim::simd::util::sum;
    /// QUIET_NAN if empty.
    [[nodiscard]] float sum() const;
    /// Arithmetic mean of the logical elements; QUIET_NAN if empty.
    [[nodiscard]] float mean() const;
    /// Sample variance (divided by size() - 1) of the logical elements;
    /// QUIET_NAN if size() <= 1.
    [[nodiscard]] float variance() const;

    /// True if size() == 0.
    [[nodiscard]] bool empty() const;
};

} // namespace murosim::simd

#endif // MUROSIM_SIMD_ARRAY_FLOAT_HPP_
