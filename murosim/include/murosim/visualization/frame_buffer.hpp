// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_FRAME_BUFFER_HPP_
#define MUROSIM_VISUALIZATION_FRAME_BUFFER_HPP_

#include <cstdint>
#include <vector>

namespace murosim::visualization
{

/// Owns a tightly-packed 8-bit RGB image (3 bytes per pixel, row-major, top
/// row first as on screen, no padding) read back from a Renderer via glReadPixels. Renderer::getFrameBuffer()
/// converts whatever format/type the GL implementation reports (RGB, RGBA,
/// BGR, BGRA) into this fixed RGB layout.
class FrameBuffer
{
private:
    std::vector<uint8_t> data_;
    size_t width_;
    size_t height_;
public:
    /// Constructs an empty buffer (0x0, no allocation).
    FrameBuffer(): width_(0), height_(0)
    {

    }

    /// Allocates a width*height*3-byte RGB buffer; contents are
    /// uninitialized until filled (e.g. by glReadPixels).
    FrameBuffer(const size_t width, const size_t height) : width_(width), height_(height)
    {
        data_.resize(width_ * height_ * 3);
    }

    /// Pointer to the first byte of the row-major RGB pixel data.
    uint8_t* data()
    {
        return data_.data();
    }

    /// @copydoc data()
    const uint8_t* data() const
    {
        return data_.data();
    }

    /// Image width in pixels.
    size_t width() const
    {
        return width_;
    }

    /// Image height in pixels.
    size_t height() const
    {
        return height_;
    }

    /// True if the buffer holds no pixel data (default-constructed).
    bool empty() const
    {
        return data_.empty();
    }

    /// Total byte size of the pixel data (width * height * 3).
    size_t size() const
    {
        return data_.size();
    }
};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_FRAME_BUFFER_HPP_
