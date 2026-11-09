// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_COLOR_HPP_
#define MUROSIM_VISUALIZATION_COLOR_HPP_

#include <ostream>
#include <Eigen/Dense>

namespace murosim::visualization
{

namespace detail {
    /// Scale factor converting an 8-bit int channel (0-255, default alpha 256
    /// so a fully-opaque default normalizes to 1.0) into a [0,1] float.
    constexpr float COLOR_NORM = 1.0f / 256.0f;
}

/// RGBA color used throughout the visualization module (Canvas, Scenery,
/// shaders). Stored internally as a float Eigen::Vector4f with each channel
/// clamped to [0,1]; constructible from float or 8-bit-int RGB/RGBA in
/// several forms, all of which clip out-of-range input rather than throwing.
class Color
{
private:
    Eigen::Vector4f color_;

    static Eigen::Vector4f clip(Eigen::Vector4f v)
    {
        return v.cwiseMax(0.0f).cwiseMin(1.0f);
    }

    static Eigen::Vector4f clipInt(Eigen::Vector4i v)
    {
        return (v.cast<float>() * detail::COLOR_NORM).cwiseMax(0.0f).cwiseMin(1.0f);
    }

public:
    /// Default color: fully transparent black (0, 0, 0, 0).
    Color() : color_(Eigen::Vector4f::Zero())
    {
    }

    /// From a float RGBA vector; each component clamped to [0,1].
    Color(Eigen::Vector4f color)
        : color_(clip(color))
    {
    }

    /// From a float RGB vector with alpha fixed to 1.0 (opaque); components
    /// clamped to [0,1].
    Color(Eigen::Vector3f color)
        : color_(clip(Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)))
    {
    }

    /// From an integer RGBA vector, channels expected in [0,255]; normalized
    /// by detail::COLOR_NORM and clamped to [0,1].
    Color(Eigen::Vector4i color)
        : color_(clipInt(color))
    {
    }

    /// From an integer RGB vector, channels expected in [0,255], alpha fixed
    /// to opaque; normalized and clamped like the RGBA overload.
    Color(Eigen::Vector3i color)
        : color_(clipInt(Eigen::Vector4i(color.x(), color.y(), color.z(), 256)))
    {
    }

    /// From float RGB channels, alpha fixed to 1.0 (opaque); clamped to [0,1].
    Color(float r, float g, float b)
        : color_(clip(Eigen::Vector4f(r, g, b, 1.0f)))
    {
    }

    /// From float RGBA channels; clamped to [0,1].
    Color(float r, float g, float b, float a)
        : color_(clip(Eigen::Vector4f(r, g, b, a)))
    {
    }

    /// From integer RGB channels in [0,255], alpha fixed to opaque;
    /// normalized and clamped to [0,1].
    Color(int r, int g, int b)
        : color_(clipInt(Eigen::Vector4i(r, g, b, 256)))
    {
    }

    /// From integer RGBA channels in [0,255]; normalized and clamped to [0,1].
    Color(int r, int g, int b, int a)
        : color_(clipInt(Eigen::Vector4i(r, g, b, a)))
    {
    }

    /// Red channel in [0,1].
    float r() const { return color_.x(); }
    /// Green channel in [0,1].
    float g() const { return color_.y(); }
    /// Blue channel in [0,1].
    float b() const { return color_.z(); }
    /// Alpha channel in [0,1].
    float a() const { return color_.w(); }

    /// The color as a float RGBA Eigen vector.
    Eigen::Vector4f asVector() const { return color_; }

    /// Streams the color as "color(r=.., g=.., b=.., a=..)".
    friend std::ostream& operator<<(std::ostream& os, Color color)
    {
        return os << "color(r=" << color.r()
                  << ", g=" << color.g()
                  << ", b=" << color.b()
                  << ", a=" << color.a() << ")";
    }
};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_COLOR_HPP_
