// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/visualization/color.hpp"
#include <sstream>

namespace
{
using murosim::visualization::Color;

// Integer channels are scaled by 1/256 (not 1/255), a power of two, so every
// stored value is exact and EXPECT_FLOAT_EQ applies throughout
constexpr float COLOR_SCALE = 1.0f / 256.0f;
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(ColorConstructor, DefaultConstructorIsTransparentBlack)
{
    const Color color;

    EXPECT_FLOAT_EQ(color.r(), 0.0f);
    EXPECT_FLOAT_EQ(color.g(), 0.0f);
    EXPECT_FLOAT_EQ(color.b(), 0.0f);
    EXPECT_FLOAT_EQ(color.a(), 0.0f);
}

TEST(ColorConstructor, FloatRgbIsOpaque)
{
    const Color color(0.5f, 0.25f, 0.75f);

    EXPECT_FLOAT_EQ(color.r(), 0.5f);
    EXPECT_FLOAT_EQ(color.g(), 0.25f);
    EXPECT_FLOAT_EQ(color.b(), 0.75f);
    EXPECT_FLOAT_EQ(color.a(), 1.0f); // alpha defaults to opaque
}

TEST(ColorConstructor, FloatRgba)
{
    const Color color(0.1f, 0.2f, 0.3f, 0.4f);

    EXPECT_FLOAT_EQ(color.r(), 0.1f);
    EXPECT_FLOAT_EQ(color.g(), 0.2f);
    EXPECT_FLOAT_EQ(color.b(), 0.3f);
    EXPECT_FLOAT_EQ(color.a(), 0.4f);
}

TEST(ColorConstructor, FloatRgbBoundsAreKept)
{
    const Color black(0.0f, 0.0f, 0.0f);
    const Color white(1.0f, 1.0f, 1.0f);

    EXPECT_FLOAT_EQ(black.r(), 0.0f);
    EXPECT_FLOAT_EQ(black.g(), 0.0f);
    EXPECT_FLOAT_EQ(black.b(), 0.0f);
    EXPECT_FLOAT_EQ(black.a(), 1.0f);
    EXPECT_FLOAT_EQ(white.r(), 1.0f);
    EXPECT_FLOAT_EQ(white.g(), 1.0f);
    EXPECT_FLOAT_EQ(white.b(), 1.0f);
    EXPECT_FLOAT_EQ(white.a(), 1.0f);
}

TEST(ColorConstructor, IntRgbIsScaledAndOpaque)
{
    const Color color(128, 64, 192);

    EXPECT_FLOAT_EQ(color.r(), 128.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.g(), 64.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.b(), 192.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.a(), 1.0f);
}

TEST(ColorConstructor, IntRgba)
{
    const Color color(255, 128, 64, 200);

    EXPECT_FLOAT_EQ(color.r(), 255.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.g(), 128.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.b(), 64.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.a(), 200.0f * COLOR_SCALE);
}

TEST(ColorConstructor, IntBoundsScaleBelowOne)
{
    const Color black(0, 0, 0);
    const Color white(255, 255, 255);

    EXPECT_FLOAT_EQ(black.r(), 0.0f);
    EXPECT_FLOAT_EQ(black.g(), 0.0f);
    EXPECT_FLOAT_EQ(black.b(), 0.0f);
    EXPECT_FLOAT_EQ(white.r(), 255.0f * COLOR_SCALE); // 0.99609375, not 1
    EXPECT_FLOAT_EQ(white.g(), 255.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(white.b(), 255.0f * COLOR_SCALE);
}

TEST(ColorConstructor, Vector4f)
{
    const Color color(Eigen::Vector4f(0.5f, 0.25f, 0.75f, 0.8f));

    EXPECT_FLOAT_EQ(color.r(), 0.5f);
    EXPECT_FLOAT_EQ(color.g(), 0.25f);
    EXPECT_FLOAT_EQ(color.b(), 0.75f);
    EXPECT_FLOAT_EQ(color.a(), 0.8f);
}

TEST(ColorConstructor, Vector3fIsOpaque)
{
    const Color color(Eigen::Vector3f(0.3f, 0.6f, 0.9f));

    EXPECT_FLOAT_EQ(color.r(), 0.3f);
    EXPECT_FLOAT_EQ(color.g(), 0.6f);
    EXPECT_FLOAT_EQ(color.b(), 0.9f);
    EXPECT_FLOAT_EQ(color.a(), 1.0f);
}

TEST(ColorConstructor, Vector4iIsScaled)
{
    const Color color(Eigen::Vector4i(128, 64, 192, 200));

    EXPECT_FLOAT_EQ(color.r(), 128.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.g(), 64.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.b(), 192.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.a(), 200.0f * COLOR_SCALE);
}

TEST(ColorConstructor, Vector3iIsScaledAndOpaque)
{
    const Color color(Eigen::Vector3i(255, 128, 64));

    EXPECT_FLOAT_EQ(color.r(), 255.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.g(), 128.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.b(), 64.0f * COLOR_SCALE);
    EXPECT_FLOAT_EQ(color.a(), 1.0f); // alpha 256 * (1/256)
}

// ============================================================================
// Clipping Tests
// ============================================================================

TEST(ColorClipping, FloatNegativeChannelsClipToZero)
{
    const Color color(-0.5f, -1.0f, -100.0f);

    EXPECT_FLOAT_EQ(color.r(), 0.0f);
    EXPECT_FLOAT_EQ(color.g(), 0.0f);
    EXPECT_FLOAT_EQ(color.b(), 0.0f);
}

TEST(ColorClipping, FloatOverflowChannelsClipToOne)
{
    const Color color(1.5f, 2.0f, 100.0f);

    EXPECT_FLOAT_EQ(color.r(), 1.0f);
    EXPECT_FLOAT_EQ(color.g(), 1.0f);
    EXPECT_FLOAT_EQ(color.b(), 1.0f);
}

TEST(ColorClipping, FloatAlphaClipsToUnitRange)
{
    const Color too_high(0.5f, 0.5f, 0.5f, 5.0f);
    const Color too_low(0.5f, 0.5f, 0.5f, -1.0f);

    EXPECT_FLOAT_EQ(too_high.a(), 1.0f);
    EXPECT_FLOAT_EQ(too_low.a(), 0.0f);
}

TEST(ColorClipping, IntNegativeChannelsClipToZero)
{
    const Color color(-10, -50, -100);

    EXPECT_FLOAT_EQ(color.r(), 0.0f);
    EXPECT_FLOAT_EQ(color.g(), 0.0f);
    EXPECT_FLOAT_EQ(color.b(), 0.0f);
}

TEST(ColorClipping, IntOverflowChannelsClipToOne)
{
    const Color color(300, 500, 1000);

    EXPECT_FLOAT_EQ(color.r(), 1.0f);
    EXPECT_FLOAT_EQ(color.g(), 1.0f);
    EXPECT_FLOAT_EQ(color.b(), 1.0f);
}

TEST(ColorClipping, IntAlphaClipsToOne)
{
    const Color color(128, 128, 128, 500);

    EXPECT_FLOAT_EQ(color.a(), 1.0f);
}

TEST(ColorClipping, Vector4fClipsEachChannel)
{
    const Color color(Eigen::Vector4f(1.5f, -0.5f, 0.5f, 2.0f));

    EXPECT_FLOAT_EQ(color.r(), 1.0f); // from 1.5
    EXPECT_FLOAT_EQ(color.g(), 0.0f); // from -0.5
    EXPECT_FLOAT_EQ(color.b(), 0.5f);
    EXPECT_FLOAT_EQ(color.a(), 1.0f); // from 2.0
}

TEST(ColorClipping, Vector3iClipsEachChannel)
{
    const Color color(Eigen::Vector3i(300, -50, 128));

    EXPECT_FLOAT_EQ(color.r(), 1.0f); // from 300
    EXPECT_FLOAT_EQ(color.g(), 0.0f); // from -50
    EXPECT_FLOAT_EQ(color.b(), 128.0f * COLOR_SCALE);
}

// ============================================================================
// Getter Tests
// ============================================================================

TEST(ColorGetter, ChannelGettersReadTheirOwnChannel)
{
    const Color color(0.8f, 0.6f, 0.4f, 0.7f);

    EXPECT_FLOAT_EQ(color.r(), 0.8f);
    EXPECT_FLOAT_EQ(color.g(), 0.6f);
    EXPECT_FLOAT_EQ(color.b(), 0.4f);
    EXPECT_FLOAT_EQ(color.a(), 0.7f);
}

TEST(ColorGetter, AsVectorIsRgba)
{
    const Color color(0.5f, 0.25f, 0.75f, 0.8f);

    const Eigen::Vector4f vector = color.asVector();

    EXPECT_FLOAT_EQ(vector.x(), 0.5f);
    EXPECT_FLOAT_EQ(vector.y(), 0.25f);
    EXPECT_FLOAT_EQ(vector.z(), 0.75f);
    EXPECT_FLOAT_EQ(vector.w(), 0.8f);
}

TEST(ColorGetter, AsVectorMatchesGetters)
{
    const Color color(0.3f, 0.6f, 0.9f, 0.4f);

    const Eigen::Vector4f vector = color.asVector();

    EXPECT_FLOAT_EQ(vector.x(), color.r());
    EXPECT_FLOAT_EQ(vector.y(), color.g());
    EXPECT_FLOAT_EQ(vector.z(), color.b());
    EXPECT_FLOAT_EQ(vector.w(), color.a());
}

// ============================================================================
// Copy Tests
// ============================================================================

TEST(ColorCopy, CopyConstructor)
{
    const Color original(0.3f, 0.6f, 0.9f, 0.5f);

    const Color copy(original);

    EXPECT_FLOAT_EQ(copy.r(), 0.3f);
    EXPECT_FLOAT_EQ(copy.g(), 0.6f);
    EXPECT_FLOAT_EQ(copy.b(), 0.9f);
    EXPECT_FLOAT_EQ(copy.a(), 0.5f);
}

TEST(ColorCopy, CopyAssignment)
{
    const Color original(0.3f, 0.6f, 0.9f, 0.5f);
    Color assigned;

    assigned = original;

    EXPECT_FLOAT_EQ(assigned.r(), 0.3f);
    EXPECT_FLOAT_EQ(assigned.g(), 0.6f);
    EXPECT_FLOAT_EQ(assigned.b(), 0.9f);
    EXPECT_FLOAT_EQ(assigned.a(), 0.5f);
}

// ============================================================================
// Stream Operator Tests
// ============================================================================

TEST(ColorStream, PrintsAllChannels)
{
    const Color color(0.5f, 0.25f, 0.75f, 1.0f);

    std::ostringstream stream;
    stream << color;

    EXPECT_EQ(stream.str(), "color(r=0.5, g=0.25, b=0.75, a=1)");
}
