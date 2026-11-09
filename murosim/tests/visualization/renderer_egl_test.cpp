// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/visualization/renderer_egl.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/geometry/world.hpp"
#include "murosim/models/world.hpp"
#include "murosim/navigation/container.hpp"
#include "murosim/robot/robot_builder.hpp"
#include "murosim/simulation/simulation.hpp"
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using murosim::geometry::Shape;
using murosim::geometry::World;
using murosim::simulation::Simulation;
using murosim::visualization::FrameBuffer;
using murosim::visualization::Renderer;
using murosim::visualization::RendererEgl;

constexpr int RESOLUTION = 128;
constexpr int HIGH_RESOLUTION = 512; // thin lines need it to survive without anti-aliasing
constexpr int NO_SAMPLES = 0;
constexpr float WORLD_HALF_SIZE = 5.0f;
constexpr float GRID_SCALE = 0.1f;
constexpr float BUFFER_SCALE = 0.2f;

// A thin anti-aliased line in a primary colour blends towards white, so a
// pixel counts as red or blue when that channel leads the other two by a
// margin; a black line only greys all three channels equally.
constexpr int TINT = 20;

/// Square world of 10 m around the origin.
World squareWorld()
{
    return World(murosim::geometry::primitives::square(2.0f * WORLD_HALF_SIZE));
}

/// The square world with a full-width wall along y = 0.
World worldWithMiddleWall()
{
    Shape shape = murosim::geometry::primitives::square(2.0f * WORLD_HALF_SIZE);
    shape.addLine(-WORLD_HALF_SIZE, 0.0f, WORLD_HALF_SIZE, 0.0f);
    return World(shape);
}

const uint8_t *pixel(const FrameBuffer &frame, const size_t x, const size_t y)
{
    return frame.data() + 3 * (y * frame.width() + x);
}

bool isWhite(const uint8_t *p)
{
    return p[0] == 255 && p[1] == 255 && p[2] == 255;
}

bool isRed(const uint8_t *p)
{
    return p[0] > p[1] + TINT && p[0] > p[2] + TINT;
}

bool isBlue(const uint8_t *p)
{
    return p[2] > p[0] + TINT && p[2] > p[1] + TINT;
}

size_t countNonWhite(const FrameBuffer &frame)
{
    size_t count = 0;
    for (size_t i = 0; i < frame.width() * frame.height(); i++)
    {
        count += !isWhite(frame.data() + 3 * i);
    }
    return count;
}

/// Number of columns of `row` that hold a non-white pixel.
size_t drawnColumns(const FrameBuffer &frame, const size_t row)
{
    size_t count = 0;
    for (size_t x = 0; x < frame.width(); x++)
    {
        count += !isWhite(pixel(frame, x, row));
    }
    return count;
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

// Argument checks happen before any EGL call, so they need no driver.

TEST(RendererEglConstructor, NonPositiveResolutionThrows)
{
    EXPECT_THROW(RendererEgl(0), std::invalid_argument);
    EXPECT_THROW(RendererEgl(-1), std::invalid_argument);
}

TEST(RendererEglConstructor, NegativeSampleCountThrows)
{
    EXPECT_THROW(RendererEgl(RESOLUTION, -1), std::invalid_argument);
}

// ============================================================================
// Test Fixture
// ============================================================================

// Renders offscreen through EGL: a GPU driver or Mesa's CPU renderer
// llvmpipe, whichever the surfaceless platform provides. Without any usable
// driver the tests are skipped, not failed.
class RendererEglTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        try
        {
            renderer = std::make_unique<RendererEgl>(RESOLUTION);
        }
        catch (const std::runtime_error &error)
        {
            GTEST_SKIP() << "no OpenGL ES context available: " << error.what();
        }
    }

    /// Simulation of the square world with one robot at `x` on the x axis.
    std::shared_ptr<Simulation> simulationWithRobotAt(const float x) const
    {
        const std::vector<murosim::navigation::Node> nodes = {murosim::navigation::Node(0.0f, 0.0f, 0.5f)};
        const std::shared_ptr<Simulation> sim = std::make_shared<Simulation>(
            squareWorld(), murosim::navigation::Container(nodes), "render_test", GRID_SCALE, BUFFER_SCALE,
            Simulation::INIT_MODE::MANUAL);
        murosim::robot::RobotBuilder builder;
        builder.setActuatorDifferentialSimple().setShapeRobotCircle(1.0f);
        const std::shared_ptr<murosim::robot::Robot> robot = builder.build();
        robot->setPose(x, 0.0f, 0.0f);
        sim->addRobot(robot);
        return sim;
    }

    std::unique_ptr<RendererEgl> renderer;
};

// ============================================================================
// Frame Buffer Tests
// ============================================================================

TEST_F(RendererEglTest, FrameBufferHasTheRequestedSize)
{
    renderer->draw(squareWorld());

    const FrameBuffer frame = renderer->getFrameBuffer();

    EXPECT_EQ(frame.width(), static_cast<size_t>(RESOLUTION));
    EXPECT_EQ(frame.height(), static_cast<size_t>(RESOLUTION));
    EXPECT_EQ(frame.size(), static_cast<size_t>(3 * RESOLUTION * RESOLUTION));
    EXPECT_FALSE(frame.empty());
}

TEST_F(RendererEglTest, SampleCountIsClampedToTheDriver)
{
    EXPECT_GE(renderer->samples(), 1);
    EXPECT_LE(renderer->samples(), Renderer::MSAA_FACTOR);
    EXPECT_EQ(RendererEgl(RESOLUTION, NO_SAMPLES).samples(), NO_SAMPLES);
    EXPECT_LE(RendererEgl(RESOLUTION, 2).samples(), 2);
}

TEST_F(RendererEglTest, RenderingIsDeterministic)
{
    renderer->draw(worldWithMiddleWall());
    const FrameBuffer first = renderer->getFrameBuffer();

    renderer->draw(worldWithMiddleWall());
    const FrameBuffer second = renderer->getFrameBuffer();

    ASSERT_EQ(second.size(), first.size());
    EXPECT_EQ(std::memcmp(first.data(), second.data(), first.size()), 0);
}

// ============================================================================
// Geometry Tests
// ============================================================================

TEST_F(RendererEglTest, InteriorOfAnEmptyWorldIsWhite)
{
    renderer->draw(squareWorld());

    const FrameBuffer frame = renderer->getFrameBuffer();

    // The border walls sit at the frame edges; the central half is empty
    for (size_t y = RESOLUTION / 4; y < 3 * RESOLUTION / 4; y++)
    {
        for (size_t x = RESOLUTION / 4; x < 3 * RESOLUTION / 4; x++)
        {
            ASSERT_TRUE(isWhite(pixel(frame, x, y))) << "pixel " << x << ", " << y;
        }
    }
    EXPECT_GT(countNonWhite(frame), static_cast<size_t>(0)); // the border itself
}

TEST_F(RendererEglTest, WallIsDrawnOnItsRow)
{
    renderer->draw(worldWithMiddleWall());

    const FrameBuffer frame = renderer->getFrameBuffer();

    // y = 0 maps to the middle of the frame; the wall spans the full width
    // and shows up on the one or two rows around the centre
    const size_t middle = RESOLUTION / 2;
    size_t drawn = 0;
    for (size_t row = middle - 2; row <= middle + 1; row++)
    {
        drawn = std::max(drawn, drawnColumns(frame, row));
    }
    EXPECT_GE(drawn, static_cast<size_t>(0.9 * RESOLUTION));
    EXPECT_EQ(drawnColumns(frame, RESOLUTION / 4), static_cast<size_t>(2)); // only the border walls
    EXPECT_EQ(drawnColumns(frame, 3 * RESOLUTION / 4), static_cast<size_t>(2));
}

TEST_F(RendererEglTest, NodesAreDrawnInRed)
{
    renderer->draw(murosim::models::world::intersection(10.0f, 1.0f));

    const FrameBuffer frame = renderer->getFrameBuffer();

    size_t red = 0;
    for (size_t i = 0; i < frame.width() * frame.height(); i++)
    {
        red += isRed(frame.data() + 3 * i);
    }
    EXPECT_GT(red, static_cast<size_t>(0));
}

TEST_F(RendererEglTest, RobotIsDrawnInItsColourAtItsPosition)
{
    const std::shared_ptr<Simulation> sim = simulationWithRobotAt(2.5f); // right half of the world
    renderer->setDrawGridMap(false);

    renderer->draw(sim);

    const FrameBuffer frame = renderer->getFrameBuffer();
    size_t blue_right = 0;
    size_t blue_left = 0;
    for (size_t y = 0; y < frame.height(); y++)
    {
        for (size_t x = 0; x < frame.width(); x++)
        {
            const bool blue = isBlue(pixel(frame, x, y));
            blue_right += blue && x > frame.width() / 2;
            blue_left += blue && x < frame.width() / 2;
        }
    }
    EXPECT_GT(blue_right, static_cast<size_t>(0)); // the default robot colour is blue
    EXPECT_EQ(blue_left, static_cast<size_t>(0));
}

TEST_F(RendererEglTest, DrawGridMapToggleChangesTheFrame)
{
    const std::shared_ptr<Simulation> sim = simulationWithRobotAt(0.0f);

    renderer->setDrawGridMap(false);
    renderer->draw(sim);
    const FrameBuffer without = renderer->getFrameBuffer();
    renderer->setDrawGridMap(true);
    renderer->draw(sim);
    const FrameBuffer with = renderer->getFrameBuffer();

    EXPECT_GT(countNonWhite(with), countNonWhite(without));
}

// ============================================================================
// Multisampling Tests
// ============================================================================

TEST_F(RendererEglTest, WithoutMultisamplingLinesAreNotSmoothed)
{
    RendererEgl aliased(HIGH_RESOLUTION, NO_SAMPLES);
    RendererEgl smoothed(HIGH_RESOLUTION);
    aliased.draw(worldWithMiddleWall());
    smoothed.draw(worldWithMiddleWall());

    const FrameBuffer hard = aliased.getFrameBuffer();
    const FrameBuffer smooth = smoothed.getFrameBuffer();

    // An aliased line only touches the pixels it covers; anti-aliasing adds
    // the partially covered ones along its edges
    EXPECT_GT(countNonWhite(hard), static_cast<size_t>(0));
    if (smoothed.samples() > 0)
    {
        EXPECT_LT(countNonWhite(hard), countNonWhite(smooth));
    }
}

TEST_F(RendererEglTest, TwoRenderersCoexist)
{
    RendererEgl other(RESOLUTION / 2);

    renderer->draw(worldWithMiddleWall());
    other.draw(squareWorld());

    const FrameBuffer frame = renderer->getFrameBuffer();
    const FrameBuffer other_frame = other.getFrameBuffer();
    EXPECT_EQ(other_frame.width(), static_cast<size_t>(RESOLUTION / 2));
    EXPECT_GE(drawnColumns(frame, RESOLUTION / 2), static_cast<size_t>(0.9 * RESOLUTION));
}
