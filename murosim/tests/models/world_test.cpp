// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/models/world.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::geometry::Line;
using murosim::io::DataContainer;
using murosim::navigation::Node;

constexpr float WORLD_SIZE = 10.0f;
constexpr float SPACE = 1.0f;
constexpr float HALF_SPACE = SPACE / 2.0f;
constexpr size_t BORDER_LINES = 4;

// Rotated primitives put wall end points a few nanometres outside the square
constexpr float EDGE_MARGIN = 1e-6f;

/// Asserts that every node lies inside the world square with its whole
/// circle, has radius HALF_SPACE and belongs to group 0.
void assertNodesFitTheWorld(const DataContainer &data)
{
    for (size_t i = 0; i < data.getNodes().size(); i++)
    {
        const Node &node = data.getNodes()[i];
        EXPECT_GE(node.getX() - node.getRadius(), 0.0f) << "node " << i;
        EXPECT_LE(node.getX() + node.getRadius(), WORLD_SIZE) << "node " << i;
        EXPECT_GE(node.getY() - node.getRadius(), 0.0f) << "node " << i;
        EXPECT_LE(node.getY() + node.getRadius(), WORLD_SIZE) << "node " << i;
        EXPECT_FLOAT_EQ(node.getRadius(), HALF_SPACE) << "node " << i;
        EXPECT_EQ(node.getGroup(), static_cast<size_t>(0)) << "node " << i;
    }
}

/// Asserts that every line end point lies inside the world square.
void assertLinesFitTheWorld(const DataContainer &data)
{
    for (size_t i = 0; i < data.getLines().size(); i++)
    {
        const Line &line = data.getLines()[i];
        EXPECT_GE(std::min(line.getPoint1X(), line.getPoint2X()), -EDGE_MARGIN) << "line " << i;
        EXPECT_LE(std::max(line.getPoint1X(), line.getPoint2X()), WORLD_SIZE + EDGE_MARGIN) << "line " << i;
        EXPECT_GE(std::min(line.getPoint1Y(), line.getPoint2Y()), -EDGE_MARGIN) << "line " << i;
        EXPECT_LE(std::max(line.getPoint1Y(), line.getPoint2Y()), WORLD_SIZE + EDGE_MARGIN) << "line " << i;
    }
}
} // namespace

// ============================================================================
// Argument Validation Tests
// ============================================================================

// All generators share the same argument check; empty() stands in for them.

TEST(ModelsWorldArguments, RejectsNonPositiveWorldSize)
{
    EXPECT_THROW(murosim::models::world::empty(0.0f, SPACE), std::invalid_argument);
    EXPECT_THROW(murosim::models::world::empty(-1.0f, SPACE), std::invalid_argument);
}

TEST(ModelsWorldArguments, RejectsNonPositiveSpace)
{
    EXPECT_THROW(murosim::models::world::empty(WORLD_SIZE, 0.0f), std::invalid_argument);
    EXPECT_THROW(murosim::models::world::empty(WORLD_SIZE, -1.0f), std::invalid_argument);
}

TEST(ModelsWorldArguments, RejectsSpaceLargerThanWorld)
{
    EXPECT_THROW(murosim::models::world::empty(1.0f, 2.0f), std::invalid_argument);
}

// ============================================================================
// Line World Tests
// ============================================================================

// Regression: the validity check used to be inverted (it threw for normal
// large worlds and accepted degenerate small ones).
TEST(ModelsWorldLine, AcceptsLargeWorld)
{
    EXPECT_NO_THROW(murosim::models::world::line(WORLD_SIZE, SPACE));
}

TEST(ModelsWorldLine, RejectsWorldSmallerThanTwiceTheSpace)
{
    EXPECT_THROW(murosim::models::world::line(1.5f, SPACE), std::invalid_argument);
    EXPECT_NO_THROW(murosim::models::world::line(2.0f, SPACE));
}

TEST(ModelsWorldLine, HasTwoNodesAndSixLines)
{
    const DataContainer data = murosim::models::world::line(WORLD_SIZE, SPACE);

    EXPECT_EQ(data.getName(), "line");
    EXPECT_EQ(data.getNodes().size(), static_cast<size_t>(2));
    EXPECT_EQ(data.getLines().size(), BORDER_LINES + 2); // two corridor walls
    EXPECT_EQ(data.getCircles().size(), static_cast<size_t>(0));
    assertNodesFitTheWorld(data);
    assertLinesFitTheWorld(data);
}

// Regression: the lower corridor wall used to be computed from
// half_world_size * 0.5 instead of half_world_size, leaving the corridor
// off-center relative to the node row.
TEST(ModelsWorldLine, CorridorIsCenteredOnNodeRow)
{
    const DataContainer data = murosim::models::world::line(WORLD_SIZE, SPACE);

    // both nodes sit on the corridor center line
    ASSERT_EQ(data.getNodes().size(), static_cast<size_t>(2));
    const float node_y = data.getNodes()[0].getPoint().y();
    EXPECT_FLOAT_EQ(node_y, WORLD_SIZE * 0.5f);
    EXPECT_FLOAT_EQ(data.getNodes()[1].getPoint().y(), node_y);

    // collect the horizontal corridor walls (interior lines, not the borders)
    std::vector<float> wall_y;
    for (const Line &line : data.getLines())
    {
        const bool horizontal = line.getPoint1Y() == line.getPoint2Y();
        const float y = line.getPoint1Y();
        if (horizontal && y > 0.0f && y < WORLD_SIZE)
        {
            wall_y.push_back(y);
        }
    }
    std::sort(wall_y.begin(), wall_y.end());

    ASSERT_EQ(wall_y.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(wall_y[0], node_y - HALF_SPACE);
    EXPECT_FLOAT_EQ(wall_y[1], node_y + HALF_SPACE);
}

TEST(ModelsWorldLine, NodesSitAtBothCorridorEnds)
{
    const DataContainer data = murosim::models::world::line(WORLD_SIZE, SPACE);

    ASSERT_EQ(data.getNodes().size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(data.getNodes()[0].getX(), HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[1].getX(), WORLD_SIZE - HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[0].getStartAngle(), 0.0f);    // faces the far end
    EXPECT_FLOAT_EQ(data.getNodes()[1].getStartAngle(), PI);
}

// ============================================================================
// Empty World Tests
// ============================================================================

TEST(ModelsWorldEmpty, RejectsWorldSmallerThanThreeTimesTheSpace)
{
    EXPECT_THROW(murosim::models::world::empty(2.5f, SPACE), std::invalid_argument);
    EXPECT_NO_THROW(murosim::models::world::empty(3.0f, SPACE));
}

TEST(ModelsWorldEmpty, HasOnlyTheBorderAndNineNodes)
{
    const DataContainer data = murosim::models::world::empty(WORLD_SIZE, SPACE);

    EXPECT_EQ(data.getName(), "empty");
    EXPECT_EQ(data.getLines().size(), BORDER_LINES);
    EXPECT_EQ(data.getCircles().size(), static_cast<size_t>(0));
    EXPECT_EQ(data.getNodes().size(), static_cast<size_t>(9)); // centre plus eight around it
    assertNodesFitTheWorld(data);
    assertLinesFitTheWorld(data);
}

TEST(ModelsWorldEmpty, FirstNodeIsTheCentre)
{
    const DataContainer data = murosim::models::world::empty(WORLD_SIZE, SPACE);

    ASSERT_FALSE(data.getNodes().empty());
    EXPECT_FLOAT_EQ(data.getNodes()[0].getX(), WORLD_SIZE / 2.0f);
    EXPECT_FLOAT_EQ(data.getNodes()[0].getY(), WORLD_SIZE / 2.0f);
}

// ============================================================================
// Intersection World Tests
// ============================================================================

TEST(ModelsWorldIntersection, RejectsWorldSmallerThanThreeTimesTheSpace)
{
    EXPECT_THROW(murosim::models::world::intersection(2.5f, SPACE), std::invalid_argument);
    EXPECT_NO_THROW(murosim::models::world::intersection(3.0f, SPACE));
}

TEST(ModelsWorldIntersection, HasFourNodesAtTheCorridorEnds)
{
    const DataContainer data = murosim::models::world::intersection(WORLD_SIZE, SPACE);

    EXPECT_EQ(data.getName(), "intersection");
    EXPECT_GT(data.getLines().size(), BORDER_LINES); // the corridor walls
    ASSERT_EQ(data.getNodes().size(), static_cast<size_t>(4));
    assertNodesFitTheWorld(data);
    assertLinesFitTheWorld(data);

    // North, east, south, west: each half a space in from its border
    const float centre = WORLD_SIZE / 2.0f;
    EXPECT_FLOAT_EQ(data.getNodes()[0].getX(), centre);
    EXPECT_FLOAT_EQ(data.getNodes()[0].getY(), WORLD_SIZE - HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[1].getX(), WORLD_SIZE - HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[1].getY(), centre);
    EXPECT_FLOAT_EQ(data.getNodes()[2].getX(), centre);
    EXPECT_FLOAT_EQ(data.getNodes()[2].getY(), HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[3].getX(), HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[3].getY(), centre);
}

// ============================================================================
// Roundabout World Tests
// ============================================================================

TEST(ModelsWorldRoundabout, RejectsWorldSmallerThanFiveTimesTheSpace)
{
    EXPECT_THROW(murosim::models::world::roundabout(4.5f, SPACE), std::invalid_argument);
    EXPECT_NO_THROW(murosim::models::world::roundabout(5.0f, SPACE));
}

TEST(ModelsWorldRoundabout, HasFourNodesAtTheCorridorEnds)
{
    const DataContainer data = murosim::models::world::roundabout(WORLD_SIZE, SPACE);

    EXPECT_EQ(data.getName(), "roundabout");
    EXPECT_GT(data.getLines().size(), BORDER_LINES);
    ASSERT_EQ(data.getNodes().size(), static_cast<size_t>(4));
    assertNodesFitTheWorld(data);
    assertLinesFitTheWorld(data);

    const float centre = WORLD_SIZE / 2.0f;
    EXPECT_FLOAT_EQ(data.getNodes()[0].getY(), WORLD_SIZE - HALF_SPACE); // north
    EXPECT_FLOAT_EQ(data.getNodes()[1].getX(), WORLD_SIZE - HALF_SPACE); // east
    EXPECT_FLOAT_EQ(data.getNodes()[2].getY(), HALF_SPACE);              // south
    EXPECT_FLOAT_EQ(data.getNodes()[3].getX(), HALF_SPACE);              // west
    EXPECT_FLOAT_EQ(data.getNodes()[0].getX(), centre);
    EXPECT_FLOAT_EQ(data.getNodes()[3].getY(), centre);
}

// ============================================================================
// Curve World Tests
// ============================================================================

TEST(ModelsWorldCurve, RejectsWorldSmallerThanFiveTimesTheSpace)
{
    EXPECT_THROW(murosim::models::world::curve(4.5f, SPACE), std::invalid_argument);
    EXPECT_NO_THROW(murosim::models::world::curve(5.0f, SPACE));
}

TEST(ModelsWorldCurve, HasTwoCornerCirclesAndNoNodes)
{
    const DataContainer data = murosim::models::world::curve(WORLD_SIZE, SPACE);

    EXPECT_EQ(data.getName(), "curve");
    EXPECT_EQ(data.getLines().size(), BORDER_LINES + 8); // two bent corridor walls of four segments
    EXPECT_EQ(data.getNodes().size(), static_cast<size_t>(0)); // no nodes: place robots by hand
    ASSERT_EQ(data.getCircles().size(), static_cast<size_t>(2));
    assertLinesFitTheWorld(data);

    // The corner circles sit one space off the centre and have half a space radius
    const float centre = WORLD_SIZE / 2.0f;
    EXPECT_FLOAT_EQ(data.getCircles()[0].getX(), centre - SPACE);
    EXPECT_FLOAT_EQ(data.getCircles()[0].getY(), centre + SPACE);
    EXPECT_FLOAT_EQ(data.getCircles()[1].getX(), centre + SPACE);
    EXPECT_FLOAT_EQ(data.getCircles()[1].getY(), centre - SPACE);
    EXPECT_FLOAT_EQ(data.getCircles()[0].getRadius(), HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getCircles()[1].getRadius(), HALF_SPACE);
}

// ============================================================================
// Bone World Tests
// ============================================================================

TEST(ModelsWorldBone, RejectsWorldSmallerThanThreeTimesTheSpace)
{
    EXPECT_THROW(murosim::models::world::bone(2.5f, SPACE), std::invalid_argument);
    EXPECT_NO_THROW(murosim::models::world::bone(3.0f, SPACE));
}

TEST(ModelsWorldBone, HasFourCornerNodes)
{
    const DataContainer data = murosim::models::world::bone(WORLD_SIZE, SPACE);

    EXPECT_EQ(data.getName(), "bone");
    EXPECT_EQ(data.getLines().size(), BORDER_LINES + 6); // two U-shaped walls of three segments
    EXPECT_EQ(data.getCircles().size(), static_cast<size_t>(0));
    ASSERT_EQ(data.getNodes().size(), static_cast<size_t>(4));
    assertNodesFitTheWorld(data);
    assertLinesFitTheWorld(data);

    // One node in each corner, half a space in from the borders
    const float far = WORLD_SIZE - HALF_SPACE;
    EXPECT_FLOAT_EQ(data.getNodes()[0].getX(), HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[0].getY(), HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[1].getX(), HALF_SPACE);
    EXPECT_FLOAT_EQ(data.getNodes()[1].getY(), far);
    EXPECT_FLOAT_EQ(data.getNodes()[2].getX(), far);
    EXPECT_FLOAT_EQ(data.getNodes()[2].getY(), far);
    EXPECT_FLOAT_EQ(data.getNodes()[3].getX(), far);
    EXPECT_FLOAT_EQ(data.getNodes()[3].getY(), HALF_SPACE);
}

TEST(ModelsWorldBone, CorridorWallsLeaveTheMiddleOpen)
{
    const DataContainer data = murosim::models::world::bone(WORLD_SIZE, SPACE);

    // The two horizontal wall segments bound the corridor half a space
    // above and below the centre line
    std::vector<float> wall_y;
    for (const Line &line : data.getLines())
    {
        const bool horizontal = line.getPoint1Y() == line.getPoint2Y();
        const float y = line.getPoint1Y();
        if (horizontal && y > 0.0f && y < WORLD_SIZE)
        {
            wall_y.push_back(y);
        }
    }
    std::sort(wall_y.begin(), wall_y.end());

    ASSERT_EQ(wall_y.size(), static_cast<size_t>(2));
    EXPECT_FLOAT_EQ(wall_y[0], WORLD_SIZE / 2.0f - HALF_SPACE);
    EXPECT_FLOAT_EQ(wall_y[1], WORLD_SIZE / 2.0f + HALF_SPACE);
}
