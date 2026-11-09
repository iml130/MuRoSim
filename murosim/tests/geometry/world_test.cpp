// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/geometry/world.hpp"

namespace
{
using murosim::geometry::Shape;
using murosim::geometry::World;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class WorldTest : public ::testing::Test
{
protected:
    // A 5 x 5 square of four lines plus two circles, one of them reaching
    // below the square so the bounding box differs from the square
    World world{{{0.0f, 0.0f, 5.0f, 0.0f},
                 {5.0f, 0.0f, 5.0f, 5.0f},
                 {5.0f, 5.0f, 0.0f, 5.0f},
                 {0.0f, 5.0f, 0.0f, 0.0f}},
                {{2.5f, 2.5f, 1.0f},
                 {1.0f, -0.5f, 0.5f}}};
};

// ============================================================================
// Line Getter Tests
// ============================================================================

TEST_F(WorldTest, GetLine)
{
    ASSERT_EQ(world.getLineSize(), static_cast<size_t>(4));

    EXPECT_FLOAT_EQ(world.getLineAt(0).getPoint1X(), 0.0f);
    EXPECT_FLOAT_EQ(world.getLineAt(0).getPoint1Y(), 0.0f);
    EXPECT_FLOAT_EQ(world.getLineAt(0).getPoint2X(), 5.0f);
    EXPECT_FLOAT_EQ(world.getLineAt(0).getPoint2Y(), 0.0f);

    EXPECT_FLOAT_EQ(world.getLinePoint1(1).x(), 5.0f);
    EXPECT_FLOAT_EQ(world.getLinePoint1(1).y(), 0.0f);
    EXPECT_FLOAT_EQ(world.getLinePoint2(1).x(), 5.0f);
    EXPECT_FLOAT_EQ(world.getLinePoint2(1).y(), 5.0f);

    EXPECT_FLOAT_EQ(world.getLinePoint1X(2), 5.0f);
    EXPECT_FLOAT_EQ(world.getLinePoint1Y(2), 5.0f);
    EXPECT_FLOAT_EQ(world.getLinePoint2X(2), 0.0f);
    EXPECT_FLOAT_EQ(world.getLinePoint2Y(2), 5.0f);
}

TEST_F(WorldTest, GetArrayLine)
{
    ASSERT_EQ(world.getArrayLinePoint1X().size(), static_cast<size_t>(4));
    ASSERT_EQ(world.getArrayLinePoint1Y().size(), static_cast<size_t>(4));
    ASSERT_EQ(world.getArrayLinePoint2X().size(), static_cast<size_t>(4));
    ASSERT_EQ(world.getArrayLinePoint2Y().size(), static_cast<size_t>(4));

    EXPECT_FLOAT_EQ(world.getArrayLinePoint1X()[0], 0.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint1Y()[0], 0.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2X()[0], 5.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2Y()[0], 0.0f);

    EXPECT_FLOAT_EQ(world.getArrayLinePoint1X()[1], 5.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint1Y()[1], 0.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2X()[1], 5.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2Y()[1], 5.0f);

    EXPECT_FLOAT_EQ(world.getArrayLinePoint1X()[2], 5.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint1Y()[2], 5.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2X()[2], 0.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2Y()[2], 5.0f);

    EXPECT_FLOAT_EQ(world.getArrayLinePoint1X()[3], 0.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint1Y()[3], 5.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2X()[3], 0.0f);
    EXPECT_FLOAT_EQ(world.getArrayLinePoint2Y()[3], 0.0f);
}

// ============================================================================
// Circle Getter Tests
// ============================================================================

TEST_F(WorldTest, GetCircle)
{
    ASSERT_EQ(world.getCircleSize(), static_cast<size_t>(2));

    EXPECT_FLOAT_EQ(world.getCircleAt(0).getX(), 2.5f);
    EXPECT_FLOAT_EQ(world.getCircleAt(0).getY(), 2.5f);
    EXPECT_FLOAT_EQ(world.getCircleAt(0).getRadius(), 1.0f);

    EXPECT_FLOAT_EQ(world.getCirclePoint(1).x(), 1.0f);
    EXPECT_FLOAT_EQ(world.getCirclePoint(1).y(), -0.5f);
    EXPECT_FLOAT_EQ(world.getCircleRadius(1), 0.5f);

    EXPECT_FLOAT_EQ(world.getCircleX(0), 2.5f);
    EXPECT_FLOAT_EQ(world.getCircleY(0), 2.5f);
    EXPECT_FLOAT_EQ(world.getCircleX(1), 1.0f);
    EXPECT_FLOAT_EQ(world.getCircleY(1), -0.5f);
}

TEST_F(WorldTest, GetArrayCircle)
{
    ASSERT_EQ(world.getArrayCircleX().size(), static_cast<size_t>(2));
    ASSERT_EQ(world.getArrayCircleY().size(), static_cast<size_t>(2));
    ASSERT_EQ(world.getArrayCircleRadius().size(), static_cast<size_t>(2));

    EXPECT_FLOAT_EQ(world.getArrayCircleX()[0], 2.5f);
    EXPECT_FLOAT_EQ(world.getArrayCircleY()[0], 2.5f);
    EXPECT_FLOAT_EQ(world.getArrayCircleRadius()[0], 1.0f);

    EXPECT_FLOAT_EQ(world.getArrayCircleX()[1], 1.0f);
    EXPECT_FLOAT_EQ(world.getArrayCircleY()[1], -0.5f);
    EXPECT_FLOAT_EQ(world.getArrayCircleRadius()[1], 0.5f);
}

// ============================================================================
// Bounding Box Tests
// ============================================================================

TEST_F(WorldTest, GetAabb)
{
    // The second circle reaches down to y = -1
    EXPECT_FLOAT_EQ(world.getAabbMinX(), 0.0f);
    EXPECT_FLOAT_EQ(world.getAabbMinY(), -1.0f);
    EXPECT_FLOAT_EQ(world.getAabbMaxX(), 5.0f);
    EXPECT_FLOAT_EQ(world.getAabbMaxY(), 5.0f);

    EXPECT_FLOAT_EQ(world.getAabbMin().x(), 0.0f);
    EXPECT_FLOAT_EQ(world.getAabbMin().y(), -1.0f);
    EXPECT_FLOAT_EQ(world.getAabbMax().x(), 5.0f);
    EXPECT_FLOAT_EQ(world.getAabbMax().y(), 5.0f);

    EXPECT_FLOAT_EQ(world.getAabbSizeX(), 5.0f);
    EXPECT_FLOAT_EQ(world.getAabbSizeY(), 6.0f);

    EXPECT_FLOAT_EQ(world.getAabbSize().x(), 5.0f);
    EXPECT_FLOAT_EQ(world.getAabbSize().y(), 6.0f);
}

// ============================================================================
// Shape Getter Tests
// ============================================================================

TEST_F(WorldTest, GetShapeMatchesWorld)
{
    const Shape shape = world.getShape();

    ASSERT_EQ(shape.getLines().size(), static_cast<size_t>(4));
    ASSERT_EQ(shape.getCircles().size(), static_cast<size_t>(2));

    for (size_t i = 0; i < shape.getLines().size(); i++)
    {
        EXPECT_FLOAT_EQ(world.getLineAt(i).getPoint1X(), shape.getLines()[i].getPoint1X()) << "line " << i;
        EXPECT_FLOAT_EQ(world.getLineAt(i).getPoint1Y(), shape.getLines()[i].getPoint1Y()) << "line " << i;
        EXPECT_FLOAT_EQ(world.getLineAt(i).getPoint2X(), shape.getLines()[i].getPoint2X()) << "line " << i;
        EXPECT_FLOAT_EQ(world.getLineAt(i).getPoint2Y(), shape.getLines()[i].getPoint2Y()) << "line " << i;
    }

    for (size_t i = 0; i < shape.getCircles().size(); i++)
    {
        EXPECT_FLOAT_EQ(world.getCircleAt(i).getX(), shape.getCircles()[i].getX()) << "circle " << i;
        EXPECT_FLOAT_EQ(world.getCircleAt(i).getY(), shape.getCircles()[i].getY()) << "circle " << i;
        EXPECT_FLOAT_EQ(world.getCircleAt(i).getRadius(), shape.getCircles()[i].getRadius()) << "circle " << i;
    }

    EXPECT_FLOAT_EQ(world.getAabbMinX(), shape.getAabbMinX());
    EXPECT_FLOAT_EQ(world.getAabbMinY(), shape.getAabbMinY());
    EXPECT_FLOAT_EQ(world.getAabbMaxX(), shape.getAabbMaxX());
    EXPECT_FLOAT_EQ(world.getAabbMaxY(), shape.getAabbMaxY());

    EXPECT_FLOAT_EQ(world.getAabbMin().x(), shape.getAabbMin().x());
    EXPECT_FLOAT_EQ(world.getAabbMin().y(), shape.getAabbMin().y());
    EXPECT_FLOAT_EQ(world.getAabbMax().x(), shape.getAabbMax().x());
    EXPECT_FLOAT_EQ(world.getAabbMax().y(), shape.getAabbMax().y());

    EXPECT_FLOAT_EQ(world.getAabbSizeX(), shape.getAabbSizeX());
    EXPECT_FLOAT_EQ(world.getAabbSizeY(), shape.getAabbSizeY());

    EXPECT_FLOAT_EQ(world.getAabbSize().x(), shape.getAabbSize().x());
    EXPECT_FLOAT_EQ(world.getAabbSize().y(), shape.getAabbSize().y());
}
