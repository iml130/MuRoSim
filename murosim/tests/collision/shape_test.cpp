// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/collision/shape.hpp"
#include "murosim/geometry/primitives.hpp"
#include "murosim/math/constants.hpp"

namespace
{
using murosim::math::PI_2;
using murosim::math::PI_4;
using murosim::geometry::Shape;
using murosim::geometry::World;
using murosim::geometry::primitives::pose;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class CollisionShapeTest : public ::testing::Test
{
protected:
    Shape rectangle = murosim::geometry::primitives::rectangle(1.0f, 2.0f);
    Shape square = murosim::geometry::primitives::square(1.0f);
    Shape circle = Shape({}, {{0.0f, 0.0f, 0.5f}});
    Eigen::Affine2f identity = Eigen::Affine2f::Identity();
};

// ============================================================================
// Shape With World Tests
// ============================================================================

TEST_F(CollisionShapeTest, SquareWithWorldSquare)
{
    const World world(square);

    EXPECT_TRUE(murosim::collision::shapeWithWorld(square, identity, world));
    EXPECT_TRUE(murosim::collision::shapeWithWorld(square, pose(0.5f, 0.5f, 0.0f), world));
    EXPECT_FALSE(murosim::collision::shapeWithWorld(square, pose(1.1f, 0.0f, 0.0f), world)); // moved clear
}

TEST_F(CollisionShapeTest, RectangleWithWorldRectangle)
{
    const World world(rectangle);

    EXPECT_TRUE(murosim::collision::shapeWithWorld(rectangle, identity, world));
    EXPECT_TRUE(murosim::collision::shapeWithWorld(rectangle, pose(0.5f, 0.5f, 0.0f), world));
    EXPECT_FALSE(murosim::collision::shapeWithWorld(rectangle, pose(0.0f, 2.1f, 0.0f), world)); // moved clear
}

TEST_F(CollisionShapeTest, CircleWithWorldCircle)
{
    const World world(circle);

    EXPECT_TRUE(murosim::collision::shapeWithWorld(circle, identity, world));
    EXPECT_TRUE(murosim::collision::shapeWithWorld(circle, pose(0.5f, 0.5f, 0.0f), world));
    EXPECT_FALSE(murosim::collision::shapeWithWorld(circle, pose(0.0f, 1.1f, 0.0f), world)); // moved clear
}

TEST_F(CollisionShapeTest, CircleWithWorldSquare)
{
    const World world(square);

    EXPECT_TRUE(murosim::collision::shapeWithWorld(circle, identity, world));
    EXPECT_TRUE(murosim::collision::shapeWithWorld(circle, pose(0.5f, 0.5f, 0.0f), world));
    EXPECT_FALSE(murosim::collision::shapeWithWorld(circle, pose(1.1f, 0.0f, 0.0f), world)); // moved clear
}

// ============================================================================
// Shape With Shape Tests
// ============================================================================

TEST_F(CollisionShapeTest, SquareWithSquare)
{
    EXPECT_TRUE(murosim::collision::shapeWithShape(square, identity, square, identity));
    EXPECT_TRUE(murosim::collision::shapeWithShape(square, pose(0.0f, 0.4f, 0.0f), square, pose(0.4f, 0.0f, 0.0f)));

    // Side by side with a gap of 0.02
    EXPECT_FALSE(murosim::collision::shapeWithShape(square, pose(0.51f, 0.0f, PI_2),
                                                    square, pose(-0.51f, 0.0f, -PI_2)));
}

TEST_F(CollisionShapeTest, RectangleWithRectangle)
{
    EXPECT_TRUE(murosim::collision::shapeWithShape(rectangle, identity, rectangle, identity));
    EXPECT_TRUE(murosim::collision::shapeWithShape(rectangle, pose(0.0f, 0.4f, 0.0f),
                                                   rectangle, pose(0.4f, 0.0f, 0.0f)));

    // Both rotated by 45 degrees and moved apart along the diagonal
    EXPECT_FALSE(murosim::collision::shapeWithShape(rectangle, pose(0.5f, 0.5f, PI_4),
                                                    rectangle, pose(-0.5f, -0.5f, PI_4)));
}

TEST_F(CollisionShapeTest, CircleWithCircle)
{
    EXPECT_TRUE(murosim::collision::shapeWithShape(circle, identity, circle, identity));
    EXPECT_TRUE(murosim::collision::shapeWithShape(circle, pose(0.0f, 0.4f, 0.0f), circle, pose(0.4f, 0.0f, 0.0f)));

    // Centres 1.02 apart, radii sum to 1.0
    EXPECT_FALSE(murosim::collision::shapeWithShape(circle, pose(0.51f, 0.0f, PI_2),
                                                    circle, pose(-0.51f, 0.0f, -PI_2)));
}

TEST_F(CollisionShapeTest, CircleWithSquare)
{
    EXPECT_TRUE(murosim::collision::shapeWithShape(circle, identity, square, identity));
    EXPECT_TRUE(murosim::collision::shapeWithShape(circle, pose(0.0f, 0.2f, 0.0f), square, pose(0.2f, 0.0f, 0.0f)));

    // Disk edge at x = 0.0, square edge at x = -0.01
    EXPECT_FALSE(murosim::collision::shapeWithShape(circle, pose(0.5f, 0.0f, PI_2),
                                                    square, pose(-0.51f, 0.0f, 0.0f)));
}
