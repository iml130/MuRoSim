// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/wfc/post_processor.hpp"
#include "murosimproc/util/image.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
using murosim::math::PI;
using murosimproc::generators::RandomRange;
using murosimproc::obstacle::ShapeDistribution;
using murosimproc::util::Array2d;
using murosimproc::wfc::PostProcessor;
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::navigation::Node;
using murosim::random::RandomGenerator;

constexpr float CELL_SIZE = 0.5f;
constexpr float NODE_RADIUS = 0.3f;
constexpr size_t BORDER_LINES = 4;
constexpr uint32_t SEEDS = 20;
constexpr float OBSTACLE_DENSITY = 4.0f; // per cell² of obstacle zone
constexpr float OBSTACLE_SIZE_MIN = 0.3f;
constexpr float OBSTACLE_SIZE_MAX = 0.5f;
const ShapeDistribution ALL_CIRCLES = {1.0f, 0.0f, 0.0f, 0.05f};
const ShapeDistribution ALL_POLYGONS = {0.0f, 1.0f, 0.0f, 0.05f};

// Room of 5 x 3 free cells with a 1-wide corridor stub below it: every
// room cell is a node candidate, the stub cell (3, 4) is a corridor and
// (3, 5) a dead end.
const std::vector<std::string> ROOM_WITH_STUB = {
    "#######",
    "#     #",
    "#     #",
    "#     #",
    "### ###",
    "### ###",
    "#######",
};

// One 1-wide corridor: only the two dead ends (1, 1) and (5, 1) are candidates.
const std::vector<std::string> CORRIDOR = {
    "#######",
    "#     #",
    "#######",
};

// A 3 x 2 room next to a single-cell pocket at (5, 1) that is walled off.
const std::vector<std::string> ROOM_AND_POCKET = {
    "#######",
    "#   # #",
    "#   ###",
    "#######",
};

// Free space around a 3 x 3 obstacle zone spanning cells (2..4, 2..4).
const std::vector<std::string> OBSTACLE_ZONE = {
    "        ",
    "        ",
    "  ...   ",
    "  ...   ",
    "  ...   ",
    "        ",
    "        ",
};

struct Geometry
{
    std::vector<Line> lines;
    std::vector<Circle> circles;
    std::vector<Node> nodes;
};

PostProcessor noObstacles()
{
    return PostProcessor(RandomRange(0.0f), RandomRange(OBSTACLE_SIZE_MIN, OBSTACLE_SIZE_MAX), ALL_CIRCLES);
}

PostProcessor withObstacles(const ShapeDistribution &shape = ALL_CIRCLES)
{
    return PostProcessor(RandomRange(OBSTACLE_DENSITY), RandomRange(OBSTACLE_SIZE_MIN, OBSTACLE_SIZE_MAX), shape);
}

Geometry process(const PostProcessor &post_processor, const std::vector<std::string> &rows,
                 const size_t target_node_count, const uint64_t seed = fixtures::SEED,
                 const float cell_size = CELL_SIZE)
{
    Geometry geometry;
    RandomGenerator random_generator(seed);
    post_processor.process(fixtures::makeImage(rows), geometry.lines, geometry.circles, geometry.nodes,
                           random_generator, cell_size, NODE_RADIUS, target_node_count);
    return geometry;
}

/// Grid cell whose centre the node sits on.
Eigen::Vector2i cellOf(const Node &node, const float cell_size = CELL_SIZE)
{
    return Eigen::Vector2i(static_cast<int>(std::floor(node.getX() / cell_size)),
                           static_cast<int>(std::floor(node.getY() / cell_size)));
}

bool hasNodeInCell(const std::vector<Node> &nodes, const int x, const int y)
{
    for (const Node &node : nodes)
    {
        if (cellOf(node) == Eigen::Vector2i(x, y))
        {
            return true;
        }
    }
    return false;
}
} // namespace

// ============================================================================
// Node Tests
// ============================================================================

TEST(PostProcessorNodes, NodesSitOnCellCentresWithTheRequestedRadius)
{
    const Geometry geometry = process(noObstacles(), CORRIDOR, 5);

    ASSERT_EQ(geometry.nodes.size(), static_cast<size_t>(5));
    for (size_t i = 0; i < geometry.nodes.size(); i++)
    {
        const Eigen::Vector2i cell = cellOf(geometry.nodes[i]);
        EXPECT_FLOAT_EQ(geometry.nodes[i].getX(), (static_cast<float>(cell.x()) + 0.5f) * CELL_SIZE) << "node " << i;
        EXPECT_FLOAT_EQ(geometry.nodes[i].getY(), (static_cast<float>(cell.y()) + 0.5f) * CELL_SIZE) << "node " << i;
        EXPECT_EQ(cell.y(), 1) << "node " << i;
        EXPECT_FLOAT_EQ(geometry.nodes[i].getRadius(), NODE_RADIUS) << "node " << i;
        EXPECT_FLOAT_EQ(geometry.nodes[i].getStartAngle(), 0.0f) << "node " << i;
        EXPECT_FLOAT_EQ(geometry.nodes[i].getStartBias(), PI) << "node " << i;
    }
}

TEST(PostProcessorNodes, EveryFreeCellIsUsedAtMostOnce)
{
    const Geometry geometry = process(noObstacles(), CORRIDOR, 5);

    ASSERT_EQ(geometry.nodes.size(), static_cast<size_t>(5));
    for (int x = 1; x <= 5; x++)
    {
        EXPECT_TRUE(hasNodeInCell(geometry.nodes, x, 1)) << "cell " << x;
    }
}

TEST(PostProcessorNodes, CountIsCappedByTheFreeCells)
{
    EXPECT_EQ(process(noObstacles(), CORRIDOR, 50).nodes.size(), static_cast<size_t>(5));
    EXPECT_EQ(process(noObstacles(), CORRIDOR, 0).nodes.size(), static_cast<size_t>(0));
}

TEST(PostProcessorNodes, CandidateCellsAreUsedBeforeCorridorCells)
{
    const Array2d<uint8_t> image = fixtures::makeImage(ROOM_WITH_STUB);

    for (uint32_t seed = 0; seed < SEEDS; seed++)
    {
        const Geometry geometry = process(noObstacles(), ROOM_WITH_STUB, 4, seed);
        ASSERT_EQ(geometry.nodes.size(), static_cast<size_t>(4)) << "seed " << seed;
        for (size_t i = 0; i < geometry.nodes.size(); i++)
        {
            const Eigen::Vector2i cell = cellOf(geometry.nodes[i]);
            ASSERT_TRUE(murosimproc::util::image::isNodeCandidate(image, cell.x(), cell.y()))
                << "seed " << seed << " node " << i << " at cell " << cell.transpose();
        }
    }
}

TEST(PostProcessorNodes, FallbackUsesCorridorCellsWhenCandidatesRunOut)
{
    const Geometry geometry = process(noObstacles(), CORRIDOR, 4, 7);

    ASSERT_EQ(geometry.nodes.size(), static_cast<size_t>(4));
    EXPECT_TRUE(hasNodeInCell(geometry.nodes, 1, 1)); // both dead ends come first
    EXPECT_TRUE(hasNodeInCell(geometry.nodes, 5, 1));
}

// Farthest-point sampling: after the random first node, the second is the
// cell farthest from it.
TEST(PostProcessorNodes, SecondNodeIsFarthestFromTheFirst)
{
    for (uint32_t seed = 0; seed < SEEDS; seed++)
    {
        const Geometry geometry = process(noObstacles(), CORRIDOR, 2, seed);
        ASSERT_EQ(geometry.nodes.size(), static_cast<size_t>(2)) << "seed " << seed;
        const int first = cellOf(geometry.nodes[0]).x();
        const int second = cellOf(geometry.nodes[1]).x();
        ASSERT_TRUE(first == 1 || first == 5) << "seed " << seed; // only the dead ends are candidates
        ASSERT_EQ(second, first == 1 ? 5 : 1) << "seed " << seed;
    }
}

TEST(PostProcessorNodes, WalledOffPocketGetsNoNode)
{
    const Geometry geometry = process(noObstacles(), ROOM_AND_POCKET, 10);

    EXPECT_EQ(geometry.nodes.size(), static_cast<size_t>(6)); // the 3 x 2 room only
    EXPECT_FALSE(hasNodeInCell(geometry.nodes, 5, 1));
}

TEST(PostProcessorNodes, ObstacleZoneCellsGetNoNode)
{
    const Geometry geometry = process(noObstacles(), OBSTACLE_ZONE, 100);
    const Array2d<uint8_t> image = fixtures::makeImage(OBSTACLE_ZONE);

    EXPECT_EQ(geometry.nodes.size(), static_cast<size_t>(8 * 7 - 9));
    for (size_t i = 0; i < geometry.nodes.size(); i++)
    {
        const Eigen::Vector2i cell = cellOf(geometry.nodes[i]);
        ASSERT_EQ(murosimproc::util::image::at(image, cell), static_cast<uint8_t>(0)) << "node " << i;
    }
}

// ============================================================================
// Wall Tests
// ============================================================================

TEST(PostProcessorWalls, BorderSpansTheLargerImageSide)
{
    const Geometry geometry = process(noObstacles(), CORRIDOR, 0);

    ASSERT_GE(geometry.lines.size(), BORDER_LINES);
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::undirectedLoops(geometry.lines);
    const float world_size = 7.0f * CELL_SIZE;
    bool has_border = false;
    for (const std::vector<Eigen::Vector2f> &loop : loops)
    {
        has_border = has_border || std::abs(std::abs(fixtures::signedArea(loop)) - world_size * world_size) < 1e-4f;
    }
    EXPECT_TRUE(has_border);
}

TEST(PostProcessorWalls, WallContoursAreScaledByTheCellSize)
{
    const Geometry geometry = process(noObstacles(), ROOM_AND_POCKET, 0);

    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::undirectedLoops(geometry.lines);
    const float room_area = 3.0f * 2.0f * CELL_SIZE * CELL_SIZE;
    bool has_room = false;
    for (const std::vector<Eigen::Vector2f> &loop : loops)
    {
        has_room = has_room || std::abs(std::abs(fixtures::signedArea(loop)) - room_area) < 1e-4f;
    }
    EXPECT_TRUE(has_room);
    EXPECT_TRUE(geometry.circles.empty());
}

TEST(PostProcessorWalls, FilledPocketLeavesNoContour)
{
    const Geometry geometry = process(noObstacles(), ROOM_AND_POCKET, 0);

    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::undirectedLoops(geometry.lines);
    const float pocket_area = CELL_SIZE * CELL_SIZE;
    for (size_t i = 0; i < loops.size(); i++)
    {
        EXPECT_GT(std::abs(fixtures::signedArea(loops[i])), pocket_area + 1e-4f) << "loop " << i;
    }
}

TEST(PostProcessorWalls, ImageWithoutWallsGivesOnlyTheBorder)
{
    const Geometry geometry = process(noObstacles(), OBSTACLE_ZONE, 0);

    EXPECT_EQ(geometry.lines.size(), BORDER_LINES);
    EXPECT_TRUE(geometry.circles.empty());
}

// ============================================================================
// Obstacle Tests
// ============================================================================

TEST(PostProcessorObstacles, ZoneIsFilledWithCirclesInsideItsBounds)
{
    const Geometry geometry = process(withObstacles(), OBSTACLE_ZONE, 0);

    ASSERT_FALSE(geometry.circles.empty());
    EXPECT_EQ(geometry.lines.size(), BORDER_LINES);
    const float zone_min = 2.0f * CELL_SIZE;
    const float zone_max = 5.0f * CELL_SIZE;
    for (size_t i = 0; i < geometry.circles.size(); i++)
    {
        const Circle &circle = geometry.circles[i];
        EXPECT_GE(circle.getX() - circle.getRadius(), zone_min - fixtures::EPSILON) << "circle " << i;
        EXPECT_GE(circle.getY() - circle.getRadius(), zone_min - fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getX() + circle.getRadius(), zone_max + fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getY() + circle.getRadius(), zone_max + fixtures::EPSILON) << "circle " << i;
        EXPECT_LE(circle.getRadius(), OBSTACLE_SIZE_MAX / 2.0f * CELL_SIZE + fixtures::EPSILON) << "circle " << i;
    }
}

TEST(PostProcessorObstacles, PolygonsStayInsideTheZone)
{
    const Geometry geometry = process(withObstacles(ALL_POLYGONS), OBSTACLE_ZONE, 0);

    EXPECT_TRUE(geometry.circles.empty());
    ASSERT_GT(geometry.lines.size(), BORDER_LINES);
    const float zone_min = 2.0f * CELL_SIZE;
    const float zone_max = 5.0f * CELL_SIZE;
    for (size_t i = BORDER_LINES; i < geometry.lines.size(); i++)
    {
        for (const Eigen::Vector2f &point : {geometry.lines[i].getPoint1(), geometry.lines[i].getPoint2()})
        {
            ASSERT_GE(point.x(), zone_min - fixtures::EPSILON) << "line " << i;
            ASSERT_GE(point.y(), zone_min - fixtures::EPSILON) << "line " << i;
            ASSERT_LE(point.x(), zone_max + fixtures::EPSILON) << "line " << i;
            ASSERT_LE(point.y(), zone_max + fixtures::EPSILON) << "line " << i;
        }
    }
}

TEST(PostProcessorObstacles, ZeroDensityPlacesNothing)
{
    const Geometry geometry = process(noObstacles(), OBSTACLE_ZONE, 0);

    EXPECT_TRUE(geometry.circles.empty());
    EXPECT_EQ(geometry.lines.size(), BORDER_LINES);
}

TEST(PostProcessorObstacles, ImageWithoutZoneGetsNoObstacles)
{
    const Geometry geometry = process(withObstacles(), ROOM_WITH_STUB, 0);

    EXPECT_TRUE(geometry.circles.empty());
}

// ============================================================================
// Output Tests
// ============================================================================

TEST(PostProcessorOutput, GeometryIsAppended)
{
    Geometry geometry;
    geometry.lines.emplace_back(20.0f, 20.0f, 21.0f, 21.0f);
    geometry.circles.emplace_back(30.0f, 30.0f, 0.5f);
    geometry.nodes.emplace_back(40.0f, 40.0f, 0.5f);
    RandomGenerator random_generator(fixtures::SEED);

    withObstacles().process(fixtures::makeImage(OBSTACLE_ZONE), geometry.lines, geometry.circles, geometry.nodes,
                            random_generator, CELL_SIZE, NODE_RADIUS, 2);

    ASSERT_GT(geometry.lines.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(geometry.lines[0].getPoint1X(), 20.0f);
    ASSERT_GT(geometry.circles.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(geometry.circles[0].getX(), 30.0f);
    ASSERT_EQ(geometry.nodes.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(geometry.nodes[0].getX(), 40.0f);
}

TEST(PostProcessorOutput, SameSeedGivesTheSameGeometry)
{
    const Geometry first = process(withObstacles(), OBSTACLE_ZONE, 5, 7);
    const Geometry second = process(withObstacles(), OBSTACLE_ZONE, 5, 7);

    ASSERT_EQ(first.circles.size(), second.circles.size());
    ASSERT_EQ(first.nodes.size(), second.nodes.size());
    for (size_t i = 0; i < first.circles.size(); i++)
    {
        ASSERT_FLOAT_EQ(first.circles[i].getX(), second.circles[i].getX()) << "circle " << i;
        ASSERT_FLOAT_EQ(first.circles[i].getRadius(), second.circles[i].getRadius()) << "circle " << i;
    }
    for (size_t i = 0; i < first.nodes.size(); i++)
    {
        ASSERT_FLOAT_EQ(first.nodes[i].getX(), second.nodes[i].getX()) << "node " << i;
        ASSERT_FLOAT_EQ(first.nodes[i].getY(), second.nodes[i].getY()) << "node " << i;
    }
}
