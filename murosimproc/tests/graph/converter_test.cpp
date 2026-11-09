// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/graph/converter.hpp"
#include "murosimproc/graph/graph.hpp"
#include "murosim/geometry/line.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::geometry::Line;
using murosimproc::graph::Graph;
using murosimproc::graph::converter::convertToTubes;
using fixtures::EPSILON;
using fixtures::SEED;

constexpr float HALF_WIDTH = 1.0f;
constexpr size_t ONE_BUCKET = 1;
constexpr size_t ARC_SEGMENTS = 8;
constexpr float NO_SIMPLIFY = 0.0f;

// Tube outlines are polygons with round ends approximated by ARC_SEGMENTS,
// so their area falls a little short of the exact stadium area
constexpr float AREA_TOLERANCE = 0.05f;

/// Graph of one edge from the origin to (`length`, 0).
Graph singleEdge(const float length)
{
    Graph graph;
    murosimproc::graph::addEdge(graph, murosimproc::graph::addNode(graph, Eigen::Vector2f(0.0f, 0.0f)),
                                murosimproc::graph::addNode(graph, Eigen::Vector2f(length, 0.0f)));
    return graph;
}

/// Areas of all closed loops, largest first.
std::vector<float> loopAreas(const std::vector<Line> &lines)
{
    std::vector<float> areas;
    for (const std::vector<Eigen::Vector2f> &loop : fixtures::closedLoops(lines))
    {
        areas.push_back(std::abs(fixtures::signedArea(loop)));
    }
    std::sort(areas.rbegin(), areas.rend());
    return areas;
}

void boundingBox(const std::vector<Line> &lines, Eigen::Vector2f &low, Eigen::Vector2f &high)
{
    low = Eigen::Vector2f(1e9f, 1e9f);
    high = -low;
    for (const Line &line : lines)
    {
        low = low.cwiseMin(line.getPoint1());
        high = high.cwiseMax(line.getPoint1());
    }
}
} // namespace

// ============================================================================
// Degenerate Input Tests
// ============================================================================

TEST(GraphConverterTubes, EmptyGraphGivesNoLines)
{
    EXPECT_TRUE(convertToTubes(Graph(), HALF_WIDTH, HALF_WIDTH, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY).empty());
}

TEST(GraphConverterTubes, ZeroBucketsGiveNoLines)
{
    EXPECT_TRUE(convertToTubes(singleEdge(4.0f), HALF_WIDTH, HALF_WIDTH, SEED, 0, ARC_SEGMENTS, NO_SIMPLIFY).empty());
}

TEST(GraphConverterTubes, VerticesWithoutEdgesGiveNoLines)
{
    Graph graph;
    murosimproc::graph::addNode(graph, Eigen::Vector2f(0.0f, 0.0f));
    murosimproc::graph::addNode(graph, Eigen::Vector2f(3.0f, 0.0f));

    EXPECT_TRUE(convertToTubes(graph, HALF_WIDTH, HALF_WIDTH, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY).empty());
}

// ============================================================================
// Single Edge Tests
// ============================================================================

TEST(GraphConverterTubes, SingleEdgeGivesOneStadiumOutline)
{
    const float length = 4.0f;

    const std::vector<Line> lines =
        convertToTubes(singleEdge(length), HALF_WIDTH, HALF_WIDTH, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY);

    const std::vector<float> areas = loopAreas(lines);
    ASSERT_EQ(areas.size(), static_cast<size_t>(1));
    const float exact = 2.0f * HALF_WIDTH * length + PI * HALF_WIDTH * HALF_WIDTH;
    EXPECT_LT(areas.front(), exact + EPSILON);
    EXPECT_GT(areas.front(), (1.0f - AREA_TOLERANCE) * exact);
}

TEST(GraphConverterTubes, OutlineStaysHalfWidthAwayFromTheEdge)
{
    const std::vector<Line> lines =
        convertToTubes(singleEdge(4.0f), HALF_WIDTH, HALF_WIDTH, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY);

    Eigen::Vector2f low;
    Eigen::Vector2f high;
    boundingBox(lines, low, high);
    EXPECT_NEAR(low.x(), -HALF_WIDTH, 1e-4f);
    EXPECT_NEAR(low.y(), -HALF_WIDTH, 1e-4f);
    EXPECT_NEAR(high.x(), 4.0f + HALF_WIDTH, 1e-4f);
    EXPECT_NEAR(high.y(), HALF_WIDTH, 1e-4f);
}

TEST(GraphConverterTubes, SingleBucketUsesTheMiddleOfTheSpacingRange)
{
    const std::vector<Line> lines =
        convertToTubes(singleEdge(4.0f), 1.0f, 3.0f, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY);

    Eigen::Vector2f low;
    Eigen::Vector2f high;
    boundingBox(lines, low, high);
    EXPECT_NEAR(low.y(), -2.0f, 1e-4f); // (1 + 3) / 2
    EXPECT_NEAR(high.y(), 2.0f, 1e-4f);
}

TEST(GraphConverterTubes, NoSegmentIsDegenerate)
{
    const std::vector<Line> lines =
        convertToTubes(singleEdge(4.0f), HALF_WIDTH, HALF_WIDTH, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY);

    for (size_t i = 0; i < lines.size(); i++)
    {
        ASSERT_GT(lines[i].getLength(), EPSILON) << "line " << i;
    }
}

// ============================================================================
// Network Tests
// ============================================================================

TEST(GraphConverterTubes, PlusGraphMergesIntoOneOutline)
{
    const std::vector<Line> lines =
        convertToTubes(fixtures::plusGraph(), 0.25f, 0.25f, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY);

    const std::vector<float> areas = loopAreas(lines);
    ASSERT_EQ(areas.size(), static_cast<size_t>(1)); // one outline, no holes
    // two crossing bars of 4 x 0.5 overlapping in the 0.5 x 0.5 centre, plus
    // four round tips of radius 0.25 (half discs)
    const float exact = 2.0f * 4.0f * 0.5f - 0.5f * 0.5f + 4.0f * PI * 0.25f * 0.25f / 2.0f;
    EXPECT_GT(areas.front(), (1.0f - AREA_TOLERANCE) * exact);
    EXPECT_LT(areas.front(), exact + EPSILON);
}

TEST(GraphConverterTubes, NarrowGridTubesLeaveHoles)
{
    const std::vector<Line> lines =
        convertToTubes(fixtures::gridGraph(3, 5.0f), 0.5f, 0.5f, SEED, ONE_BUCKET, ARC_SEGMENTS, NO_SIMPLIFY);

    // one outline and the four cells between the tubes
    const std::vector<float> areas = loopAreas(lines);
    ASSERT_EQ(areas.size(), static_cast<size_t>(5));
    for (size_t i = 1; i < areas.size(); i++)
    {
        EXPECT_NEAR(areas[i], 4.0f * 4.0f, 0.1f) << "hole " << i; // (5 - 2 * 0.5)^2, corners rounded
    }
}

TEST(GraphConverterTubes, SeveralBucketsVaryTheWidthDeterministically)
{
    const Graph graph = fixtures::gridGraph(3, 5.0f);

    const std::vector<Line> first = convertToTubes(graph, 0.3f, 0.9f, SEED, 4, ARC_SEGMENTS, NO_SIMPLIFY);
    const std::vector<Line> second = convertToTubes(graph, 0.3f, 0.9f, SEED, 4, ARC_SEGMENTS, NO_SIMPLIFY);
    const std::vector<Line> other_seed = convertToTubes(graph, 0.3f, 0.9f, SEED + 1, 4, ARC_SEGMENTS, NO_SIMPLIFY);

    ASSERT_EQ(first.size(), second.size());
    for (size_t i = 0; i < first.size(); i++)
    {
        ASSERT_FLOAT_EQ(first[i].getPoint1X(), second[i].getPoint1X()) << "line " << i;
        ASSERT_FLOAT_EQ(first[i].getPoint1Y(), second[i].getPoint1Y()) << "line " << i;
    }
    EXPECT_NE(first.size(), other_seed.size()); // a different bucket assignment changes the outline
}

TEST(GraphConverterTubes, SimplificationReducesTheSegmentCount)
{
    const Graph graph = fixtures::gridGraph(3, 5.0f);

    const std::vector<Line> detailed = convertToTubes(graph, 0.5f, 0.5f, SEED, ONE_BUCKET, 16, NO_SIMPLIFY);
    const std::vector<Line> simplified = convertToTubes(graph, 0.5f, 0.5f, SEED, ONE_BUCKET, 16, 0.2f);

    EXPECT_LT(simplified.size(), detailed.size());
    EXPECT_EQ(loopAreas(simplified).size(), static_cast<size_t>(5));
}

// ============================================================================
// Boost Geometry Tests
// ============================================================================

// Regression: with system Boost.Geometry <= 1.88 (1.83 on Ubuntu 24.04) the
// union of the bucket buffers for this exact input produced two polygons,
// one of them a stray fragment of 172.6 that survives removeFragments(),
// and lost a third of the total area; corridor walls went missing about
// once in 10,000 generated maps. The fetched Boost.Geometry 1.90 pinned in
// murosimproc/CMakeLists.txt yields one polygon of area 1626.73. Do not
// weaken the pin: this test is what notices.
TEST(GraphConverterBoost, GridUnionYieldsOnePolygonWithTheFullArea)
{
    // 8 x 8 grid, step 5, tube half-width 2.72 just past the touching
    // threshold of step / 2, 8 buckets, seed 12345
    const Graph graph = fixtures::gridGraph(8, 5.0f);

    const std::vector<Line> lines = convertToTubes(graph, 2.72f, 2.72f, 12345, 8, ARC_SEGMENTS, NO_SIMPLIFY);

    const std::vector<float> areas = loopAreas(lines);
    ASSERT_EQ(areas.size(), static_cast<size_t>(1)) << "the tubes must merge into a single outline";
    EXPECT_NEAR(areas.front(), 1626.73f, 0.5f);
}
