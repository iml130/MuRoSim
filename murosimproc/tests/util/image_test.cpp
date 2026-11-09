// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimproc/util/image.hpp"
#include "murosimproc/util/array2d.hpp"
#include "fixtures.hpp"
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using Image = murosimproc::util::Array2d<uint8_t>;
using fixtures::EPSILON;

constexpr uint8_t FREE = 0;
constexpr uint8_t WALL = 1;
constexpr uint8_t MARK = 3;

size_t count(const Image &image, const uint8_t value)
{
    size_t total = 0;
    for (size_t i = 0; i < image.size(); i++)
    {
        total += image[i] == value;
    }
    return total;
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// A room split by a wall with one gap: two free halves of 6 cells each,
// connected only through the gap cell (3, 3), 13 free cells in total.
class UtilImageTest : public ::testing::Test
{
protected:
    Image image = fixtures::makeImage({
        "#######",
        "#  #  #",
        "#  #  #",
        "#     #",
        "#######",
    });
};

// ============================================================================
// In Image Tests
// ============================================================================

TEST_F(UtilImageTest, IsInImageAcceptsAllCellsAndRejectsTheOutside)
{
    using murosimproc::util::image::isInImage;

    EXPECT_TRUE(isInImage(image, Eigen::Vector2i(0, 0)));
    EXPECT_TRUE(isInImage(image, 6, 4));
    EXPECT_TRUE(isInImage(image, static_cast<size_t>(6), static_cast<size_t>(4)));
    EXPECT_FALSE(isInImage(image, Eigen::Vector2i(7, 0)));
    EXPECT_FALSE(isInImage(image, -1, 0));
    EXPECT_FALSE(isInImage(image, 0, 5));
    EXPECT_FALSE(isInImage(image, static_cast<size_t>(7), static_cast<size_t>(0)));
}

// ============================================================================
// Fill Region Tests
// ============================================================================

TEST_F(UtilImageTest, FillRegionReplacesTheConnectedRegionOnly)
{
    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(1, 1), FREE, MARK);

    // the gap at (3, 3) connects both halves: all 13 free cells are marked
    EXPECT_EQ(count(image, MARK), static_cast<size_t>(13));
    EXPECT_EQ(count(image, FREE), static_cast<size_t>(0));
    EXPECT_EQ(image(3, 1), WALL); // walls untouched
}

TEST_F(UtilImageTest, FillRegionStopsAtWalls)
{
    image(3, 3) = WALL; // close the gap

    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(1, 1), FREE, MARK);

    EXPECT_EQ(count(image, MARK), static_cast<size_t>(6)); // the left half only
    EXPECT_EQ(image(5, 1), FREE);
}

TEST_F(UtilImageTest, FillRegionIsFourConnected)
{
    Image diagonal = fixtures::makeImage({
        " #",
        "# ",
    });

    murosimproc::util::image::fillRegion(diagonal, Eigen::Vector2i(0, 0), FREE, MARK);

    EXPECT_EQ(diagonal(0, 0), MARK);
    EXPECT_EQ(diagonal(1, 1), FREE); // only diagonally adjacent
}

TEST_F(UtilImageTest, FillRegionIsANoOpOutsideOrOnAnotherValue)
{
    const Image before = image;

    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(10, 10), FREE, MARK);
    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(0, 0), FREE, MARK); // a wall cell
    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(1, 1), FREE, FREE); // same value would never end

    EXPECT_TRUE(image == before);
}

TEST_F(UtilImageTest, FillRegionReportsTheChangedIndices)
{
    std::vector<size_t> indices = {99};
    image(3, 3) = WALL;

    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(5, 1), FREE, MARK, indices);

    EXPECT_EQ(indices.size(), static_cast<size_t>(6)); // the right half
    for (size_t i = 0; i < indices.size(); i++)
    {
        EXPECT_EQ(image[indices[i]], MARK) << "index " << indices[i];
    }
}

TEST_F(UtilImageTest, FillRegionWithIndicesClearsTheVectorOnANoOp)
{
    std::vector<size_t> indices = {99};

    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(0, 0), FREE, MARK, indices);

    EXPECT_TRUE(indices.empty());
}

// The overloads without an old value fill every connected cell that differs
// from the new value, whatever its value: walls and free space together.
// The WFC post-processor relies on this to treat free cells and obstacle
// zones as one region.
TEST_F(UtilImageTest, FillRegionWithoutOldValueFloodsEveryOtherValue)
{
    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(0, 0), MARK); // starts on a wall

    EXPECT_EQ(count(image, MARK), image.size()); // walls and free cells alike
}

TEST_F(UtilImageTest, FillRegionWithoutOldValueStopsAtTheTargetValue)
{
    image(3, 3) = MARK; // close the gap with the target value

    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(1, 1), MARK);

    // the fill crosses walls but not cells that already hold the target: the
    // gap cell blocks nothing though, since the wall ring connects both halves
    EXPECT_EQ(count(image, MARK), image.size());
}

TEST_F(UtilImageTest, FillRegionWithoutOldValueJoinsFreeCellsAndObstacleZones)
{
    Image zones = fixtures::makeImage({
        "#####",
        "#.. #",
        "#####",
    });

    std::vector<size_t> indices;
    murosimproc::util::image::fillRegion(zones, Eigen::Vector2i(1, 1), WALL, indices);

    EXPECT_EQ(indices.size(), static_cast<size_t>(3)); // the two zone cells and the free cell next to them
    EXPECT_EQ(count(zones, WALL), zones.size());
}

TEST_F(UtilImageTest, FillRegionWithoutOldValueReportsIndices)
{
    std::vector<size_t> indices;

    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(1, 1), MARK, indices);

    EXPECT_EQ(indices.size(), image.size());
    EXPECT_EQ(count(image, MARK), image.size());
}

TEST_F(UtilImageTest, FillRegionWithoutOldValueIsANoOpOnTheTargetValue)
{
    std::vector<size_t> indices = {99};
    const Image before = image;

    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(1, 1), FREE, indices); // already FREE
    murosimproc::util::image::fillRegion(image, Eigen::Vector2i(20, 20), MARK, indices); // outside

    EXPECT_TRUE(image == before);
    EXPECT_TRUE(indices.empty());
}

// ============================================================================
// Node Candidate Tests
// ============================================================================

// A robot parked on a candidate cell does not block others: dead ends and
// cells of a free 2x2 block qualify, corridors and junctions do not.

TEST(UtilImageNodeCandidate, DeadEndAndCorridor)
{
    const Image image = fixtures::makeImage({
        "#####",
        "#   #",
        "### #",
        "#####",
    });
    using murosimproc::util::image::isNodeCandidate;

    EXPECT_TRUE(isNodeCandidate(image, 1, 1));  // dead end of the horizontal corridor
    EXPECT_FALSE(isNodeCandidate(image, 2, 1)); // straight corridor cell
    EXPECT_FALSE(isNodeCandidate(image, 3, 1)); // curve, the inner diagonal is a wall
    EXPECT_TRUE(isNodeCandidate(image, 3, 2));  // end of the vertical stub
}

TEST(UtilImageNodeCandidate, IntersectionsAreNoCandidates)
{
    const Image image = fixtures::makeImage({
        "##### #####",
        "#         #",
        "### # # ###",
        "### # # ###",
    });
    using murosimproc::util::image::isNodeCandidate;

    EXPECT_FALSE(isNodeCandidate(image, 5, 1)); // four-way crossing
    EXPECT_FALSE(isNodeCandidate(image, 3, 1)); // T junction
    EXPECT_FALSE(isNodeCandidate(image, 7, 1));
    EXPECT_FALSE(isNodeCandidate(image, 2, 1)); // corridor between the junctions
    EXPECT_FALSE(isNodeCandidate(image, 0, 0)); // a wall cell never qualifies
}

TEST(UtilImageNodeCandidate, OpenAreaIsACandidate)
{
    const Image image = fixtures::makeImage({
        "######",
        "#    #",
        "#    #",
        "#    #",
        "######",
    });

    // every room cell, corner, edge or interior, is part of a free 2x2 block
    for (int y = 1; y <= 3; y++)
    {
        for (int x = 1; x <= 4; x++)
        {
            EXPECT_TRUE(murosimproc::util::image::isNodeCandidate(image, x, y)) << "cell " << x << ", " << y;
        }
    }
}

TEST(UtilImageNodeCandidate, AlcoveNextToACorridor)
{
    const Image image = fixtures::makeImage({
        "#####",
        "## ##",
        "#   #",
        "#####",
    });

    EXPECT_TRUE(murosimproc::util::image::isNodeCandidate(image, 2, 1));  // the alcove is a dead end
    EXPECT_FALSE(murosimproc::util::image::isNodeCandidate(image, 2, 2)); // the cell in front of it is a T junction
}

TEST(UtilImageNodeCandidate, DiagonalContactIsNoCandidate)
{
    const Image image = fixtures::makeImage({
        "####",
        "# ##",
        "## #",
        "####",
    });

    // diagonal-only contact: no robot can pass, no 2x2 block, no dead end
    EXPECT_FALSE(murosimproc::util::image::isNodeCandidate(image, 1, 1));
    EXPECT_FALSE(murosimproc::util::image::isNodeCandidate(image, 2, 2));
}

TEST(UtilImageNodeCandidate, OutsideTheImageIsNoCandidate)
{
    const Image image = fixtures::makeImage({"  ", "  "});

    EXPECT_FALSE(murosimproc::util::image::isNodeCandidate(image, -1, 0));
    EXPECT_FALSE(murosimproc::util::image::isNodeCandidate(image, 2, 0));
}

// ============================================================================
// Contour Tests
// ============================================================================

// Contour points sit on the boundary between a wall cell and a free cell,
// in coordinates where cell (x, y) covers [x, x + 1] x [y, y + 1].

TEST(UtilImageContour, SingleWallCellGivesOneClosedSquare)
{
    const Image image = fixtures::makeImage({
        "     ",
        "     ",
        "  #  ",
        "     ",
        "     ",
    });

    const std::vector<std::vector<Eigen::Vector2f>> contours = murosimproc::util::image::findContourPoints(image);

    ASSERT_EQ(contours.size(), static_cast<size_t>(1));
    const std::vector<Eigen::Vector2f> &contour = contours.front();
    ASSERT_EQ(contour.size(), static_cast<size_t>(5)); // four corners plus the repeated start
    EXPECT_NEAR(contour.front().x(), contour.back().x(), EPSILON);
    EXPECT_NEAR(contour.front().y(), contour.back().y(), EPSILON);
    for (size_t i = 0; i < 4; i++)
    {
        // every corner of cell (2, 2)
        EXPECT_TRUE((contour[i].x() == 2.0f || contour[i].x() == 3.0f) &&
                    (contour[i].y() == 2.0f || contour[i].y() == 3.0f))
            << "point " << i << " at " << contour[i].transpose();
    }
}

TEST(UtilImageContour, WallBlockOutlineEnclosesItsArea)
{
    const Image image = fixtures::makeImage({
        "      ",
        " ###  ",
        " ###  ",
        "      ",
    });

    const std::vector<std::vector<Eigen::Vector2f>> contours = murosimproc::util::image::findContourPoints(image);

    ASSERT_EQ(contours.size(), static_cast<size_t>(1));
    std::vector<Eigen::Vector2f> loop = contours.front();
    loop.pop_back(); // drop the repeated closing point
    EXPECT_NEAR(std::abs(fixtures::signedArea(loop)), 6.0f, EPSILON); // 3 x 2 cells
}

TEST(UtilImageContour, TwoSeparateWallsGiveTwoContours)
{
    const Image image = fixtures::makeImage({
        "       ",
        " #   # ",
        "       ",
    });

    EXPECT_EQ(murosimproc::util::image::findContourPoints(image).size(), static_cast<size_t>(2));
}

TEST(UtilImageContour, FreeRoomInsideWallsGivesAClosedInnerContour)
{
    const Image image = fixtures::makeImage({
        "#####",
        "#   #",
        "#   #",
        "#####",
    });

    const std::vector<std::vector<Eigen::Vector2f>> contours = murosimproc::util::image::findContourPoints(image);

    ASSERT_EQ(contours.size(), static_cast<size_t>(1));
    std::vector<Eigen::Vector2f> loop = contours.front();
    loop.pop_back();
    EXPECT_NEAR(std::abs(fixtures::signedArea(loop)), 6.0f, EPSILON); // the 3 x 2 room
}

TEST(UtilImageContour, WallTouchingTheBorderGivesAnOpenContour)
{
    const Image image = fixtures::makeImage({
        "##   ",
        "##   ",
        "     ",
    });

    const std::vector<std::vector<Eigen::Vector2f>> contours = murosimproc::util::image::findContourPoints(image);

    // the outline runs from one image border to the other and does not close
    ASSERT_EQ(contours.size(), static_cast<size_t>(1));
    const std::vector<Eigen::Vector2f> &contour = contours.front();
    ASSERT_GE(contour.size(), static_cast<size_t>(2));
    EXPECT_FALSE(contour.front().x() == contour.back().x() && contour.front().y() == contour.back().y());
    EXPECT_TRUE(contour.front().x() == 0.0f || contour.front().y() == 0.0f); // starts on the border
    EXPECT_TRUE(contour.back().x() == 0.0f || contour.back().y() == 0.0f);   // ends on the border
}

TEST(UtilImageContour, UniformImageHasNoContour)
{
    EXPECT_TRUE(murosimproc::util::image::findContourPoints(fixtures::makeImage({"   ", "   "})).empty());
    EXPECT_TRUE(murosimproc::util::image::findContourPoints(fixtures::makeImage({"###", "###"})).empty());
}

TEST(UtilImageContour, ObstacleZonesCountAsWalls)
{
    const Image walls = fixtures::makeImage({"     ", "  #  ", "     "});
    const Image zones = fixtures::makeImage({"     ", "  .  ", "     "});

    EXPECT_EQ(murosimproc::util::image::findContourPoints(zones).size(),
              murosimproc::util::image::findContourPoints(walls).size());
}

TEST(UtilImageContour, ContourLinesConnectConsecutivePoints)
{
    const Image image = fixtures::makeImage({
        "     ",
        " ##  ",
        "     ",
    });

    const std::vector<murosim::geometry::Line> lines = murosimproc::util::image::findContourLines(image);

    EXPECT_EQ(lines.size(), static_cast<size_t>(4)); // a 2 x 1 block has four sides
    const std::vector<std::vector<Eigen::Vector2f>> loops = fixtures::closedLoops(lines);
    ASSERT_EQ(loops.size(), static_cast<size_t>(1));
    EXPECT_NEAR(std::abs(fixtures::signedArea(loops.front())), 2.0f, EPSILON);
}

// ============================================================================
// Stream Operator Tests
// ============================================================================

TEST(UtilImageStream, PrintsFramedAsciiArt)
{
    Image image = fixtures::makeImage({" #.", "#  "});
    image(1, 1) = 7; // an unknown value

    std::ostringstream stream;
    stream << image;

    EXPECT_EQ(stream.str(), "+---+\n| #.|\n|#? |\n+---+\n");
}
