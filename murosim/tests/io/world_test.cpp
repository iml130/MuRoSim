// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/io/world.hpp"
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using murosim::geometry::Circle;
using murosim::geometry::Line;
using murosim::io::DataContainer;
using murosim::navigation::Node;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class WorldFileTest : public ::testing::Test
{
protected:
    std::filesystem::path temp_dir;

    // Unique per test process: ctest -j runs every test as its own process,
    // and several suites may run on one machine at the same time
    void SetUp() override
    {
        std::random_device random_device;
        const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
        temp_dir = std::filesystem::temp_directory_path() /
                   ("murosim_world_test_" + name + "_" + std::to_string(random_device()));
        std::filesystem::create_directories(temp_dir);
    }

    void TearDown() override
    {
        std::filesystem::remove_all(temp_dir);
    }

    std::filesystem::path createTempFile(const std::string &name, const std::string &content)
    {
        const std::filesystem::path path = temp_dir / name;
        std::ofstream file(path);
        file << content;
        return path;
    }
};

// ============================================================================
// Extension Tests
// ============================================================================

TEST(WorldFileExtension, ValidExtension)
{
    EXPECT_TRUE(murosim::io::world::isWorldFile("test.world"));
    EXPECT_TRUE(murosim::io::world::isWorldFile("/path/to/map.world"));
    EXPECT_TRUE(murosim::io::world::isWorldFile("./relative/path.world"));
}

TEST(WorldFileExtension, InvalidExtension)
{
    EXPECT_FALSE(murosim::io::world::isWorldFile("test.txt"));
    EXPECT_FALSE(murosim::io::world::isWorldFile("test.world.bak"));
    EXPECT_FALSE(murosim::io::world::isWorldFile("test"));
    EXPECT_FALSE(murosim::io::world::isWorldFile("world"));
    EXPECT_FALSE(murosim::io::world::isWorldFile(".world"));
}

// ============================================================================
// Load Valid File Tests
// ============================================================================

TEST_F(WorldFileTest, LoadValidFileWithAllElements)
{
    const std::string content =
        "# Test world file\n"
        "c 1.0 2.0 0.5\n"
        "l 0.0 0.0 1.0 1.0\n"
        "n 5.0 6.0 0.3 45.0 10.0 1 0\n";

    const std::filesystem::path path = createTempFile("valid.world", content);
    const DataContainer data = murosim::io::world::loadWorldFile(path);

    ASSERT_EQ(data.getCircles().size(), static_cast<size_t>(1));
    ASSERT_EQ(data.getLines().size(), static_cast<size_t>(1));
    ASSERT_EQ(data.getNodes().size(), static_cast<size_t>(1));

    EXPECT_FLOAT_EQ(data.getCircles()[0].getX(), 1.0f);
    EXPECT_FLOAT_EQ(data.getCircles()[0].getY(), 2.0f);
    EXPECT_FLOAT_EQ(data.getCircles()[0].getRadius(), 0.5f);

    EXPECT_FLOAT_EQ(data.getLines()[0].getPoint1X(), 0.0f);
    EXPECT_FLOAT_EQ(data.getLines()[0].getPoint1Y(), 0.0f);
    EXPECT_FLOAT_EQ(data.getLines()[0].getPoint2X(), 1.0f);
    EXPECT_FLOAT_EQ(data.getLines()[0].getPoint2Y(), 1.0f);

    EXPECT_FLOAT_EQ(data.getNodes()[0].getX(), 5.0f);
    EXPECT_FLOAT_EQ(data.getNodes()[0].getY(), 6.0f);
    EXPECT_FLOAT_EQ(data.getNodes()[0].getRadius(), 0.3f);
    EXPECT_EQ(data.getNodes()[0].getId(), static_cast<size_t>(1));
    EXPECT_EQ(data.getNodes()[0].getGroup(), static_cast<size_t>(0));
}

TEST_F(WorldFileTest, LoadValidFileWithMultipleElements)
{
    const std::string content =
        "c 1.0 2.0 0.5\n"
        "c 3.0 4.0 1.0\n"
        "l 0.0 0.0 1.0 1.0\n"
        "l 2.0 2.0 3.0 3.0\n"
        "l 4.0 4.0 5.0 5.0\n"
        "n 5.0 6.0 0.3 0.0 0.0 1 0\n"
        "n 7.0 8.0 0.4 0.0 0.0 2 1\n";

    const std::filesystem::path path = createTempFile("multiple.world", content);
    const DataContainer data = murosim::io::world::loadWorldFile(path);

    EXPECT_EQ(data.getCircles().size(), static_cast<size_t>(2));
    EXPECT_EQ(data.getLines().size(), static_cast<size_t>(3));
    EXPECT_EQ(data.getNodes().size(), static_cast<size_t>(2));
}

TEST_F(WorldFileTest, LoadValidFileWithComments)
{
    const std::string content =
        "# This is a comment\n"
        "c 1.0 2.0 0.5\n"
        "# Another comment\n"
        "l 0.0 0.0 1.0 1.0\n"
        "# nodes -> n x y r start_angle start_bias id group\n"
        "n 5.0 6.0 0.3 0.0 0.0 1 0\n";

    const std::filesystem::path path = createTempFile("comments.world", content);
    const DataContainer data = murosim::io::world::loadWorldFile(path);

    EXPECT_EQ(data.getCircles().size(), static_cast<size_t>(1));
    EXPECT_EQ(data.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(data.getNodes().size(), static_cast<size_t>(1));
}

TEST_F(WorldFileTest, LoadValidFileWithEmptyLines)
{
    const std::string content =
        "c 1.0 2.0 0.5\n"
        "\n"
        "l 0.0 0.0 1.0 1.0\n"
        "\n"
        "\n"
        "n 5.0 6.0 0.3 0.0 0.0 1 0\n";

    const std::filesystem::path path = createTempFile("empty_lines.world", content);
    const DataContainer data = murosim::io::world::loadWorldFile(path);

    EXPECT_EQ(data.getCircles().size(), static_cast<size_t>(1));
    EXPECT_EQ(data.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(data.getNodes().size(), static_cast<size_t>(1));
}

// Regression: lines containing only spaces or tabs were treated as an unknown
// element type and rejected the whole file.
TEST_F(WorldFileTest, LoadValidFileWithWhitespaceOnlyLines)
{
    const std::string content =
        "# Test world file\n"
        "c 1.0 2.0 0.5\n"
        "   \n"
        "l 0.0 0.0 1.0 1.0\n"
        "\t\n"
        " \t \n"
        "n 5.0 6.0 0.3 45.0 10.0 1 0\n"
        " \n";

    const std::filesystem::path path = createTempFile("whitespace.world", content);
    const DataContainer data = murosim::io::world::loadWorldFile(path);

    EXPECT_EQ(data.getCircles().size(), static_cast<size_t>(1));
    EXPECT_EQ(data.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(data.getNodes().size(), static_cast<size_t>(1));
}

TEST_F(WorldFileTest, LoadFileNameBecomesContainerName)
{
    const std::filesystem::path path = createTempFile("mymap.world", "c 1.0 2.0 0.5\n");
    const DataContainer data = murosim::io::world::loadWorldFile(path);

    EXPECT_EQ(data.getName(), "mymap");
}

// ============================================================================
// Load Error Tests
// ============================================================================

TEST_F(WorldFileTest, LoadNonWorldExtension)
{
    const std::filesystem::path path = createTempFile("test.txt", "c 1.0 2.0 0.5\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadNonExistentFile)
{
    const std::filesystem::path path = temp_dir / "nonexistent.world";

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadEmptyFile)
{
    const std::filesystem::path path = createTempFile("empty.world", "");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadFileOnlyComments)
{
    const std::filesystem::path path = createTempFile("only_comments.world",
                                                     "# This is a comment\n# Another comment\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadNegativeNodeId)
{
    const std::filesystem::path path = createTempFile("negative_id.world", "n 1.0 2.0 0.5 0.0 0.0 -1 0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadNegativeNodeGroup)
{
    const std::filesystem::path path = createTempFile("negative_group.world", "n 1.0 2.0 0.5 0.0 0.0 1 -1\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadNegativeNodeRadius)
{
    const std::filesystem::path path = createTempFile("negative_node_radius.world", "n 1.0 2.0 -0.5 0.0 0.0 1 0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadZeroNodeRadius)
{
    const std::filesystem::path path = createTempFile("zero_node_radius.world", "n 1.0 2.0 0.0 0.0 0.0 1 0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadNegativeCircleRadius)
{
    const std::filesystem::path path = createTempFile("negative_circle_radius.world", "c 1.0 2.0 -0.5\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadZeroCircleRadius)
{
    const std::filesystem::path path = createTempFile("zero_circle_radius.world", "c 1.0 2.0 0.0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadZeroLengthLine)
{
    const std::filesystem::path path = createTempFile("zero_length_line.world", "l 1.0 2.0 1.0 2.0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadUnknownType)
{
    const std::filesystem::path path = createTempFile("unknown_type.world", "x 1.0 2.0 3.0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadIncompleteCircle)
{
    const std::filesystem::path path = createTempFile("incomplete_circle.world", "c 1.0 2.0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadIncompleteLine)
{
    const std::filesystem::path path = createTempFile("incomplete_line.world", "l 1.0 2.0 3.0\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadIncompleteNode)
{
    const std::filesystem::path path = createTempFile("incomplete_node.world", "n 1.0 2.0 0.5 0.0 0.0 1\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadExtraDataCircle)
{
    const std::filesystem::path path = createTempFile("extra_circle.world", "c 1.0 2.0 0.5 extra\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadExtraDataLine)
{
    const std::filesystem::path path = createTempFile("extra_line.world", "l 1.0 2.0 3.0 4.0 extra\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadExtraDataNode)
{
    const std::filesystem::path path = createTempFile("extra_node.world", "n 1.0 2.0 0.5 0.0 0.0 1 0 extra\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadInvalidFloatCircle)
{
    const std::filesystem::path path = createTempFile("invalid_float_circle.world", "c abc 2.0 0.5\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadNaNCircle)
{
    const std::filesystem::path path = createTempFile("nan_circle.world", "c nan 2.0 0.5\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

TEST_F(WorldFileTest, LoadInfCircle)
{
    const std::filesystem::path path = createTempFile("inf_circle.world", "c inf 2.0 0.5\n");

    EXPECT_THROW(murosim::io::world::loadWorldFile(path), std::runtime_error);
}

// ============================================================================
// Save And Load Tests
// ============================================================================

TEST_F(WorldFileTest, SaveAndLoadRoundTrip)
{
    const std::vector<Line> lines = {Line(0.0f, 0.0f, 1.0f, 1.0f), Line(2.0f, 2.0f, 3.0f, 3.0f)};
    const std::vector<Circle> circles = {Circle(1.0f, 2.0f, 0.5f), Circle(3.0f, 4.0f, 1.0f)};
    const std::vector<Node> nodes = {Node(Circle(5.0f, 6.0f, 0.3f), 0.0f, 0.0f, 1, 0),
                                     Node(Circle(7.0f, 8.0f, 0.4f), 0.0f, 0.0f, 2, 1)};

    const std::filesystem::path path = temp_dir / "roundtrip.world";
    murosim::io::world::saveWorldFile(path, lines, circles, nodes);
    const DataContainer loaded = murosim::io::world::loadWorldFile(path);

    ASSERT_EQ(loaded.getLines().size(), lines.size());
    ASSERT_EQ(loaded.getCircles().size(), circles.size());
    ASSERT_EQ(loaded.getNodes().size(), nodes.size());

    for (size_t i = 0; i < lines.size(); ++i)
    {
        EXPECT_FLOAT_EQ(loaded.getLines()[i].getPoint1X(), lines[i].getPoint1X()) << "line " << i;
        EXPECT_FLOAT_EQ(loaded.getLines()[i].getPoint1Y(), lines[i].getPoint1Y()) << "line " << i;
        EXPECT_FLOAT_EQ(loaded.getLines()[i].getPoint2X(), lines[i].getPoint2X()) << "line " << i;
        EXPECT_FLOAT_EQ(loaded.getLines()[i].getPoint2Y(), lines[i].getPoint2Y()) << "line " << i;
    }

    for (size_t i = 0; i < circles.size(); ++i)
    {
        EXPECT_FLOAT_EQ(loaded.getCircles()[i].getX(), circles[i].getX()) << "circle " << i;
        EXPECT_FLOAT_EQ(loaded.getCircles()[i].getY(), circles[i].getY()) << "circle " << i;
        EXPECT_FLOAT_EQ(loaded.getCircles()[i].getRadius(), circles[i].getRadius()) << "circle " << i;
    }

    for (size_t i = 0; i < nodes.size(); ++i)
    {
        EXPECT_FLOAT_EQ(loaded.getNodes()[i].getX(), nodes[i].getX()) << "node " << i;
        EXPECT_FLOAT_EQ(loaded.getNodes()[i].getY(), nodes[i].getY()) << "node " << i;
        EXPECT_FLOAT_EQ(loaded.getNodes()[i].getRadius(), nodes[i].getRadius()) << "node " << i;
        EXPECT_EQ(loaded.getNodes()[i].getId(), nodes[i].getId()) << "node " << i;
        EXPECT_EQ(loaded.getNodes()[i].getGroup(), nodes[i].getGroup()) << "node " << i;
    }
}

// The writers must serialize floats losslessly: a fixed decimal precision
// would quantize coordinates, shifting geometry on every save/load cycle and
// collapsing sub-quantum lines into "zero length" entries the loader rejects.
// Hence exact comparison here, not EXPECT_FLOAT_EQ.
TEST_F(WorldFileTest, SaveAndLoadRoundTripIsLossless)
{
    const std::vector<Line> lines = {Line(1.23451f, 0.0f, 1.23492f, 0.0f),
                                     Line(0.333333f, 0.666667f, 10.000001f, -7.123456f)};
    const std::vector<Circle> circles = {Circle(0.000123f, -0.000456f, 0.001f)};
    const std::vector<Node> nodes;

    const std::filesystem::path path = temp_dir / "roundtrip_lossless.world";
    murosim::io::world::saveWorldFile(path, lines, circles, nodes);
    const DataContainer loaded = murosim::io::world::loadWorldFile(path);

    ASSERT_EQ(loaded.getLines().size(), lines.size());
    ASSERT_EQ(loaded.getCircles().size(), circles.size());

    for (size_t i = 0; i < lines.size(); ++i)
    {
        EXPECT_EQ(loaded.getLines()[i].getPoint1X(), lines[i].getPoint1X()) << "line " << i;
        EXPECT_EQ(loaded.getLines()[i].getPoint1Y(), lines[i].getPoint1Y()) << "line " << i;
        EXPECT_EQ(loaded.getLines()[i].getPoint2X(), lines[i].getPoint2X()) << "line " << i;
        EXPECT_EQ(loaded.getLines()[i].getPoint2Y(), lines[i].getPoint2Y()) << "line " << i;
    }

    EXPECT_EQ(loaded.getCircles()[0].getX(), circles[0].getX());
    EXPECT_EQ(loaded.getCircles()[0].getY(), circles[0].getY());
    EXPECT_EQ(loaded.getCircles()[0].getRadius(), circles[0].getRadius());
}

TEST_F(WorldFileTest, SaveAddsWorldExtension)
{
    const std::vector<Line> lines;
    const std::vector<Circle> circles = {Circle(1.0f, 2.0f, 0.5f)};
    const std::vector<Node> nodes;

    murosim::io::world::saveWorldFile(temp_dir / "noext", lines, circles, nodes);

    EXPECT_TRUE(std::filesystem::exists(temp_dir / "noext.world"));
}

TEST_F(WorldFileTest, SaveWithDataContainer)
{
    const std::vector<Line> lines = {Line(0.0f, 0.0f, 1.0f, 1.0f)};
    const std::vector<Circle> circles = {Circle(1.0f, 2.0f, 0.5f)};
    const std::vector<Node> nodes = {Node(Circle(5.0f, 6.0f, 0.3f), 0.0f, 0.0f, 1, 0)};
    const DataContainer data(lines, circles, nodes, "testmap");

    const std::filesystem::path path = temp_dir / "container.world";
    murosim::io::world::saveWorldFile(path, data);
    const DataContainer loaded = murosim::io::world::loadWorldFile(path);

    EXPECT_EQ(loaded.getLines().size(), static_cast<size_t>(1));
    EXPECT_EQ(loaded.getCircles().size(), static_cast<size_t>(1));
    EXPECT_EQ(loaded.getNodes().size(), static_cast<size_t>(1));
}
