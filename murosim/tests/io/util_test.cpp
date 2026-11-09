// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/io/util.hpp"

// ============================================================================
// Split String With Char Separator Tests
// ============================================================================

TEST(UtilSplitStringChar, BasicSplit)
{
    const auto result = murosim::io::util::splitString("a,b,c", ',');
    ASSERT_EQ(result.size(), static_cast<size_t>(3));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
    EXPECT_EQ(result[2], "c");
}

TEST(UtilSplitStringChar, EmptyString)
{
    const auto result = murosim::io::util::splitString("", ',');
    EXPECT_TRUE(result.empty());
}

TEST(UtilSplitStringChar, NoSeparator)
{
    const auto result = murosim::io::util::splitString("hello", ',');
    ASSERT_EQ(result.size(), static_cast<size_t>(1));
    EXPECT_EQ(result[0], "hello");
}

TEST(UtilSplitStringChar, ConsecutiveSeparators)
{
    const auto result = murosim::io::util::splitString("a,,b", ',');
    ASSERT_EQ(result.size(), static_cast<size_t>(2));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
}

TEST(UtilSplitStringChar, LeadingSeparator)
{
    const auto result = murosim::io::util::splitString(",a,b", ',');
    ASSERT_EQ(result.size(), static_cast<size_t>(2));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
}

TEST(UtilSplitStringChar, TrailingSeparator)
{
    const auto result = murosim::io::util::splitString("a,b,", ',');
    ASSERT_EQ(result.size(), static_cast<size_t>(2));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
}

TEST(UtilSplitStringChar, OnlySeparators)
{
    const auto result = murosim::io::util::splitString(",,,", ',');
    EXPECT_TRUE(result.empty());
}

TEST(UtilSplitStringChar, SpaceSeparator)
{
    const auto result = murosim::io::util::splitString("hello world test", ' ');
    ASSERT_EQ(result.size(), static_cast<size_t>(3));
    EXPECT_EQ(result[0], "hello");
    EXPECT_EQ(result[1], "world");
    EXPECT_EQ(result[2], "test");
}

// ============================================================================
// Split String With String Separators Tests
// ============================================================================

TEST(UtilSplitStringMulti, BasicSplit)
{
    const auto result = murosim::io::util::splitString("a,b;c", ",;");
    ASSERT_EQ(result.size(), static_cast<size_t>(3));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
    EXPECT_EQ(result[2], "c");
}

TEST(UtilSplitStringMulti, EmptyString)
{
    const auto result = murosim::io::util::splitString("", ",;");
    EXPECT_TRUE(result.empty());
}

TEST(UtilSplitStringMulti, NoSeparator)
{
    const auto result = murosim::io::util::splitString("hello", ",;");
    ASSERT_EQ(result.size(), static_cast<size_t>(1));
    EXPECT_EQ(result[0], "hello");
}

TEST(UtilSplitStringMulti, MixedSeparators)
{
    const auto result = murosim::io::util::splitString("a,b;c:d", ",;:");
    ASSERT_EQ(result.size(), static_cast<size_t>(4));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
    EXPECT_EQ(result[2], "c");
    EXPECT_EQ(result[3], "d");
}

TEST(UtilSplitStringMulti, ConsecutiveMixedSeparators)
{
    const auto result = murosim::io::util::splitString("a,;b", ",;");
    ASSERT_EQ(result.size(), static_cast<size_t>(2));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
}

TEST(UtilSplitStringMulti, WhitespaceSeparators)
{
    const auto result = murosim::io::util::splitString("a b\tc\nd", " \t\n");
    ASSERT_EQ(result.size(), static_cast<size_t>(4));
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
    EXPECT_EQ(result[2], "c");
    EXPECT_EQ(result[3], "d");
}

// ============================================================================
// Get Filename Tests
// ============================================================================

TEST(UtilGetFilename, BasicFilename)
{
    const auto result = murosim::io::util::getFilename("/path/to/file.txt");
    EXPECT_EQ(result, "file");
}

TEST(UtilGetFilename, WorldFile)
{
    const auto result = murosim::io::util::getFilename("/path/to/map.world");
    EXPECT_EQ(result, "map");
}

TEST(UtilGetFilename, NoExtension)
{
    const auto result = murosim::io::util::getFilename("/path/to/file");
    EXPECT_EQ(result, "file");
}

TEST(UtilGetFilename, MultipleExtensions)
{
    const auto result = murosim::io::util::getFilename("/path/to/file.tar.gz");
    EXPECT_EQ(result, "file.tar");
}

TEST(UtilGetFilename, HiddenFile)
{
    // .hidden is treated as filename (no extension), so result is .hidden
    const auto result = murosim::io::util::getFilename("/path/to/.hidden");
    EXPECT_EQ(result, ".hidden");
}

TEST(UtilGetFilename, CurrentDirectory)
{
    const auto result = murosim::io::util::getFilename("file.txt");
    EXPECT_EQ(result, "file");
}

TEST(UtilGetFilename, RelativePath)
{
    const auto result = murosim::io::util::getFilename("./subdir/file.world");
    EXPECT_EQ(result, "file");
}
