// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/navigation/container.hpp"
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
using murosim::navigation::Container;
using murosim::navigation::Node;

constexpr float EPSILON = 1e-5f;

// Group contract of the container: group 0 is the default bucket and may hold
// any number of nodes; every other group must hold at least two nodes,
// otherwise the constructor throws std::invalid_argument.

/// Node number `id` at (id, 0) with radius 1 in the given group.
Node nodeInGroup(const size_t id, const size_t group)
{
    return Node(static_cast<float>(id), 0.0f, 1.0f, 0.0f, 0.0f, id, group);
}

/// Nodes 0 and 1 in group 0, nodes 2 and 3 in group 1.
std::vector<Node> twoGroupsOfTwo()
{
    return {nodeInGroup(0, 0), nodeInGroup(1, 0), nodeInGroup(2, 1), nodeInGroup(3, 1)};
}

/// Three nodes with distinct coordinates and radii, all in group 0.
std::vector<Node> threeNodes()
{
    return {Node(1.0f, 2.0f, 0.5f, 0.0f, 0.0f, 0, 0),
            Node(3.0f, 4.0f, 0.6f, 0.0f, 0.0f, 1, 0),
            Node(5.0f, 6.0f, 0.7f, 0.0f, 0.0f, 2, 0)};
}
} // namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(ContainerConstructor, DefaultConstructor)
{
    const Container container;

    EXPECT_TRUE(container.empty());
    EXPECT_EQ(container.getNodes().size(), static_cast<size_t>(0));
}

TEST(ContainerConstructor, VectorConstructor)
{
    const std::vector<Node> nodes = threeNodes();
    const Container container(nodes);

    EXPECT_FALSE(container.empty());
    EXPECT_EQ(container.getNodes().size(), static_cast<size_t>(3));
}

TEST(ContainerConstructor, MoveConstructor)
{
    std::vector<Node> nodes = {nodeInGroup(0, 0), nodeInGroup(1, 0)};
    const Container container(std::move(nodes));

    EXPECT_FALSE(container.empty());
    EXPECT_EQ(container.getNodes().size(), static_cast<size_t>(2));
}

// ============================================================================
// Node Access Tests
// ============================================================================

TEST(ContainerAccess, GetNodes)
{
    const Container container(threeNodes());

    ASSERT_EQ(container.getNodes().size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(container.getNodes()[0].getX(), 1.0f);
    EXPECT_FLOAT_EQ(container.getNodes()[0].getY(), 2.0f);
    EXPECT_FLOAT_EQ(container.getNodes()[1].getX(), 3.0f);
    EXPECT_FLOAT_EQ(container.getNodes()[1].getY(), 4.0f);
    EXPECT_FLOAT_EQ(container.getNodes()[2].getX(), 5.0f);
    EXPECT_FLOAT_EQ(container.getNodes()[2].getY(), 6.0f);
}

TEST(ContainerAccess, Empty)
{
    const Container empty_container;
    EXPECT_TRUE(empty_container.empty());

    const Container non_empty_container({nodeInGroup(0, 0), nodeInGroup(1, 0)});
    EXPECT_FALSE(non_empty_container.empty());
}

TEST(ContainerAccess, GetNodePoints)
{
    const Container container(threeNodes());

    const std::vector<Eigen::Vector2f> points = container.getNodePoints();

    ASSERT_EQ(points.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(points[0].x(), 1.0f);
    EXPECT_FLOAT_EQ(points[0].y(), 2.0f);
    EXPECT_FLOAT_EQ(points[1].x(), 3.0f);
    EXPECT_FLOAT_EQ(points[1].y(), 4.0f);
    EXPECT_FLOAT_EQ(points[2].x(), 5.0f);
    EXPECT_FLOAT_EQ(points[2].y(), 6.0f);
}

TEST(ContainerAccess, GetNodePointsEmpty)
{
    const Container container;

    EXPECT_EQ(container.getNodePoints().size(), static_cast<size_t>(0));
}

// ============================================================================
// Indexed Access Tests
// ============================================================================

TEST(ContainerIndexedAccess, GetNodeSize)
{
    const Container container(threeNodes());

    EXPECT_EQ(container.getNodeSize(), static_cast<size_t>(3));
}

TEST(ContainerIndexedAccess, GetNodeSizeEmpty)
{
    const Container container;

    EXPECT_EQ(container.getNodeSize(), static_cast<size_t>(0));
}

TEST(ContainerIndexedAccess, GetNodeAt)
{
    const Container container(threeNodes());

    const Node &node0 = container.getNodeAt(0);
    EXPECT_NEAR(node0.getPoint().x(), 1.0f, EPSILON);
    EXPECT_NEAR(node0.getPoint().y(), 2.0f, EPSILON);
    EXPECT_NEAR(node0.getRadius(), 0.5f, EPSILON);
    EXPECT_EQ(node0.getId(), static_cast<size_t>(0));

    const Node &node1 = container.getNodeAt(1);
    EXPECT_NEAR(node1.getPoint().x(), 3.0f, EPSILON);
    EXPECT_NEAR(node1.getPoint().y(), 4.0f, EPSILON);
    EXPECT_NEAR(node1.getRadius(), 0.6f, EPSILON);
    EXPECT_EQ(node1.getId(), static_cast<size_t>(1));

    const Node &node2 = container.getNodeAt(2);
    EXPECT_NEAR(node2.getPoint().x(), 5.0f, EPSILON);
    EXPECT_NEAR(node2.getPoint().y(), 6.0f, EPSILON);
    EXPECT_NEAR(node2.getRadius(), 0.7f, EPSILON);
    EXPECT_EQ(node2.getId(), static_cast<size_t>(2));
}

TEST(ContainerIndexedAccess, GetNodeAtConsistentWithGetNodes)
{
    const Container container(threeNodes());

    ASSERT_EQ(container.getNodeSize(), static_cast<size_t>(3));
    for (size_t i = 0; i < container.getNodeSize(); i++)
    {
        const Node &node_at = container.getNodeAt(i);
        const Node &node_vector = container.getNodes()[i];

        EXPECT_NEAR(node_at.getPoint().x(), node_vector.getPoint().x(), EPSILON) << "node " << i;
        EXPECT_NEAR(node_at.getPoint().y(), node_vector.getPoint().y(), EPSILON) << "node " << i;
        EXPECT_NEAR(node_at.getRadius(), node_vector.getRadius(), EPSILON) << "node " << i;
        EXPECT_EQ(node_at.getId(), node_vector.getId()) << "node " << i;
    }
}

// ============================================================================
// Group Tests
// ============================================================================

TEST(ContainerGroup, GroupIndicesSize)
{
    const Container container(twoGroupsOfTwo());

    EXPECT_EQ(container.getGroupIndices().size(), static_cast<size_t>(2));
}

TEST(ContainerGroup, GroupIndicesHoldTheGroupMembers)
{
    const Container container(twoGroupsOfTwo());

    const std::vector<std::vector<size_t>> &groups = container.getGroupIndices();
    ASSERT_EQ(groups.size(), static_cast<size_t>(2));
    ASSERT_EQ(groups[0].size(), static_cast<size_t>(2));
    ASSERT_EQ(groups[1].size(), static_cast<size_t>(2));

    // Every index listed for a group refers to a node of that group
    for (size_t group = 0; group < groups.size(); group++)
    {
        for (const size_t index : groups[group])
        {
            EXPECT_EQ(container.getNodes()[index].getGroup(), group) << "index " << index;
        }
    }
}

TEST(ContainerGroup, LookUpGroupMapsGroupIdToIndex)
{
    // Group ids need not be consecutive: the container renumbers them to the
    // internal group indices, and lookUpGroup translates an original id
    const Container container({nodeInGroup(0, 5), nodeInGroup(1, 5), nodeInGroup(2, 10), nodeInGroup(3, 10)});

    const size_t index_5 = container.lookUpGroup(5);
    const size_t index_10 = container.lookUpGroup(10);

    ASSERT_LT(index_5, container.getGroupIndices().size());
    ASSERT_LT(index_10, container.getGroupIndices().size());
    EXPECT_NE(index_5, index_10);

    // The looked-up group holds exactly the nodes that were given that id
    // (node ids 0 and 1 for group 5, 2 and 3 for group 10), and those nodes
    // now carry the internal index as their group
    const std::vector<size_t> &members_5 = container.getGroupIndices()[index_5];
    const std::vector<size_t> &members_10 = container.getGroupIndices()[index_10];
    ASSERT_EQ(members_5.size(), static_cast<size_t>(2));
    ASSERT_EQ(members_10.size(), static_cast<size_t>(2));

    for (const size_t index : members_5)
    {
        EXPECT_LE(container.getNodes()[index].getId(), static_cast<size_t>(1)) << "index " << index;
        EXPECT_EQ(container.getNodes()[index].getGroup(), index_5) << "index " << index;
    }
    for (const size_t index : members_10)
    {
        EXPECT_GE(container.getNodes()[index].getId(), static_cast<size_t>(2)) << "index " << index;
        EXPECT_EQ(container.getNodes()[index].getGroup(), index_10) << "index " << index;
    }
}

TEST(ContainerGroup, TwoNodesAreTheMinimumGroup)
{
    const Container container({nodeInGroup(0, 0), nodeInGroup(1, 0)});

    EXPECT_FALSE(container.empty());
    EXPECT_EQ(container.getNodes().size(), static_cast<size_t>(2));
    EXPECT_EQ(container.getGroupIndices().size(), static_cast<size_t>(1));
}

TEST(ContainerGroup, AllNodesInOneGroup)
{
    const Container container({nodeInGroup(0, 0), nodeInGroup(1, 0), nodeInGroup(2, 0)});

    ASSERT_EQ(container.getGroupIndices().size(), static_cast<size_t>(1));
    EXPECT_EQ(container.getGroupIndices()[0].size(), static_cast<size_t>(3));
}

TEST(ContainerGroup, ThreeGroupsOfTwo)
{
    const Container container({nodeInGroup(0, 0), nodeInGroup(1, 0), nodeInGroup(2, 1),
                               nodeInGroup(3, 1), nodeInGroup(4, 2), nodeInGroup(5, 2)});

    ASSERT_EQ(container.getGroupIndices().size(), static_cast<size_t>(3));
    for (size_t i = 0; i < container.getGroupIndices().size(); i++)
    {
        EXPECT_EQ(container.getGroupIndices()[i].size(), static_cast<size_t>(2)) << "group " << i;
    }
}

// ============================================================================
// Group Contract Tests
// ============================================================================

// Regression: a container whose every group had fewer than 2 nodes used to
// write out of bounds into an empty group-index vector. The contract is now
// explicit: group 0 may hold any number of nodes, a non-default group must
// hold at least two, otherwise the constructor throws.
TEST(ContainerGroupContract, SingleNodeInDefaultGroupIsValid)
{
    const Container container({nodeInGroup(0, 0)});

    EXPECT_EQ(container.getNodes().size(), static_cast<size_t>(1));
    ASSERT_GE(container.getGroupIndices().size(), static_cast<size_t>(1));
    EXPECT_EQ(container.getGroupIndices()[0].size(), static_cast<size_t>(1));
    EXPECT_EQ(container.getNodes()[0].getGroup(), static_cast<size_t>(0));
}

TEST(ContainerGroupContract, ManyNodesInDefaultGroupIsValid)
{
    const std::vector<Node> nodes = {nodeInGroup(0, 0), nodeInGroup(1, 0), nodeInGroup(2, 0)};

    EXPECT_NO_THROW(Container{nodes});
}

TEST(ContainerGroupContract, NonDefaultGroupWithTwoNodesIsValid)
{
    const std::vector<Node> nodes = {nodeInGroup(0, 3), nodeInGroup(1, 3)};

    EXPECT_NO_THROW(Container{nodes});
}

TEST(ContainerGroupContract, LoneNodeInNonDefaultGroupThrows)
{
    // A valid default-group pair plus a single node in group 1
    const std::vector<Node> nodes = {nodeInGroup(0, 0), nodeInGroup(1, 0), nodeInGroup(2, 1)};

    EXPECT_THROW(Container{nodes}, std::invalid_argument);
}

TEST(ContainerGroupContract, AllNonDefaultGroupsTooSmallThrow)
{
    // Three nodes in three different non-default groups
    const std::vector<Node> nodes = {nodeInGroup(0, 5), nodeInGroup(1, 6), nodeInGroup(2, 7)};

    EXPECT_THROW(Container{nodes}, std::invalid_argument);
}
