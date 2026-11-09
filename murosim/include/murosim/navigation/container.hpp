// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_NAVIGATION_CONTAINER_HPP_
#define MUROSIM_NAVIGATION_CONTAINER_HPP_

#include "node.hpp"

#include <vector>


namespace murosim::navigation
{

/// An owning, grouped collection of murosim::navigation::Node%s used as the
/// set of possible spawn/waypoint/goal locations for a simulation. On
/// construction, nodes are sorted by id and their (possibly sparse, arbitrary)
/// group ids are validated and compacted into contiguous bucket indices: group
/// 0 is the default/ungrouped bucket (any size), while any other group must
/// contain at least 2 nodes (a malformed cluster otherwise throws
/// std::invalid_argument). Node::getGroup() reflects the compacted bucket
/// index after construction, not the original id passed in.
class Container
{
private:
    std::vector<Node> nodes_;
    std::vector<std::vector<size_t>> indices_group_;

    // look up table for group and item
    std::vector<size_t> lut_group_;

    // Validates groups (a non-default group needs >= 2 nodes, else throws
    // std::invalid_argument), compacts group ids to bucket indices, and fills
    // indices_group_. Group 0 is the default/ungrouped bucket of any size.
    void groupNodes();
public:
    /// Constructs an empty container.
    Container();
    /// Constructs a container from a copy of nodes (sorted and grouped).
    Container(const std::vector<Node> &nodes);
    /// Constructs a container by moving in nodes (sorted and grouped).
    Container(std::vector<Node> && nodes);

    /// Returns, per compacted group bucket, the node indices (into getNodes()) belonging to that group.
    const std::vector<std::vector<size_t>>& getGroupIndices() const;

    /// Returns the center point of every node, in the same order as getNodes().
    std::vector<Eigen::Vector2f> getNodePoints() const;

    /// Maps an original group id to its compacted bucket index; unknown ids fall back to bucket 0.
    size_t lookUpGroup(const size_t group) const;

    /// Returns all nodes, sorted by id.
    const std::vector<Node>& getNodes() const;
    /// Returns the node at index. @throws std::out_of_range if index >= getNodeSize().
    const Node& getNodeAt(const size_t index) const;
    /// Returns the number of nodes.
    size_t getNodeSize() const;
    /// Returns whether the container holds no nodes.
    bool empty() const;
};

} // namespace murosim::navigation

#endif // MUROSIM_NAVIGATION_CONTAINER_HPP_
