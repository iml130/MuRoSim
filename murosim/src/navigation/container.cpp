// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/navigation/container.hpp"

#include <algorithm>
#include <map>
#include <sstream>
#include <stdexcept>

namespace murosim::navigation
{

Container::Container()
{

}

Container::Container(const std::vector<Node> &nodes): Container(std::vector<Node>(nodes))
{

}

Container::Container(std::vector<Node> &&nodes): nodes_(std::move(nodes))
{
    std::sort(nodes_.begin(), nodes_.end());

    groupNodes();
}

void Container::groupNodes()
{
    // 1. Count the nodes per original group id. std::map keeps the ids ordered,
    //    so the compact bucket indices below are deterministic.
    std::map<size_t, size_t> group_counts;
    for (const Node &node : nodes_)
    {
        group_counts[node.getGroup()]++;
    }

    // 2. Validate and build the lookup table (compact index -> original group id).
    //    Group 0 is the default/ungrouped bucket and may hold any number of
    //    nodes. A real cluster (group id >= 1) must contain at least 2 nodes;
    //    a malformed cluster is a caller/generator error -> invalid_argument.
    lut_group_.clear();
    for (const auto &[group, count] : group_counts)
    {
        if (group != 0 && count < 2)
        {
            std::stringstream ss;
            ss << "[navigation::Container]: Group " << group << " has only " << count
               << " node(s); a non-default group must contain at least 2 nodes "
               << "(assign ungrouped nodes to group 0).";
            throw std::invalid_argument(ss.str());
        }

        lut_group_.push_back(group);
    }

    // 3. Remap every node's group id to its compact bucket index and fill the
    //    per-group index buckets. max(.., 1) keeps a single (empty) bucket for an
    //    empty container so consumers can index group 0 unconditionally.
    indices_group_.assign(std::max<size_t>(lut_group_.size(), 1), {});
    for (size_t i = 0; i < nodes_.size(); ++i)
    {
        const size_t index = lookUpGroup(nodes_[i].getGroup());
        nodes_[i].setGroup(index);
        indices_group_[index].emplace_back(i);
    }
}

size_t Container::lookUpGroup(const size_t group) const
{
    for (size_t i = 0; i < lut_group_.size(); ++i)
    {
        if (lut_group_[i] == group)
        {
            return i;
        }
    }

    // Unknown group ids fall back to the default bucket 0.
    return 0;
}

const std::vector<std::vector<size_t>>& Container::getGroupIndices() const
{
    return indices_group_;
}

const std::vector<Node>& Container::getNodes() const
{
    return nodes_;
}

const Node& Container::getNodeAt(const size_t index) const
{
    if (index >= nodes_.size())
    {
        throw std::out_of_range("[navigation::Container::getNodeAt]: node index out of range.");
    }
    return nodes_[index];
}

size_t Container::getNodeSize() const
{
    return nodes_.size();
}

bool Container::empty() const
{
    return nodes_.empty();
}

std::vector<Eigen::Vector2f> Container::getNodePoints() const
{
    std::vector<Eigen::Vector2f> points;

    points.reserve(nodes_.size());

    for (size_t i = 0; i < nodes_.size(); i++)
    {
        points.emplace_back(nodes_[i].getCircle().getPoint());
    }

    return points;
}

} // namespace murosim::navigation
