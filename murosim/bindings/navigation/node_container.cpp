// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/navigation/container.hpp"
#include "murosim/navigation/node.hpp"

#include <sstream>

namespace murosimpy {
namespace {

murosim::navigation::Container createNodeContainer(const std::vector<murosim::navigation::Node> &nodes)
{
    return murosim::navigation::Container(nodes);
}

// Helper to collect nodes from container (returns copy of internal vector)
std::vector<murosim::navigation::Node> collectNodes(const murosim::navigation::Container &container) {
    return container.getNodes();
}

} // anonymous namespace

void initNavigationNodeContainer(pybind11::module &m)
{
    auto pyNodeContainer = pybind11::class_<murosim::navigation::Container>(m, "NodeContainer",
        R"doc(An owning, grouped collection of Nodes used as the set of possible
spawn/waypoint/goal locations for a simulation.

On construction, nodes are sorted by id and their (possibly sparse,
arbitrary) group ids are validated and compacted into contiguous bucket
indices: group 0 is the default/ungrouped bucket (any size), while any
other group must contain at least 2 nodes. Node.group reflects the
compacted bucket index after construction, not the original id passed in.)doc");

    // === NodeContainer ===
    pyNodeContainer.def(pybind11::init(&createNodeContainer),
        "Create a node container from a list of nodes (copied, then sorted by id and grouped).\n\n"
        "Args:\n"
        "    nodes: List of Node objects\n\n"
        "Raises:\n"
        "    ValueError: if a non-default group (id != 0) contains fewer than 2 nodes.",
        pybind11::arg("nodes"));

    // Properties
    pyNodeContainer.def_property_readonly("size", [](const murosim::navigation::Container &c) {
        return c.getNodes().size();
    }, "Number of nodes in the container.");
    pyNodeContainer.def_property_readonly("empty", &murosim::navigation::Container::empty,
        "Whether the container holds no nodes.");
    pyNodeContainer.def_property_readonly("nodes", &collectNodes,
        "List of all nodes in the container, sorted by id.");

    // Indexed access
    pyNodeContainer.def("node_at", [](const murosim::navigation::Container &c, size_t index) {
        const auto& nodes = c.getNodes();
        checkIndex(index, nodes.size(), "Node");
        return nodes[index];
    }, "Get a copy of the node at the specified index (into the id-sorted node list).\n\n"
       "Args:\n"
       "    index: Zero-based index into the container's nodes.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));

    // Python special methods
    pyNodeContainer.def("__repr__", [](const murosim::navigation::Container &self) {
        std::ostringstream oss;
        oss << "<NodeContainer size=" << self.getNodes().size() << ">";
        return oss.str();
    }, "Return a debug string representation.");
    pyNodeContainer.def("__len__", [](const murosim::navigation::Container &self) {
        return self.getNodes().size();
    }, "Return the number of nodes.");
    pyNodeContainer.def("__getitem__", [](const murosim::navigation::Container &self, size_t index) {
        const auto& nodes = self.getNodes();
        checkIndex(index, nodes.size(), "Node");
        return nodes[index];
    }, "Get a copy of the node at the specified index (into the id-sorted node list).\n\n"
       "Args:\n"
       "    index: Zero-based index into the container's nodes.\n\n"
       "Raises:\n"
       "    IndexError: if index is out of range.",
       pybind11::arg("index"));
    pyNodeContainer.def("__iter__", [](const murosim::navigation::Container &self) {
        // Iterate the container's own node vector (stable, kept alive by
        // keep_alive below). Do NOT iterate a temporary copy: its begin()/end()
        // would dangle once the temporary is destroyed. Yield copies so
        // iteration matches node_at/__getitem__ - the default
        // reference_internal policy would hand out mutable aliases of the
        // container's internal nodes.
        const std::vector<murosim::navigation::Node> &nodes = self.getNodes();
        return pybind11::make_iterator<pybind11::return_value_policy::copy>(nodes.begin(), nodes.end());
    }, pybind11::keep_alive<0, 1>(),
       "Iterate over independent copies of all nodes, in id-sorted order. "
       "The returned iterator keeps this container alive for its lifetime.");
    pyNodeContainer.def("__copy__", [](const murosim::navigation::Container &self) {
        return murosim::navigation::Container(self);
    }, "Support copy.copy(); returns an independent copy.");
    pyNodeContainer.def("__deepcopy__", [](const murosim::navigation::Container &self, pybind11::dict) {
        return murosim::navigation::Container(self);
    }, "Support copy.deepcopy(); returns an independent copy.",
       pybind11::arg("memo"));
}

} // namespace murosimpy
