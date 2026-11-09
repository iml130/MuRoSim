// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/graph/filter.hpp"

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cassert>

namespace murosimproc::graph::filter
{

void byEdgeChoiceRandom(Graph& graph, murosim::random::RandomGenerator& random_generator, float keep_probability)
{
    if (keep_probability < 0.0f || keep_probability > 1.0f)
    {
        throw std::invalid_argument("keep_probability must be in [0.0, 1.0]");
    }

    if (empty(graph))
    {
        return;
    }

    const size_t num_nodes = getNodeSize(graph);
    std::vector<bool> node_visited(num_nodes, false);

    // Encode edges as 64-bit: high 32 bits = min(u,v), low 32 bits = max(u,v)
    auto encode_edge = [](uint32_t u, uint32_t v) -> uint64_t {
        return (static_cast<uint64_t>(std::min(u, v)) << 32) | static_cast<uint64_t>(std::max(u, v));
    };

    std::vector<uint64_t> edges_to_remove;
    std::vector<uint64_t> edges_to_keep;
    std::vector<uint64_t> edges_candidates;

    const uint32_t index_node = static_cast<uint32_t>(random_generator() % num_nodes);
    node_visited[index_node] = true;

    for (Vertex neighbor : getNodeNeighbors(graph, static_cast<Vertex>(index_node)))
    {
        edges_candidates.push_back(encode_edge(index_node, static_cast<uint32_t>(neighbor)));
    }

    while (!edges_candidates.empty())
    {
        // Select random edge
        const size_t index_edge = random_generator() % edges_candidates.size();
        const uint64_t edge = edges_candidates[index_edge];

        // Remove edge from candidates
        std::swap(edges_candidates[index_edge], edges_candidates.back());
        edges_candidates.pop_back();

        // Extract u and v from the encoded edge
        uint32_t u = static_cast<uint32_t>(edge >> 32);
        uint32_t v = static_cast<uint32_t>(edge & 0xFFFFFFFFu);

        if (node_visited[u] && node_visited[v])
        {
            // Both nodes already visited, skip edge
            edges_to_remove.push_back(edge);
            continue;
        }

        edges_to_keep.push_back(edge);

        assert(node_visited[u] || node_visited[v]);

        if (node_visited[u])
        {
            std::swap(u, v);
        }

        for (Vertex neighbor : getNodeNeighbors(graph, static_cast<Vertex>(u)))
        {
            // Avoid adding the edge back to v
            if (static_cast<uint32_t>(neighbor) == v)
            {
                continue;
            }

            if (!node_visited[neighbor])
            {
                edges_candidates.push_back(encode_edge(u, static_cast<uint32_t>(neighbor)));
            }
        }

        node_visited[u] = true;
    }

    // Keep some cycle edges based on keep_probability
    const size_t number_to_keep = std::min(
        static_cast<size_t>(static_cast<float>(edges_to_remove.size()) * keep_probability),
        edges_to_remove.size());
    const size_t number_of_edges_to_remove = edges_to_remove.size() - number_to_keep;

    // Shuffle edges to remove
    std::shuffle(edges_to_remove.begin(), edges_to_remove.end(), random_generator);

    for (size_t i = 0; i < number_of_edges_to_remove; ++i)
    {
        const uint64_t edge = edges_to_remove[i];
        const uint32_t u = static_cast<uint32_t>(edge >> 32);
        const uint32_t v = static_cast<uint32_t>(edge & 0xFFFFFFFFu);

        removeEdge(graph, static_cast<Vertex>(u), static_cast<Vertex>(v));
    }
}

std::vector<Vertex> selectNodeVertices(const Graph& graph, const size_t min_node_count, murosim::random::RandomGenerator& random_generator)
{
    const size_t num_vertices = getNodeSize(graph);
    std::vector<Vertex> selected;
    std::vector<Vertex> junctions;

    for (size_t i = 0; i < num_vertices; ++i)
    {
        const Vertex v = static_cast<Vertex>(i);
        const size_t degree = getNodeNeighbors(graph, v).size();

        if (degree == 0)
        {
            continue;  // Isolated vertex — skip
        }

        if (degree <= 2)
        {
            selected.push_back(v);
        }
        else
        {
            junctions.push_back(v);
        }
    }

    // Fill up with junctions, lowest degree first (they block the fewest
    // routes); shuffle beforehand so ties are broken randomly
    if (selected.size() < min_node_count && !junctions.empty())
    {
        std::shuffle(junctions.begin(), junctions.end(), random_generator);
        std::stable_sort(junctions.begin(), junctions.end(),
            [&graph](const Vertex a, const Vertex b) {
                return getNodeNeighbors(graph, a).size() < getNodeNeighbors(graph, b).size();
            });

        const size_t missing = min_node_count - selected.size();
        junctions.resize(std::min(junctions.size(), missing));
        selected.insert(selected.end(), junctions.begin(), junctions.end());
    }

    return selected;
}

} // namespace murosimproc::graph::filter
