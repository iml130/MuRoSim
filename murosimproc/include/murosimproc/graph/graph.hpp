// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GRAPH_GRAPH_HPP_
#define MUROSIMPROC_GRAPH_GRAPH_HPP_

#include <Eigen/Geometry>
#include <vector>
#include <set>
#include <utility>

namespace murosimproc::graph
{

/// Vertex property: 2D position
struct VertexProperty
{
    Eigen::Vector2f position;

    /// Constructs a vertex at the origin.
    VertexProperty() : position(Eigen::Vector2f::Zero()) {}
    /// Constructs a vertex at `pos`.
    VertexProperty(const Eigen::Vector2f& pos) : position(pos) {}
    /// Constructs a vertex at `(x, y)`.
    VertexProperty(float x, float y) : position(x, y) {}
};

/// Vertex descriptor type
using Vertex = size_t;

/// Edge descriptor type
struct Edge
{
    Vertex source;
    Vertex target;
};

/// Undirected graph with set-based adjacency (prevents duplicate edges).
/// Replaces boost::adjacency_list<setS, vecS, undirectedS, VertexProperty>.
class Graph
{
public:
    /// Add a vertex with the given property; returns its descriptor
    Vertex addVertex(const VertexProperty& prop)
    {
        Vertex v = vertices_.size();
        vertices_.push_back(prop);
        adjacency_.emplace_back();
        return v;
    }

    /// Add an undirected edge. Returns {edge, true} if new, {edge, false} if duplicate.
    std::pair<Edge, bool> addEdge(Vertex u, Vertex v)
    {
        bool inserted = adjacency_[u].insert(v).second;
        adjacency_[v].insert(u);
        return {Edge{u, v}, inserted};
    }

    /// Remove an undirected edge
    void removeEdge(Vertex u, Vertex v)
    {
        adjacency_[u].erase(v);
        adjacency_[v].erase(u);
    }

    /// Number of vertices
    size_t numVertices() const { return vertices_.size(); }

    /// Access vertex property by index
    const VertexProperty& operator[](Vertex v) const { return vertices_[v]; }
    VertexProperty& operator[](Vertex v) { return vertices_[v]; }

    /// Collect all unique edges (each undirected edge once, with source < target)
    std::vector<Edge> edges() const
    {
        std::vector<Edge> result;
        for (size_t u = 0; u < adjacency_.size(); ++u)
        {
            for (size_t v : adjacency_[u])
            {
                if (u < v)
                {
                    result.push_back(Edge{u, v});
                }
            }
        }
        return result;
    }

    /// Get neighbors of a vertex
    const std::set<size_t>& adjacentVertices(Vertex v) const
    {
        return adjacency_[v];
    }

private:
    std::vector<VertexProperty> vertices_;
    std::vector<std::set<size_t>> adjacency_;
};

/// Add a node with a 2D position; returns the vertex descriptor
inline Vertex addNode(Graph& g, const Eigen::Vector2f& position)
{
    return g.addVertex(VertexProperty(position));
}

/// Get the number of nodes
inline size_t getNodeSize(const Graph& g)
{
    return g.numVertices();
}

/// Add an edge between two vertices (undirected)
inline std::pair<Edge, bool> addEdge(Graph& g, Vertex u, Vertex v)
{
    return g.addEdge(u, v);
}

/// Remove an edge between two vertices
inline void removeEdge(Graph& g, Vertex u, Vertex v)
{
    g.removeEdge(u, v);
}

/// Get the position of a node
inline const Eigen::Vector2f& getNodePosition(const Graph& g, Vertex u)
{
    return g[u].position;
}

/// Get all edges as a vector
inline std::vector<Edge> getEdges(const Graph& g)
{
    return g.edges();
}

/// Add a bidirectional edge (same as addEdge for undirected graphs)
inline std::pair<Edge, bool> addBidirectionalEdge(Graph& g, Vertex u, Vertex v)
{
    return g.addEdge(u, v);
}

/// Check if graph is empty (no vertices)
inline bool empty(const Graph& g)
{
    return g.numVertices() == 0;
}

/// Get neighbors of a node
inline const std::set<size_t>& getNodeNeighbors(const Graph& g, Vertex node)
{
    return g.adjacentVertices(node);
}

} // namespace murosimproc::graph

#endif // MUROSIMPROC_GRAPH_GRAPH_HPP_
