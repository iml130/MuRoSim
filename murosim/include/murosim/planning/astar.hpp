// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_PLANNING_ASTAR_HPP_
#define MUROSIM_PLANNING_ASTAR_HPP_

#include "planner.hpp"
#include <unordered_map>
#include <vector>
#include <string>

namespace murosim::planning
{

/// A* grid path planner over a murosim::grid::Map, moving between FREE cells.
/// With use_neighbors8, diagonal moves are allowed (cost sqrt(2), guided by
/// the octile heuristic) and are only considered when both adjacent
/// orthogonal neighbors are also free, so the path never cuts through an
/// obstacle corner; otherwise only the 4 orthogonal moves are used (cost 1,
/// guided by the Manhattan heuristic). Reports its type as ASTAR8 or ASTAR4
/// accordingly.
class AStar : public Planner
{
public:
    /// Constructs an A* planner; use_neighbors8 selects 8-connected (diagonal-aware) vs. 4-connected search.
    AStar(const bool use_neighbors8 = true);
    /// Runs A* from point_start to point_goal (both assumed to be FREE cells).
    /// @return the path in world coordinates from start to goal, or an empty vector if no path exists.
    virtual std::vector<Eigen::Vector2f> computePath(const murosim::grid::Map& map, const Eigen::Vector2f point_start, const Eigen::Vector2f point_goal) override;
    virtual std::unique_ptr<Planner> clone() const override;
    /// @return Planner::TYPE::ASTAR8 or Planner::TYPE::ASTAR4 depending on the constructor argument.
    virtual Planner::TYPE getType() const override;

private:
    const bool use_neighbors8_;
    /// Manhattan (L1) distance heuristic, used with 4-connected search.
    static double heuristicManhattanDistance(const Eigen::Vector2i p1, const Eigen::Vector2i p2);
    /// Octile distance heuristic (optimal for 8-direction movement with diagonal cost sqrt(2)), used with 8-connected search.
    static double heuristicOctile(const Eigen::Vector2i p1, const Eigen::Vector2i p2);
    /// Returns the FREE 8-connected neighbors of the cell at index, each paired with its move cost (1 or sqrt(2)).
    /// A diagonal neighbor is only included when both straight neighbors flanking it are also free.
    std::vector<std::pair<size_t, double>> getNeighbors8(const murosim::grid::Map& map, const size_t index) const;
    /// Same as the index overload, for the cell at point.
    std::vector<std::pair<size_t, double>> getNeighbors8(const murosim::grid::Map& map, const Eigen::Vector2i point) const;
    /// Returns the FREE 4-connected (orthogonal) neighbors of the cell at index, each paired with move cost 1.
    std::vector<std::pair<size_t, double>> getNeighbors4(const murosim::grid::Map& map, const size_t index) const;
    /// Same as the index overload, for the cell at point.
    std::vector<std::pair<size_t, double>> getNeighbors4(const murosim::grid::Map& map, const Eigen::Vector2i point) const;

};
} // namespace murosim::planning

#endif // MUROSIM_PLANNING_ASTAR_HPP_
