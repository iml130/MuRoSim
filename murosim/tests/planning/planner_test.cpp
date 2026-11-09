// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/planning/astar.hpp"
#include "murosim/planning/factory.hpp"
#include "murosim/planning/planner.hpp"
#include "murosim/planning/util.hpp"
#include "murosim/planning/voronoi.hpp"
#include "murosim/grid/map.hpp"
#include "murosim/geometry/world.hpp"
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using murosim::geometry::Line;
using murosim::geometry::World;
using murosim::grid::Map;
using murosim::planning::AStar;
using murosim::planning::Planner;
using murosim::planning::Voronoi;
using murosim::planning::factory::createPlanner;
using murosim::planning::util::findNearestPathToFreePoint;

constexpr float WORLD_SIZE = 10.0f;
constexpr float GRID_SCALE = 0.1f;
constexpr float BUFFER_SCALE = 0.2f;
constexpr float WIDE_BUFFER_SCALE = 0.4f; // a four-cell buffer band along the walls
constexpr bool NEIGHBORS_8 = true;
constexpr bool NEIGHBORS_4 = false;

// Path points are cell centres, so a requested point may come back up to
// half a cell away (measured: 0.04 m for a goal 0.04 m off the centre, 0.0 m
// when start and goal are free cells)
constexpr float HALF_CELL = GRID_SCALE / 2.0f;

/// 10 m x 10 m world enclosed by border walls.
World borderWorld()
{
    return World({Line(0.0f, 0.0f, WORLD_SIZE, 0.0f),
                  Line(WORLD_SIZE, 0.0f, WORLD_SIZE, WORLD_SIZE),
                  Line(WORLD_SIZE, WORLD_SIZE, 0.0f, WORLD_SIZE),
                  Line(0.0f, WORLD_SIZE, 0.0f, 0.0f)},
                 {});
}

/// The border world split into two sealed rooms by a full-height wall at
/// x = 5, so no path exists between them.
World splitWorld()
{
    return World({Line(0.0f, 0.0f, WORLD_SIZE, 0.0f),
                  Line(WORLD_SIZE, 0.0f, WORLD_SIZE, WORLD_SIZE),
                  Line(WORLD_SIZE, WORLD_SIZE, 0.0f, WORLD_SIZE),
                  Line(0.0f, WORLD_SIZE, 0.0f, 0.0f),
                  Line(WORLD_SIZE / 2.0f, 0.0f, WORLD_SIZE / 2.0f, WORLD_SIZE)},
                 {});
}

/// The border world with a wall at x = 5 from y = 2 up to y = 8: a path
/// from the left half to the right half has to go around one of its ends.
World barrierWorld()
{
    return World({Line(0.0f, 0.0f, WORLD_SIZE, 0.0f),
                  Line(WORLD_SIZE, 0.0f, WORLD_SIZE, WORLD_SIZE),
                  Line(WORLD_SIZE, WORLD_SIZE, 0.0f, WORLD_SIZE),
                  Line(0.0f, WORLD_SIZE, 0.0f, 0.0f),
                  Line(WORLD_SIZE / 2.0f, 2.0f, WORLD_SIZE / 2.0f, 8.0f)},
                 {});
}

/// The border world with a closed 2 m x 2 m box around (5, 5); its interior
/// is a separate region that the map marks UNREACHABLE.
World boxedWorld()
{
    return World({Line(0.0f, 0.0f, WORLD_SIZE, 0.0f),
                  Line(WORLD_SIZE, 0.0f, WORLD_SIZE, WORLD_SIZE),
                  Line(WORLD_SIZE, WORLD_SIZE, 0.0f, WORLD_SIZE),
                  Line(0.0f, WORLD_SIZE, 0.0f, 0.0f),
                  Line(4.0f, 4.0f, 6.0f, 4.0f),
                  Line(6.0f, 4.0f, 6.0f, 6.0f),
                  Line(6.0f, 6.0f, 4.0f, 6.0f),
                  Line(4.0f, 6.0f, 4.0f, 4.0f)},
                 {});
}

Map borderMap(const float buffer_scale = BUFFER_SCALE)
{
    return Map(borderWorld(), {}, GRID_SCALE, buffer_scale);
}

Eigen::Vector2i cellOf(const Map &map, const Eigen::Vector2f &point)
{
    Eigen::Vector2i cell;
    map.fromWorldIntoMap(point, cell);
    return cell;
}

/// Asserts that every path point lies on a traversable (FREE or BUFFER)
/// cell and that consecutive points are neighbouring cells: 8-connected
/// steps are at most one cell on each axis, 4-connected steps exactly one
/// cell on one axis. A path through a wall or with a jump fails here.
void assertPathTraversable(const Map &map, const std::vector<Eigen::Vector2f> &path, const bool neighbors8)
{
    ASSERT_FALSE(path.empty());
    for (size_t i = 0; i < path.size(); i++)
    {
        const Eigen::Vector2i cell = cellOf(map, path[i]);
        ASSERT_TRUE(map.isInMap(cell)) << "point " << i;
        ASSERT_TRUE(map.isFree(cell) || map.isBuffer(cell)) << "point " << i << " on cell " << cell.transpose();

        if (i > 0)
        {
            const Eigen::Vector2i step = (cell - cellOf(map, path[i - 1])).cwiseAbs();
            if (neighbors8)
            {
                ASSERT_LE(step.maxCoeff(), 1) << "step " << i;
            }
            else
            {
                ASSERT_EQ(step.x() + step.y(), 1) << "step " << i;
            }
            ASSERT_NE(step.x() + step.y(), 0) << "step " << i << " does not move";
        }
    }
}
} // namespace

// ============================================================================
// Plan Tests
// ============================================================================

// Regression: the single-cell early return used to leak grid-cell coordinates
// into the world-coordinate path.
TEST(PlannerPlan, SameCellReturnsWorldCoordinates)
{
    const Map map = borderMap();
    AStar planner(NEIGHBORS_8);

    const Eigen::Vector2f start(5.0f, 5.0f);
    const Eigen::Vector2f goal(5.04f, 5.04f); // same grid cell

    const std::vector<Eigen::Vector2f> path = planner.plan(map, start, goal);

    // The one point is the goal cell in world coordinates, not the raw grid
    // indices (which would be around (50, 50))
    ASSERT_EQ(path.size(), static_cast<size_t>(1));
    EXPECT_NEAR(path.front().x(), goal.x(), HALF_CELL);
    EXPECT_NEAR(path.front().y(), goal.y(), HALF_CELL);
}

// Regression: A* used to be given the non-FREE goal cell as target (instead
// of the nearest free cell), which can never enter the frontier, so planning
// silently returned an empty path whenever the goal lay in a BUFFER cell.
TEST(PlannerPlan, GoalInBufferCellIsReachable)
{
    const Map map = borderMap(WIDE_BUFFER_SCALE);
    AStar planner(NEIGHBORS_8);

    const Eigen::Vector2f start(5.0f, 5.0f);
    const Eigen::Vector2f goal(9.7f, 5.0f); // 0.3 m from the wall: inside the buffer

    // Guard the setup: the goal cell must really be non-FREE, the start FREE
    ASSERT_NE(map(cellOf(map, goal)), Map::FIELD::FREE);
    ASSERT_EQ(map(cellOf(map, start)), Map::FIELD::FREE);

    const std::vector<Eigen::Vector2f> path = planner.plan(map, start, goal);

    // The path runs from the start to the goal
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.front().x(), start.x(), HALF_CELL);
    EXPECT_NEAR(path.front().y(), start.y(), HALF_CELL);
    EXPECT_NEAR(path.back().x(), goal.x(), HALF_CELL);
    EXPECT_NEAR(path.back().y(), goal.y(), HALF_CELL);
    assertPathTraversable(map, path, NEIGHBORS_8);
}

TEST(PlannerPlan, FreeGoalRunsFromStartToGoal)
{
    const Map map = borderMap();
    AStar planner(NEIGHBORS_8);

    const Eigen::Vector2f start(2.0f, 2.0f);
    const Eigen::Vector2f goal(8.0f, 8.0f);

    const std::vector<Eigen::Vector2f> path = planner.plan(map, start, goal);

    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.front().x(), start.x(), HALF_CELL);
    EXPECT_NEAR(path.front().y(), start.y(), HALF_CELL);
    EXPECT_NEAR(path.back().x(), goal.x(), HALF_CELL);
    EXPECT_NEAR(path.back().y(), goal.y(), HALF_CELL);
}

TEST(PlannerPlan, EightNeighborPathIsTraversableAndConnected)
{
    const Map map = borderMap();
    AStar planner(NEIGHBORS_8);

    const std::vector<Eigen::Vector2f> path =
        planner.plan(map, Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(8.0f, 8.0f));

    // A diagonal of 60 cells needs 61 points with 8-connected steps
    EXPECT_EQ(path.size(), static_cast<size_t>(61));
    assertPathTraversable(map, path, NEIGHBORS_8);
}

TEST(PlannerPlan, FourNeighborPathHasOnlyAxisAlignedSteps)
{
    const Map map = borderMap();
    AStar planner(NEIGHBORS_4);

    const std::vector<Eigen::Vector2f> path =
        planner.plan(map, Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(8.0f, 8.0f));

    // 60 cells in x plus 60 in y: 121 points
    EXPECT_EQ(path.size(), static_cast<size_t>(121));
    assertPathTraversable(map, path, NEIGHBORS_4);
}

TEST(PlannerPlan, PathGoesAroundBarrier)
{
    const Map map(barrierWorld(), {}, GRID_SCALE, BUFFER_SCALE);
    AStar planner(NEIGHBORS_8);
    const Eigen::Vector2f start(3.0f, 5.0f);
    const Eigen::Vector2f goal(7.0f, 5.0f);

    const std::vector<Eigen::Vector2f> path = planner.plan(map, start, goal);

    // Straight across would be 41 points. Around the top end the path has
    // to climb past y = 8 plus the buffer: at least 33 diagonal cells up
    // and 33 down, 67 points (measured 69). No point may lie on the wall
    // or jump over it, and where the path crosses x = 5 it must be beyond
    // an end of the wall
    EXPECT_GE(path.size(), static_cast<size_t>(67));
    assertPathTraversable(map, path, NEIGHBORS_8);
    EXPECT_NEAR(path.front().x(), start.x(), HALF_CELL);
    EXPECT_NEAR(path.back().x(), goal.x(), HALF_CELL);
    for (const Eigen::Vector2f &point : path)
    {
        if (std::abs(point.x() - WORLD_SIZE / 2.0f) <= HALF_CELL)
        {
            EXPECT_GT(std::abs(point.y() - 5.0f), 3.0f) << "crossing at " << point.transpose();
        }
    }
}

// ============================================================================
// Failure Contract Tests
// ============================================================================

// plan() returns a non-empty path or throws: invalid input (a point outside
// the map) throws std::out_of_range, an unsolvable map throws
// std::runtime_error. It never returns an empty path.

TEST(PlannerFailure, StartOutOfMapThrowsOutOfRange)
{
    const Map map = borderMap();
    AStar planner(NEIGHBORS_8);

    EXPECT_THROW(planner.plan(map, Eigen::Vector2f(-5.0f, -5.0f), Eigen::Vector2f(8.0f, 8.0f)),
                 std::out_of_range);
}

TEST(PlannerFailure, GoalOutOfMapThrowsOutOfRange)
{
    const Map map = borderMap();
    AStar planner(NEIGHBORS_8);

    EXPECT_THROW(planner.plan(map, Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(50.0f, 50.0f)),
                 std::out_of_range);
}

TEST(PlannerFailure, DisconnectedRegionsThrowRuntimeError)
{
    const Map map(splitWorld(), {}, GRID_SCALE, BUFFER_SCALE);
    AStar planner(NEIGHBORS_8);

    // Start in the left room, goal in the right room: the rooms are sealed off
    EXPECT_THROW(planner.plan(map, Eigen::Vector2f(2.0f, 5.0f), Eigen::Vector2f(8.0f, 5.0f)),
                 std::runtime_error);
}

TEST(PlannerFailure, GoalInsideClosedBoxThrowsRuntimeError)
{
    const Map map(boxedWorld(), {}, GRID_SCALE, BUFFER_SCALE);
    AStar planner(NEIGHBORS_8);

    // The box interior is UNREACHABLE, so no free cell can be reached from the goal
    EXPECT_THROW(planner.plan(map, Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(5.0f, 5.0f)),
                 std::runtime_error);
}

// ============================================================================
// Type Tests
// ============================================================================

TEST(PlannerType, AStarReportsItsNeighbourMode)
{
    EXPECT_EQ(AStar(NEIGHBORS_4).getType(), Planner::TYPE::ASTAR4);
    EXPECT_EQ(AStar(NEIGHBORS_8).getType(), Planner::TYPE::ASTAR8);
    EXPECT_EQ(AStar().getType(), Planner::TYPE::ASTAR8); // default
}

TEST(PlannerType, TypeAsStringNamesEveryValue)
{
    EXPECT_EQ(Planner::getTypeAsString(Planner::TYPE::NONE), "NONE");
    EXPECT_EQ(Planner::getTypeAsString(Planner::TYPE::ASTAR4), "ASTAR4");
    EXPECT_EQ(Planner::getTypeAsString(Planner::TYPE::ASTAR8), "ASTAR8");
    EXPECT_EQ(Planner::getTypeAsString(Planner::TYPE::VORONOI), "VORONOI");
    EXPECT_EQ(AStar(NEIGHBORS_4).getTypeAsString(), "ASTAR4");
}

TEST(PlannerType, TypeFromStringRoundTrips)
{
    const std::vector<Planner::TYPE> types = {Planner::TYPE::NONE, Planner::TYPE::ASTAR4, Planner::TYPE::ASTAR8,
                                              Planner::TYPE::VORONOI};

    for (const Planner::TYPE type : types)
    {
        EXPECT_EQ(Planner::getTypeFromString(Planner::getTypeAsString(type)), type) << Planner::getTypeAsString(type);
    }
}

TEST(PlannerType, TypeFromStringIgnoresCase)
{
    EXPECT_EQ(Planner::getTypeFromString("astar8"), Planner::TYPE::ASTAR8);
    EXPECT_EQ(Planner::getTypeFromString("Voronoi"), Planner::TYPE::VORONOI);
}

TEST(PlannerType, UnknownTypeStringThrows)
{
    EXPECT_THROW(Planner::getTypeFromString("dijkstra"), std::runtime_error);
}

TEST(PlannerType, CloneKeepsNeighbourMode)
{
    const std::unique_ptr<Planner> clone = AStar(NEIGHBORS_4).clone();
    const Map map = borderMap();

    ASSERT_NE(clone, nullptr);
    EXPECT_EQ(clone->getType(), Planner::TYPE::ASTAR4);
    assertPathTraversable(map, clone->plan(map, Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(8.0f, 8.0f)),
                          NEIGHBORS_4);
}

// ============================================================================
// Factory Tests
// ============================================================================

TEST(PlannerFactory, NoneYieldsNoPlanner)
{
    EXPECT_EQ(createPlanner(Planner::TYPE::NONE), nullptr);
}

TEST(PlannerFactory, AStarTypesYieldMatchingPlanners)
{
    const std::unique_ptr<Planner> astar4 = createPlanner(Planner::TYPE::ASTAR4);
    const std::unique_ptr<Planner> astar8 = createPlanner(Planner::TYPE::ASTAR8);

    ASSERT_NE(astar4, nullptr);
    ASSERT_NE(astar8, nullptr);
    EXPECT_EQ(astar4->getType(), Planner::TYPE::ASTAR4);
    EXPECT_EQ(astar8->getType(), Planner::TYPE::ASTAR8);
}

TEST(PlannerFactory, AStar4PlannerPlansAxisAlignedPaths)
{
    const std::unique_ptr<Planner> planner = createPlanner(Planner::TYPE::ASTAR4);
    const Map map = borderMap();

    assertPathTraversable(map, planner->plan(map, Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(8.0f, 8.0f)),
                          NEIGHBORS_4);
}

TEST(PlannerFactory, VoronoiTypeYieldsVoronoiPlanner)
{
    const std::unique_ptr<Planner> planner = createPlanner(Planner::TYPE::VORONOI);

    ASSERT_NE(planner, nullptr);
    EXPECT_EQ(planner->getType(), Planner::TYPE::VORONOI);
}

// ============================================================================
// Voronoi Tests
// ============================================================================

// The Voronoi planner is a placeholder: it reports its type and clones, but
// planning is not implemented and throws.

TEST(PlannerVoronoi, ReportsVoronoiType)
{
    EXPECT_EQ(Voronoi().getType(), Planner::TYPE::VORONOI);
    EXPECT_EQ(Voronoi().getTypeAsString(), "VORONOI");
}

TEST(PlannerVoronoi, CloneIsVoronoi)
{
    const std::unique_ptr<Planner> clone = Voronoi().clone();

    ASSERT_NE(clone, nullptr);
    EXPECT_EQ(clone->getType(), Planner::TYPE::VORONOI);
}

TEST(PlannerVoronoi, PlanThrowsNotImplemented)
{
    const Map map = borderMap();
    Voronoi planner;

    EXPECT_THROW(planner.plan(map, Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(8.0f, 8.0f)), std::runtime_error);
}

// ============================================================================
// Nearest Free Point Tests
// ============================================================================

TEST(PlannerUtilNearestFreePoint, FreePointReturnsItself)
{
    const Map map = borderMap();
    const Eigen::Vector2f point(5.03f, 5.07f); // off the cell centre on purpose

    const std::vector<Eigen::Vector2f> path = findNearestPathToFreePoint(map, point);

    ASSERT_EQ(path.size(), static_cast<size_t>(1));
    EXPECT_FLOAT_EQ(path.front().x(), point.x()); // the point itself, not the cell centre
    EXPECT_FLOAT_EQ(path.front().y(), point.y());
}

TEST(PlannerUtilNearestFreePoint, BufferPointEscapesToNearestFreeCell)
{
    const Map map = borderMap(WIDE_BUFFER_SCALE);
    const Eigen::Vector2f point(9.7f, 5.0f); // 0.3 m from the right wall, inside the buffer band
    ASSERT_TRUE(map.isBuffer(cellOf(map, point)));

    const std::vector<Eigen::Vector2f> path = findNearestPathToFreePoint(map, point);

    // Straight left, one cell per step, until the first FREE cell
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.front().x(), point.x(), HALF_CELL);
    EXPECT_NEAR(path.front().y(), point.y(), HALF_CELL);
    EXPECT_TRUE(map.isFree(cellOf(map, path.back())));
    for (size_t i = 0; i + 1 < path.size(); i++)
    {
        EXPECT_TRUE(map.isBuffer(cellOf(map, path[i]))) << "point " << i;
        EXPECT_NEAR(path[i + 1].x(), path[i].x() - GRID_SCALE, HALF_CELL) << "step " << i;
        EXPECT_NEAR(path[i + 1].y(), point.y(), HALF_CELL) << "step " << i;
    }
}

TEST(PlannerUtilNearestFreePoint, OccupiedPointEscapesThroughTheBuffer)
{
    const Map map = borderMap();
    const Eigen::Vector2f point(WORLD_SIZE, 5.0f); // on the right wall
    ASSERT_TRUE(map.isObstacle(cellOf(map, point)));

    const std::vector<Eigen::Vector2f> path = findNearestPathToFreePoint(map, point);

    ASSERT_GT(path.size(), static_cast<size_t>(1));
    EXPECT_TRUE(map.isFree(cellOf(map, path.back())));
    EXPECT_LT(path.back().x(), point.x());
}

TEST(PlannerUtilNearestFreePoint, CellOverloadMatchesWorldOverload)
{
    const Map map = borderMap(WIDE_BUFFER_SCALE);
    const Eigen::Vector2f point(9.7f, 5.0f);

    const std::vector<Eigen::Vector2f> from_world = findNearestPathToFreePoint(map, point);
    const std::vector<Eigen::Vector2f> from_cell = findNearestPathToFreePoint(map, cellOf(map, point));

    ASSERT_EQ(from_cell.size(), from_world.size());
    for (size_t i = 0; i < from_world.size(); i++)
    {
        EXPECT_NEAR(from_cell[i].x(), from_world[i].x(), HALF_CELL) << "point " << i;
        EXPECT_NEAR(from_cell[i].y(), from_world[i].y(), HALF_CELL) << "point " << i;
    }
}

TEST(PlannerUtilNearestFreePoint, EnclosedPointHasNoEscape)
{
    const Map map(boxedWorld(), {}, GRID_SCALE, BUFFER_SCALE);
    const Eigen::Vector2f point(5.0f, 5.0f); // inside the closed box

    EXPECT_TRUE(findNearestPathToFreePoint(map, point).empty());
}

TEST(PlannerUtilNearestFreePoint, PointOutsideMapThrows)
{
    const Map map = borderMap();

    EXPECT_THROW(findNearestPathToFreePoint(map, Eigen::Vector2f(-5.0f, 5.0f)), std::runtime_error);
}
