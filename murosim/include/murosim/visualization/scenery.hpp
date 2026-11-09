// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_SCENERY_HPP_
#define MUROSIM_VISUALIZATION_SCENERY_HPP_

#include "murosim/io/data_container.hpp"
#include "murosim/simulation/simulation.hpp"
#include "murosim/visualization/color.hpp"
#include "murosim/visualization/canvas.hpp"

namespace murosim::visualization
{

/// Fixed drawing style and color palette for rendering murosim scenes onto a
/// Canvas: world geometry, occupancy grid, robots (shape, lidar rays, goal
/// marker, trajectory, planned path) and navigation nodes. Renderer owns one
/// Scenery and delegates to its drawScenery*() methods, which each call
/// canvas.drawAll() once so a whole scene is issued in a handful of GL draw
/// calls. Colors and toggles are configurable via the setters below; the
/// per-element depth/thickness/alpha constants in scenery.cpp are fixed.
class Scenery
{
private:

    Color color_background_;
    Color color_world_;
    Color color_goal_;
    Color color_goal_on_;
    Color color_goal_reached_;
    Color color_grid_map_free_;
    Color color_grid_map_occupied_;
    Color color_grid_map_buffer_;
    Color color_grid_map_unreachable_;
    size_t laser_skip_steps_;
    size_t robot_trajectory_length_;
    bool draw_grid_map_;
    bool draw_robot_path_;
    /// If true, world/robot/shape circles are drawn as unfilled ring outlines
    /// instead of filled circles.
    bool draw_ring_instead_of_circle_;

    void drawBackground(murosim::visualization::Canvas &canvas) const;
    void drawShape(murosim::visualization::Canvas &canvas, const murosim::geometry::Shape &shape) const;
    void drawWorld(murosim::visualization::Canvas &canvas, const murosim::geometry::World &world) const;
    void drawRobot(murosim::visualization::Canvas &canvas, const murosim::robot::Robot &robot) const;
    /// Same as drawRobot() but skips applying the robot's own pose transform;
    /// used when the caller has already placed the canvas in the robot's
    /// local frame.
    void drawRobotWithoutPoseTransform(murosim::visualization::Canvas &canvas, const murosim::robot::Robot &robot) const;
    /// Draws every robot in simulation plus its goal marker (colored by
    /// STATUS_STATE), recent trajectory, and planned path, per the
    /// corresponding draw/length settings.
    void drawRobots(murosim::visualization::Canvas &canvas, const murosim::simulation::Simulation &simulation) const;
    /// Draws lidar's individual laser rays as faint lines from its origin
    /// (pose composed with the lidar's own mount pose) to each range
    /// endpoint, sampling every laser_skip_steps_ rays; no-op if
    /// laser_skip_steps_ is 0.
    void drawLidarLaser(murosim::visualization::Canvas &canvas, const murosim::sensor::Lidar &lidar, const Eigen::Affine2f &pose, const murosim::visualization::Color &color) const;
    /// Draws grid_map as translucent quads if draw_grid_map_ is set; no-op otherwise.
    void drawGridMap(murosim::visualization::Canvas &canvas, const murosim::grid::Map &grid_map) const;
    void drawNode(murosim::visualization::Canvas &canvas, const murosim::navigation::Node &node) const;
    void drawNodes(murosim::visualization::Canvas &canvas, const murosim::navigation::Container &nodes) const;
public:
    /// Constructs a Scenery with the default palette (white background, black
    /// world/grid geometry, red/orange/green goal states) and default
    /// toggles (grid map hidden, robot path shown, circles drawn as rings).
    Scenery();
    void setColorBackground(murosim::visualization::Color &&color);
    /// @copydoc setColorBackground(murosim::visualization::Color&&)
    void setColorBackground(const murosim::visualization::Color &color);
    void setColorWorld(murosim::visualization::Color &&color);
    /// @copydoc setColorWorld(murosim::visualization::Color&&)
    void setColorWorld(const murosim::visualization::Color &color);
    /// Color of a robot's goal marker while it is still searching for the goal.
    void setColorGoal(murosim::visualization::Color && color);
    /// @copydoc setColorGoal(murosim::visualization::Color&&)
    void setColorGoal(const murosim::visualization::Color &color);
    /// Color of a robot's goal marker while the robot is on the goal.
    void setColorGoalOn(murosim::visualization::Color && color);
    /// @copydoc setColorGoalOn(murosim::visualization::Color&&)
    void setColorGoalOn(const murosim::visualization::Color &color);
    /// Color of a robot's goal marker once the goal has been reached.
    void setColorGoalReached(murosim::visualization::Color && color);
    /// @copydoc setColorGoalReached(murosim::visualization::Color&&)
    void setColorGoalReached(const murosim::visualization::Color &color);
    void setColorGridMapFree(murosim::visualization::Color && color);
    /// @copydoc setColorGridMapFree(murosim::visualization::Color&&)
    void setColorGridMapFree(const murosim::visualization::Color &color);
    void setColorGridMapOccupied(murosim::visualization::Color && color);
    /// @copydoc setColorGridMapOccupied(murosim::visualization::Color&&)
    void setColorGridMapOccupied(const murosim::visualization::Color &color);
    void setColorGridMapBuffer(murosim::visualization::Color && color);
    /// @copydoc setColorGridMapBuffer(murosim::visualization::Color&&)
    void setColorGridMapBuffer(const murosim::visualization::Color &color);
    /// Maximum number of recent poses drawn as a fading trajectory trail
    /// behind each robot; values <= 1 disable the trajectory.
    void setRobotTrajectoryLength(const size_t trajectory_length);
    /// Draw every Nth lidar ray (0 disables lidar-ray drawing entirely).
    void setLaserSkipSteps(const size_t skip_steps);
    void setDrawGridMap(const bool draw_grid_map);
    void setDrawRobotPath(const bool draw_path);
    /// Draws a grid of independent worlds (each with its own navigation
    /// nodes) side by side in one frame, one cell per DataContainer.
    void drawSceneryDataContainers(murosim::visualization::Canvas &canvas, const std::vector<murosim::io::DataContainer> &containers) const;
    /// Draws a grid of independent simulations (world, grid map, robots) side
    /// by side in one frame, one cell per simulation.
    void drawScenerySimulations(murosim::visualization::Canvas &canvas, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const;
    /// Draws one simulation's grid map, world and robots filling the whole frame.
    void drawScenerySimulation(murosim::visualization::Canvas &canvas, const murosim::simulation::Simulation &simulation) const;
    /// Draws just world's geometry filling the whole frame.
    void drawSceneryWorld(murosim::visualization::Canvas &canvas, const murosim::geometry::World &world) const;
    /// @copydoc drawSceneryWorld(murosim::visualization::Canvas&, const murosim::geometry::World&) const
    /// Also draws nodes on top.
    void drawSceneryWorldWithNodes(murosim::visualization::Canvas &canvas, const murosim::geometry::World &world, const murosim::navigation::Container &nodes) const;
    /// Draws a single standalone shape filling the whole frame.
    void drawSceneryShape(murosim::visualization::Canvas &canvas, const murosim::geometry::Shape &shape) const;
    /// Draws a single robot (with its own pose applied) filling the whole frame.
    void drawSceneryRobot(murosim::visualization::Canvas &canvas, const murosim::robot::Robot &robot) const;
};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_SCENERY_HPP_
