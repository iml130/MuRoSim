// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/visualization/scenery.hpp"

#include <algorithm>

constexpr float ALPHA_LASER = 0.5f;
constexpr float ALPHA_NODE_CIRCLE = 0.25f;
constexpr float ALPHA_NODE_START_ANGLE = 0.75f;
constexpr float ALPHA_NODE_BIAS = 0.5f;
constexpr float ALPHA_GOAL = 0.25f;
constexpr float ALPHA_SHAPE_ROBOT = 1.0f;
constexpr float ALPHA_SHAPE_LIDAR = 0.9f;
constexpr float ALPHA_ROBOT_TRAJECTORY = 0.2f;
constexpr float ALPHA_ROBOT_PATH = 0.4f;
constexpr float ALPHA_GRID_MAP = 0.25f;
constexpr float THICKNESS_WORLD = 1.0f;
constexpr float THICKNESS_ROBOT = 1.0f;
constexpr float THICKNESS_ROBOT_TRAJECTORY = 2.0f;
constexpr float THICKNESS_ROBOT_PATH = 1.5f;
constexpr float THICKNESS_LASER = 0.5f;
constexpr float THICKNESS_GOAL = 1.0f;
constexpr float DEPTH_WORLD = 0.9f;
constexpr float DEPTH_GOAL = 0.8f;
constexpr float DEPTH_ROBOT = 0.5f;
constexpr float DEPTH_ROBOT_TRAJECTORY = 0.75f;
constexpr float DEPTH_ROBOT_PATH = 0.6f;
constexpr float DEPTH_LASER = 0.7f;
constexpr float DEPTH_GRID_MAP = 0.95f;


namespace murosim::visualization
{

Scenery::Scenery()
  : color_background_(Color(1.0f, 1.0f, 1.0f)),
    color_world_(Color(0.0f, 0.0f, 0.0f)),
    color_goal_(Color(1.0f, 0.0f, 0.0f)),
    color_goal_on_(Color(1.0f, 0.5f, 0.0f)),
    color_goal_reached_(Color(0.0f, 1.0f, 0.0f)),
    color_grid_map_free_(Color(1.0f, 1.0f, 1.0f)),
    color_grid_map_occupied_(Color(0.0f, 0.0f, 0.0f)),
    color_grid_map_buffer_(Color(0.5f, 0.5f, 0.5f)),
    color_grid_map_unreachable_(Color(0.25f, 0.25f, 0.25f)),
    laser_skip_steps_(0),
    robot_trajectory_length_(30),
    draw_grid_map_(false),
    draw_robot_path_(true),
    draw_ring_instead_of_circle_(true)
{

}

void Scenery::drawBackground(murosim::visualization::Canvas &canvas) const
{
    canvas.drawBackground(color_background_);
}

void Scenery::drawShape(murosim::visualization::Canvas &canvas, const murosim::geometry::Shape &shape) const
{
    canvas.addShape(shape, color_world_, THICKNESS_WORLD, DEPTH_WORLD, draw_ring_instead_of_circle_);
}

void Scenery::drawWorld(murosim::visualization::Canvas &canvas, const murosim::geometry::World &world) const
{
    for (size_t i = 0; i < world.getLineSize(); i++)
    {
        canvas.addLine(world.getLineAt(i), color_world_, THICKNESS_WORLD, DEPTH_WORLD);
    }

    if (draw_ring_instead_of_circle_)
    {
        for (size_t i = 0; i < world.getCircleSize(); i++)
        {
            canvas.addRing(world.getCircleAt(i), color_world_, THICKNESS_WORLD, DEPTH_WORLD);
        }
    }
    else
    {
        for (size_t i = 0; i < world.getCircleSize(); i++)
        {
            canvas.addCircle(world.getCircleAt(i), color_world_, DEPTH_WORLD);
        }
    }
}

void Scenery::drawLidarLaser(murosim::visualization::Canvas &canvas, const murosim::sensor::Lidar &lidar, const Eigen::Affine2f &pose, const murosim::visualization::Color &color) const
{
    const murosim::visualization::Color color_laser = murosim::visualization::Color(color.r(), color.g(), color.b(), color.a() * ALPHA_LASER);

    if(laser_skip_steps_ != 0)
    {
        const Eigen::Affine2f p = pose * lidar.getPose();

        for(size_t index = laser_skip_steps_ / 2; index < lidar.getLaserSize(); index += laser_skip_steps_)
        {
            const Eigen::Vector2f p_laser = p * lidar.getLaserPoint(index);
            canvas.addLine(murosim::geometry::Line(p.translation(), p_laser), color_laser, THICKNESS_LASER, DEPTH_LASER);
        }
    }
}

void Scenery::drawGridMap(murosim::visualization::Canvas &canvas, const murosim::grid::Map &grid_map) const
{
    if (draw_grid_map_)
    {
        const murosim::visualization::Color color_free = murosim::visualization::Color(color_grid_map_free_.r(), color_grid_map_free_.g(), color_grid_map_free_.b(), color_grid_map_free_.a() * ALPHA_GRID_MAP);
        const murosim::visualization::Color color_occupied = murosim::visualization::Color(color_grid_map_occupied_.r(), color_grid_map_occupied_.g(), color_grid_map_occupied_.b(), color_grid_map_occupied_.a() * ALPHA_GRID_MAP);
        const murosim::visualization::Color color_buffer = murosim::visualization::Color(color_grid_map_buffer_.r(), color_grid_map_buffer_.g(), color_grid_map_buffer_.b(), color_grid_map_buffer_.a() * ALPHA_GRID_MAP);
        canvas.addGridMap(grid_map, color_free, color_occupied, color_buffer, DEPTH_GRID_MAP);
    }
}



void Scenery::drawNode(murosim::visualization::Canvas &canvas, const murosim::navigation::Node &node) const
{
    const murosim::visualization::Color color_circle = murosim::visualization::Color(color_goal_.r(), color_goal_.g(), color_goal_.b(), color_goal_.a() * ALPHA_NODE_CIRCLE);
    const murosim::visualization::Color color_start_angle = murosim::visualization::Color(color_goal_.r(), color_goal_.g(), color_goal_.b(), color_goal_.a() * ALPHA_NODE_START_ANGLE);
    const murosim::visualization::Color color_bias = murosim::visualization::Color(color_goal_.r(), color_goal_.g(), color_goal_.b(), color_goal_.a() * ALPHA_NODE_BIAS);
    // getGoalRadius() already resolves the "auto" sentinel to the circle radius
    const float radius = node.getGoalRadius();

    const Eigen::Vector2f point_center = node.getPoint();
    const Eigen::Vector2f point_start_angle = Eigen::Rotation2Df(node.getStartAngle()) * Eigen::Vector2f(radius, 0) + point_center;
    const Eigen::Vector2f point_start_bias_left = Eigen::Rotation2Df(node.getStartAngle() + node.getStartBias()) * Eigen::Vector2f(radius, 0) + point_center;
    const Eigen::Vector2f point_start_bias_right = Eigen::Rotation2Df(node.getStartAngle() - node.getStartBias()) * Eigen::Vector2f(radius, 0) + point_center;

    canvas.addRing(node.getCircle(), color_circle, THICKNESS_GOAL, DEPTH_GOAL);
    canvas.addLine(murosim::geometry::Line(point_center, point_start_angle), color_start_angle, THICKNESS_GOAL, DEPTH_GOAL);
    canvas.addLine(murosim::geometry::Line(point_center, point_start_bias_left), color_bias, THICKNESS_GOAL * 0.5f, DEPTH_GOAL);
    canvas.addLine(murosim::geometry::Line(point_center, point_start_bias_right), color_bias, THICKNESS_GOAL * 0.5f, DEPTH_GOAL);
}

void Scenery::drawNodes(murosim::visualization::Canvas &canvas, const murosim::navigation::Container &nodes) const
{
    for (const auto& node : nodes.getNodes())
    {
        drawNode(canvas, node);
    }
}

void Scenery::drawRobot(murosim::visualization::Canvas &canvas, const murosim::robot::Robot &robot) const
{
    const murosim::visualization::Color color_robot = robot.getColorRobot();
    const murosim::visualization::Color color_shape_robot = murosim::visualization::Color(color_robot.r(), color_robot.g(), color_robot.b(), color_robot.a() * ALPHA_SHAPE_ROBOT);
    const murosim::visualization::Color color_shape_lidar = murosim::visualization::Color(color_robot.r(), color_robot.g(), color_robot.b(), color_robot.a() * ALPHA_SHAPE_LIDAR);
    const murosim::visualization::Color color_lidar = robot.getColorLidar();
    const murosim::visualization::Color color_laser = murosim::visualization::Color(color_lidar.r(), color_lidar.g(), color_lidar.b(), color_lidar.a() * ALPHA_LASER);
    const Eigen::Affine2f pose = robot.getPose();

    for (const murosim::sensor::Lidar &lidar : robot.getLidars())
    {
        drawLidarLaser(canvas, lidar, pose, color_laser);
    }

    canvas.addShape(robot.getShapeRobot(), pose, color_shape_robot, THICKNESS_ROBOT, DEPTH_ROBOT, draw_ring_instead_of_circle_);
    canvas.addShape(robot.getShapeLidar(), pose, color_shape_lidar, THICKNESS_ROBOT, DEPTH_ROBOT, draw_ring_instead_of_circle_);
}

void Scenery::drawRobotWithoutPoseTransform(murosim::visualization::Canvas &canvas, const murosim::robot::Robot &robot) const
{
    const murosim::visualization::Color color_robot = robot.getColorRobot();
    const murosim::visualization::Color color_shape_robot = murosim::visualization::Color(color_robot.r(), color_robot.g(), color_robot.b(), color_robot.a() * ALPHA_SHAPE_ROBOT);
    const murosim::visualization::Color color_shape_lidar = murosim::visualization::Color(color_robot.r(), color_robot.g(), color_robot.b(), color_robot.a() * ALPHA_SHAPE_LIDAR);
    const murosim::visualization::Color color_lidar = robot.getColorLidar();
    const murosim::visualization::Color color_laser = murosim::visualization::Color(color_lidar.r(), color_lidar.g(), color_lidar.b(), color_lidar.a() * ALPHA_LASER);

    for (const murosim::sensor::Lidar &lidar : robot.getLidars())
    {
        drawLidarLaser(canvas, lidar, Eigen::Affine2f::Identity(), color_laser);
    }

    canvas.addShape(robot.getShapeRobot(), color_shape_robot, THICKNESS_ROBOT, DEPTH_ROBOT, draw_ring_instead_of_circle_);
    canvas.addShape(robot.getShapeLidar(), color_shape_lidar, THICKNESS_ROBOT, DEPTH_ROBOT, draw_ring_instead_of_circle_);
}

void Scenery::drawRobots(murosim::visualization::Canvas &canvas, const murosim::simulation::Simulation &simulation) const
{
    for (const auto& robot_ptr : simulation.getRobots())
    {
        const murosim::robot::Robot &robot = *robot_ptr;
        const murosim::visualization::Color color_robot = robot.getColorRobot();

        drawRobot(canvas, robot);

        const murosim::geometry::Circle &goal_circle = robot.getGoal().getCircle();

        const murosim::visualization::Color *color_goal_state = nullptr;
        switch (robot.getStatusState())
        {
        case murosim::robot::Robot::STATUS_STATE::SEARCH_GOAL:
            color_goal_state = &color_goal_;
            break;

        case murosim::robot::Robot::STATUS_STATE::ON_GOAL:
            color_goal_state = &color_goal_on_;
            break;

        case murosim::robot::Robot::STATUS_STATE::REACHED_GOAL:
            color_goal_state = &color_goal_reached_;
            break;

        default:
            break;
        }

        if (color_goal_state != nullptr)
        {
            canvas.addCircle(goal_circle,
                murosim::visualization::Color(
                    color_goal_state->r(),
                    color_goal_state->g(),
                    color_goal_state->b(),
                    color_goal_state->a() * ALPHA_GOAL),
                DEPTH_GOAL);
        }

        // Trajectory and planner-path drawing route through the line shader
        // (one segment per line) because ShaderPath::draw issues
        // glDrawArraysInstanced with 2 vertices per instance, which produces
        // no triangles for GL_TRIANGLE_STRIP and renders nothing.
        if (1 < robot.getStateSize() && 1 < robot_trajectory_length_)
        {
            const size_t size = std::min(robot_trajectory_length_, robot.getStateSize());
            const float steps = static_cast<float>(size > 1 ? size - 1 : 1);
            const float a_start = color_robot.a() * ALPHA_ROBOT_TRAJECTORY;

            for (size_t i = 0; i + 1 < size; i++)
            {
                const Eigen::Vector2f p_a = robot.getStateAt(i).getPose().translation();
                const Eigen::Vector2f p_b = robot.getStateAt(i + 1).getPose().translation();
                const float t = static_cast<float>(i) / steps;
                const float alpha = a_start * (1.0f - t);
                const murosim::visualization::Color seg_color(
                    color_robot.r(), color_robot.g(), color_robot.b(), alpha);
                canvas.addLine(murosim::geometry::Line(p_a, p_b),
                    seg_color, THICKNESS_ROBOT_TRAJECTORY, DEPTH_ROBOT_TRAJECTORY);
            }
        }

        if (draw_robot_path_ && 2 <= robot.getPathPointSize())
        {
            const murosim::visualization::Color color = robot.getColorPath();
            const murosim::visualization::Color color_path(
                color.r(), color.g(), color.b(), color.a() * ALPHA_ROBOT_PATH);

            const std::vector<Eigen::Vector2f>& path = robot.getPathPoints();
            std::vector<murosim::geometry::Line> segments;
            segments.reserve(path.size() - 1);
            for (size_t i = 0; i + 1 < path.size(); i++)
            {
                segments.emplace_back(path[i], path[i + 1]);
            }
            canvas.addLine(segments, color_path,
                THICKNESS_ROBOT_PATH, DEPTH_ROBOT_PATH);
        }

    }
}

void Scenery::setColorBackground(murosim::visualization::Color &&color)
{
    color_background_ = std::move(color);
}

void Scenery::setColorBackground(const murosim::visualization::Color &color)
{
    color_background_ = color;
}

void Scenery::setColorWorld(murosim::visualization::Color &&color)
{
    color_world_ = std::move(color);
}

void Scenery::setColorWorld(const murosim::visualization::Color &color)
{
    color_world_ = color;
}

void Scenery::setColorGoal(murosim::visualization::Color && color)
{
    color_goal_ = std::move(color);
}

void Scenery::setColorGoal(const murosim::visualization::Color &color)
{
    color_goal_ = color;
}

void Scenery::setColorGoalOn(murosim::visualization::Color && color)
{
    color_goal_on_ = std::move(color);
}

void Scenery::setColorGoalOn(const murosim::visualization::Color &color)
{
    color_goal_on_ = color;
}

void Scenery::setColorGoalReached(murosim::visualization::Color && color)
{
    color_goal_reached_ = std::move(color);
}

void Scenery::setColorGoalReached(const murosim::visualization::Color &color)
{
    color_goal_reached_ = color;
}

void Scenery::setColorGridMapFree(murosim::visualization::Color &&color)
{
    color_grid_map_free_ = std::move(color);
}

void Scenery::setColorGridMapFree(const murosim::visualization::Color &color)
{
    color_grid_map_free_ = color;
}

void Scenery::setColorGridMapOccupied(murosim::visualization::Color &&color)
{
    color_grid_map_occupied_ = std::move(color);
}

void Scenery::setColorGridMapOccupied(const murosim::visualization::Color &color)
{
    color_grid_map_occupied_ = color;
}

void Scenery::setColorGridMapBuffer(murosim::visualization::Color &&color)
{
    color_grid_map_buffer_ = std::move(color);
}

void Scenery::setColorGridMapBuffer(const murosim::visualization::Color &color)
{
    color_grid_map_buffer_ = color;
}

void Scenery::setRobotTrajectoryLength(const size_t trajectory_length)
{
    robot_trajectory_length_ = trajectory_length;
}

void Scenery::setLaserSkipSteps(const size_t skip_steps)
{
    laser_skip_steps_ = skip_steps;
}

void Scenery::setDrawGridMap(const bool draw_grid_map)
{
    draw_grid_map_ = draw_grid_map;
}

void Scenery::setDrawRobotPath(const bool draw_robot_path)
{
    draw_robot_path_ = draw_robot_path;
}

void Scenery::drawSceneryDataContainers(murosim::visualization::Canvas &canvas, const std::vector<murosim::io::DataContainer> &containers) const
{
    drawBackground(canvas);

    const float margin = 0.02f;
    const std::size_t grid      = static_cast<std::size_t>(std::ceil(std::sqrt(containers.size())));
    const float       cell_size  = 2.0f / static_cast<float>(grid);
    const Eigen::Vector2f origin_offset = Eigen::Vector2f(cell_size, cell_size) * margin * 0.5f;
    const Eigen::Vector2f scale = Eigen::Vector2f(cell_size, cell_size) * (1.0f - margin);

    canvas.setDrawThickness(canvas.getDrawThickness() * cell_size);

    for (size_t i = 0; i < containers.size(); i++)
    {
        const murosim::geometry::World world = containers[i].createWorld();
        const murosim::navigation::Container nodes = containers[i].createNodeContainer();
        const Eigen::Vector2f area_min = world.getAabbMin();
        const Eigen::Vector2f area_max = world.getAabbMax();

        const Eigen::Vector2f origin = Eigen::Vector2f(-1.0f, -1.0f) + Eigen::Vector2f(i / grid, i % grid) * cell_size + origin_offset;

        canvas.setDrawArea(area_min, area_max, origin, scale);

        drawWorld(canvas, world);
        drawNodes(canvas, nodes);
    }

    canvas.drawAll();
}

void Scenery::drawScenerySimulations(murosim::visualization::Canvas &canvas, const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) const
{
    drawBackground(canvas);

    const float margin = 0.02f;
    const std::size_t grid      = static_cast<std::size_t>(std::ceil(std::sqrt(simulations.size())));
    const float       cell_size  = 2.0f / static_cast<float>(grid);
    const Eigen::Vector2f origin_offset = Eigen::Vector2f(cell_size, cell_size) * margin * 0.5f;
    const Eigen::Vector2f scale = Eigen::Vector2f(cell_size, cell_size) * (1.0f - margin);

    canvas.setDrawThickness(canvas.getDrawThickness() * cell_size);

    for (size_t i = 0; i < simulations.size(); i++)
    {
        assert(simulations[i] != nullptr);

        const std::shared_ptr<murosim::simulation::Simulation> &simulation = simulations[i];
        const Eigen::Vector2f area_min = simulation->getWorld().getAabbMin();
        const Eigen::Vector2f area_max = simulation->getWorld().getAabbMax();

        const Eigen::Vector2f origin = Eigen::Vector2f(-1.0f, -1.0f) + Eigen::Vector2f(i / grid, i % grid) * cell_size + origin_offset;

        canvas.setDrawArea(area_min, area_max, origin, scale);

        drawGridMap(canvas, simulation->getGridMap());
        drawWorld(canvas, simulation->getWorld());
        drawRobots(canvas, *simulation);
    }

    canvas.drawAll();
}

void Scenery::drawScenerySimulation(murosim::visualization::Canvas &canvas, const murosim::simulation::Simulation &simulation) const
{
    drawBackground(canvas);
    drawGridMap(canvas, simulation.getGridMap());
    drawWorld(canvas, simulation.getWorld());
    drawRobots(canvas, simulation);
    canvas.drawAll();
}

void Scenery::drawSceneryWorld(murosim::visualization::Canvas &canvas, const murosim::geometry::World &world) const
{
    drawBackground(canvas);
    drawWorld(canvas, world);
    canvas.drawAll();
}

void Scenery::drawSceneryWorldWithNodes(murosim::visualization::Canvas &canvas, const murosim::geometry::World &world, const murosim::navigation::Container &nodes) const
{
    drawBackground(canvas);
    drawWorld(canvas, world);
    drawNodes(canvas, nodes);
    canvas.drawAll();
}

void Scenery::drawSceneryShape(murosim::visualization::Canvas &canvas, const murosim::geometry::Shape &shape) const
{
    drawBackground(canvas);
    drawShape(canvas, shape);
    canvas.drawAll();
}

void Scenery::drawSceneryRobot(murosim::visualization::Canvas &canvas, const murosim::robot::Robot &robot) const
{
    drawBackground(canvas);
    drawRobot(canvas, robot);
    canvas.drawAll();
}

} // namespace murosim::visualization
