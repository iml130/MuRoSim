// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/collision/lidar.hpp"

#include "murosim/collision/laser.hpp"

#include <stdexcept>
#include <vector>

namespace murosim::collision
{

void lidarSweepComplete(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const murosim::geometry::World &world, murosim::random::RandomGenerator &random_generator)
{
    // Iterate through each robot
    for (size_t i = 0; i < robots.size(); i++)
    {
        murosim::robot::Robot &robot = *(robots[i]);

        if (robot.isFinished())
        {
            continue;
        }

        // Iterate through the lidar of the robot
        for (murosim::sensor::Lidar &lidar : robot.getLidars())
        {
            lidar.fillLaserDistanceWithInf();

            // The geometry must be transformed into the lidar frame, which
            // the raw laser kernels (lidar at the origin) require.
            const Eigen::Affine2f world_to_lidar = (robot.getPose() * lidar.getPose()).inverse();

            lidarInFrameWithWorld(lidar, world_to_lidar, world);

            for (size_t k = 0; k < robots.size(); k++)
            {
                if (i != k)
                {
                    lidarInFrameWithShape(lidar, world_to_lidar, robots[k]->getShapeLidar(), robots[k]->getPose());
                }
            }

            // nearest hits -> readings: range limits (+inf/-inf) and noise
            lidar.applyNoise(random_generator);

            // a complete sweep finishes a full rotation: publish it at once
            lidar.publishScan();
        }
    }
}

namespace
{

/** \brief Recompute the rays in [index_begin, index_end) of one lidar against
 * the world and the other robots, including the range limits and the per-ray
 * noise.
 *
 * Every step writes exactly the window: fillLaserDistanceWithInf and
 * applyNoise are element-wise, and the laser kernels store only in-window lanes
 * (see storeWindow in array_kernels.cpp), so rays outside [index_begin,
 * index_end) are never touched.
 */
void sweepLidarWindow(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar,
                      const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const size_t index_robot,
                      const murosim::geometry::World &world, murosim::random::RandomGenerator &random_generator,
                      const size_t index_begin, const size_t index_end)
{
    if (index_begin >= index_end)
    {
        return;
    }

    lidar.fillLaserDistanceWithInf(index_begin, index_end);

    murosim::collision::lidarInFrameWithWorld(lidar, world_to_lidar, world, index_begin, index_end);

    for (size_t k = 0; k < robots.size(); k++)
    {
        if (k != index_robot)
        {
            murosim::collision::lidarInFrameWithShape(lidar, world_to_lidar, robots[k]->getShapeLidar(), robots[k]->getPose(), index_begin, index_end);
        }
    }

    lidar.applyNoise(random_generator, index_begin, index_end);
}

} // namespace

void lidarSweepTime(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const murosim::geometry::World &world, murosim::random::RandomGenerator &random_generator, const float dt)
{
    // Iterate through each robot
    for (size_t i = 0; i < robots.size(); i++)
    {
        murosim::robot::Robot &robot = *(robots[i]);

        if (robot.isFinished())
        {
            continue;
        }

        // Iterate through the lidar of the robot
        for (murosim::sensor::Lidar &lidar : robot.getLidars())
        {
            // Validate that frequency is set for time-based sweep
            if (lidar.getFrequency() <= 0.0f)
            {
                throw std::runtime_error("Lidar frequency must be positive for time-based sensor updates (CONTINUOUS mode)");
            }

            // Advance the beam from its current phase by dt. The phase is kept
            // bounded in the lidar, so the mapping never sees a large absolute
            // time and the scan continues from where the last call left off.
            size_t index_begin = 0;
            size_t index_end = 0;
            const float time_previous = lidar.getScanTime();
            const bool window_in_next_rotation = lidar.indexWindowForTimeInterval(time_previous, time_previous + dt, index_begin, index_end);
            lidar.advanceScanTime(dt);

            const size_t num_rays = lidar.getLaserSize();
            const Eigen::Affine2f world_to_lidar = (robot.getPose() * lidar.getPose()).inverse();

            // A rotation completes when the beam crosses the scan seam (the start
            // angle), i.e. when the phase advances past one rotation period.
            const float scan_duration = 1.0f / lidar.getFrequency();
            const bool rotation_completed = (time_previous + dt) >= scan_duration;

            // a wrapped window (begin > end) is processed as two contiguous segments
            if (window_in_next_rotation)
            {
                // the interval began in the dead zone and crossed the seam: the
                // completed rotation is already fully in the buffer, so publish
                // it before the next rotation's first rays overwrite it
                lidar.publishScan();
                sweepLidarWindow(lidar, world_to_lidar, robots, i, world, random_generator, index_begin, index_end);
            }
            else if (index_begin <= index_end)
            {
                sweepLidarWindow(lidar, world_to_lidar, robots, i, world, random_generator, index_begin, index_end);

                // a single window covering the whole rotation (e.g. a step
                // spanning a full period) finishes it; publish the scan
                if (rotation_completed)
                {
                    lidar.publishScan();
                }
            }
            else
            {
                // the window straddles the seam: finish the current rotation and
                // publish it before the next rotation overwrites the first rays
                sweepLidarWindow(lidar, world_to_lidar, robots, i, world, random_generator, index_begin, num_rays);
                lidar.publishScan();
                sweepLidarWindow(lidar, world_to_lidar, robots, i, world, random_generator, 0, index_end);
            }
        }
    }
}

void lidarWithWorld(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &pose_robot, const murosim::geometry::World &world)
{
    lidarInFrameWithWorld(lidar, (pose_robot * lidar.getPose()).inverse(), world);
}

void lidarWithShape(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &pose_robot, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose_shape)
{
    lidarInFrameWithShape(lidar, (pose_robot * lidar.getPose()).inverse(), shape, pose_shape);
}

void lidarInFrameWithWorld(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::World &world)
{
    lidarInFrameWithWorld(lidar, world_to_lidar, world, 0, lidar.getLaserSize());
}

void lidarInFrameWithShape(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose_shape)
{
    lidarInFrameWithShape(lidar, world_to_lidar, shape, pose_shape, 0, lidar.getLaserSize());
}

void lidarInFrameWithWorld(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::World &world, const size_t index_begin, const size_t index_end)
{
    // lidar with all lines
    for (size_t i = 0; i < world.getLineSize(); ++i)
    {
        // Transform the line into the coordinate system of the lidar
        const murosim::geometry::Line line = world_to_lidar * world.getLineAt(i);

        laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(), lidar.getArrayLaserDistanceBuffer(), line.getPoint1X(), line.getPoint1Y(), line.getPoint2X(), line.getPoint2Y(), index_begin, index_end);
    }

    // lidar with all circles
    for (size_t i = 0; i < world.getCircleSize(); ++i)
    {
        // Transform the circle into the coordinate system of the lidar
        const murosim::geometry::Circle circle = world_to_lidar * world.getCircleAt(i);

        laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(), lidar.getArrayLaserDistanceBuffer(), circle.getX(), circle.getY(), circle.getRadius(), index_begin, index_end);
    }
}

void lidarInFrameWithShape(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose_shape, const size_t index_begin, const size_t index_end)
{
    // Transformation from the shape frame into the lidar frame
    const Eigen::Affine2f transformation = world_to_lidar * pose_shape;

    // lidar with all lines
    for (const murosim::geometry::Line &l : shape.getLines())
    {
        // Transform the line into the coordinate system of the lidar
        const murosim::geometry::Line line = transformation * l;

        laserWithLine(lidar.getArrayLaserX(), lidar.getArrayLaserY(), lidar.getArrayLaserDistanceBuffer(), line.getPoint1X(), line.getPoint1Y(), line.getPoint2X(), line.getPoint2Y(), index_begin, index_end);
    }

    // lidar with all circles
    for (const murosim::geometry::Circle &c : shape.getCircles())
    {
        // Transform the circle into the coordinate system of the lidar
        const murosim::geometry::Circle circle = transformation * c;

        laserWithCircle(lidar.getArrayLaserX(), lidar.getArrayLaserY(), lidar.getArrayLaserDistanceBuffer(), circle.getX(), circle.getY(), circle.getRadius(), index_begin, index_end);
    }
}

} // namespace murosim::collision
