// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_COLLISION_LIDAR_HPP_
#define MUROSIM_COLLISION_LIDAR_HPP_

#include "murosim/sensor/lidar.hpp"
#include "murosim/geometry/world.hpp"
#include "murosim/geometry/shape.hpp"
#include "murosim/robot/robot.hpp"
#include "murosim/random/random_generator.hpp"

namespace murosim::collision
{
    /// Performs one full, instantaneous lidar update (SYNCHRONIZED mode): for
    /// every non-finished robot's lidars, resets all rays to +inf (no hit),
    /// casts every ray against the world and every other robot's shape,
    /// applies the range limits (+inf/-inf, REP 117) and the sensor noise, and
    /// publishes the completed scan.
    void lidarSweepComplete(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const murosim::geometry::World &world, murosim::random::RandomGenerator &random_generator);
    /** \brief Advance every robot's CONTINUOUS-mode lidars by dt seconds,
     * recomputing only the rays the beam sweeps during that interval. Each
     * lidar keeps its own scan phase, so the scan continues across calls. */
    void lidarSweepTime(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, const murosim::geometry::World &world, murosim::random::RandomGenerator &random_generator, const float dt);
    /// Casts every ray of lidar (mounted on a robot at pose_robot) against
    /// every line and circle in world, lowering each ray in lidar's distance
    /// buffer to its nearest hit (start from Lidar::fillLaserDistanceWithInf,
    /// finish with Lidar::applyNoise).
    void lidarWithWorld(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &pose_robot, const murosim::geometry::World &world);
    /// Casts every ray of lidar (mounted on a robot at pose_robot) against
    /// shape (posed at pose_shape), updating lidar's distance buffer.
    void lidarWithShape(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &pose_robot, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose_shape);

    /** \brief Like lidarWithWorld/lidarWithShape, but with the transformation
     * into the lidar frame ((pose_robot * lidar.getPose()).inverse())
     * precomputed by the caller, so it is inverted once per lidar instead of
     * once per obstacle. The ranged overloads update exactly the rays in
     * [index_begin, index_end); rays outside the window are never written. */
    void lidarInFrameWithWorld(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::World &world);
    void lidarInFrameWithShape(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose_shape);
    void lidarInFrameWithWorld(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::World &world, const size_t index_begin, const size_t index_end);
    void lidarInFrameWithShape(murosim::sensor::Lidar &lidar, const Eigen::Affine2f &world_to_lidar, const murosim::geometry::Shape &shape, const Eigen::Affine2f &pose_shape, const size_t index_begin, const size_t index_end);
} // namespace murosim::collision

#endif // MUROSIM_COLLISION_LIDAR_HPP_
