// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_LIDAR_POINT_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_LIDAR_POINT_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Cartesian lidar hit points (x, y) for one lidar sensor, in metres. Feature
/// shape per stacked state is [2, num_lasers] (x row, then y row). Not
/// normalized. Readings are clamped to the lidar's [range_min, range_max]
/// first (Lidar::clampDistance), so a ray without a return (+inf) gives the
/// point at range_max and every point is finite. When `local` is true,
/// points are given as distance times unit
/// beam direction in the lidar's own frame (NOT the robot frame unless the
/// lidar is mounted at the robot origin with zero rotation); otherwise points
/// are transformed through the lidar's mounting pose and the robot's pose
/// into world coordinates. Reads State::getLidarDistanceAt(index_lidar) and
/// the lidar's beam directions/mounting pose for each stacked state. All
/// robots must have the same lidar configuration at `index_lidar` (checked by
/// validate()/shape() via util::robot::requireConsistentLidars). Internally
/// parallelizes compute() over robots using up to num_threads std::threads
/// (0 maps to hardware_concurrency); computeRange() (used by ObservationSet)
/// stays single-threaded since the caller's thread pool already parallelizes.
class LidarPoint : public ObservationRobot
{
private:
    size_t index_lidar_;
    bool local_;
    size_t num_threads_;

    /// Write hit points for robots [begin, end) into `array` (offset internally by begin).
    void fillObservation(const size_t begin, const size_t end, float* array,
                         const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots,
                         const size_t stack_size, const size_t size_lidar) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

    /// Require the base robot/state checks plus consistent lidar configuration at `index_lidar`.
    virtual void validate(const EntityList &robots) const override;

public:
    /// @param index_lidar Index into each robot's lidar list.
    /// @param local Report points in the lidar's own frame instead of world coordinates.
    /// @param stack_size Number of stacked past readings per robot; 0 and 1 are equivalent (no stacking).
    /// @param num_threads Worker threads used inside compute(); 0 uses hardware_concurrency().
    LidarPoint(const size_t index_lidar, const bool local,
               const size_t stack_size, const size_t num_threads = 1);

    /// Per-robot feature shape is [2, num_lasers] per stacked state; requires consistent lidars.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;

    /// Range path for ObservationSet: no internal thread spawning — the
    /// caller's pool provides the parallelism.
    virtual void computeRange(const EntityList &robots, const size_t begin,
                              const size_t end, float* output) const override;

    /// Index into each robot's lidar list this observation reads.
    size_t indexLidar() const noexcept { return index_lidar_; }
    /// Whether points are reported in the lidar's own frame instead of world coordinates.
    bool local() const noexcept { return local_; }
    /// Configured worker thread count for the internal compute() path.
    size_t numThreads() const noexcept { return num_threads_; }
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_LIDAR_POINT_HPP_
