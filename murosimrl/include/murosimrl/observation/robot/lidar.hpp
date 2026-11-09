// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMRL_OBSERVATION_ROBOT_LIDAR_HPP_
#define MUROSIMRL_OBSERVATION_ROBOT_LIDAR_HPP_

#include "murosimrl/observation/observation_robot.hpp"

namespace murosimrl::observation::robot
{

/// Lidar range readings for one lidar sensor, in metres. Feature shape per
/// stacked state is [num_lasers]. Readings are clamped to [lower, upper],
/// where lower is the robot's own Lidar::getRangeMin() and upper is
/// range_max if positive, else the robot's own Lidar::getRangeMax(); so +inf
/// (no return) reads upper and -inf (too close) reads lower, and the output
/// is always finite. With normalize, (reading - lower) / (upper - lower)
/// maps lower to 0 and upper to 1 per robot, so fleets with different lidar
/// ranges still normalize to [0, 1]. Reads State::getLidarDistanceAt(index_lidar)
/// for each stacked state. All robots must have the same lidar configuration
/// at `index_lidar` (checked by validate()/shape() via
/// util::robot::requireConsistentLidars). Internally parallelizes compute()
/// over robots using up to num_threads std::threads (0 maps to
/// hardware_concurrency); computeRange() (used by ObservationSet) stays
/// single-threaded since the caller's thread pool already parallelizes.
class Lidar : public ObservationRobot
{
private:
    size_t index_lidar_;
    float range_max_;
    bool normalize_;
    size_t num_threads_;

    /// Write range readings for robots [begin, end) into `array` (offset internally by begin).
    void fillObservation(const size_t begin, const size_t end, float* array,
                         const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots,
                         const size_t stack_size, const size_t size_lidar) const;

protected:
    virtual void compute(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots, float* output) const override;

    /// Require the base robot/state checks plus consistent lidar configuration
    /// at `index_lidar`, and a positive range_max above every lidar's range_min.
    virtual void validate(const EntityList &robots) const override;

public:
    /// @param index_lidar Index into each robot's lidar list.
    /// @param range_max Upper clamp distance in metres; 0 uses each robot's own sensor range_max. Clamped to >= 0.
    /// @param normalize Scale readings to [0, 1]: the lidar's range_min maps to 0, the upper clamp distance to 1.
    /// @param stack_size Number of stacked past readings per robot; 0 and 1 are equivalent (no stacking).
    /// @param num_threads Worker threads used inside compute(); 0 uses hardware_concurrency().
    Lidar(const size_t index_lidar, const float range_max, const bool normalize,
          const size_t stack_size, const size_t num_threads = 1);

    /// Per-robot feature shape is [num_lasers] per stacked state; requires consistent lidars.
    virtual std::vector<size_t> shape(const std::vector<std::shared_ptr<murosim::robot::Robot>> &robots) const override;

    /// Range path for ObservationSet: no internal thread spawning — the
    /// caller's pool provides the parallelism.
    virtual void computeRange(const EntityList &robots, const size_t begin,
                              const size_t end, float* output) const override;

    /// Index into each robot's lidar list this observation reads.
    size_t indexLidar() const noexcept { return index_lidar_; }
    /// Configured upper clamp distance in metres (0 means "each robot's own sensor range_max").
    float rangeMax() const noexcept { return range_max_; }
    /// Whether readings are scaled to [0, 1].
    bool normalize() const noexcept { return normalize_; }
    /// Configured worker thread count for the internal compute() path.
    size_t numThreads() const noexcept { return num_threads_; }
};

} // namespace murosimrl::observation::robot

#endif // MUROSIMRL_OBSERVATION_ROBOT_LIDAR_HPP_
