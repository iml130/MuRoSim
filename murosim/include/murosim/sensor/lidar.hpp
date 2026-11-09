// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_SENSOR_LIDAR_HPP_
#define MUROSIM_SENSOR_LIDAR_HPP_

#include "murosim/simd/array_float.hpp"
#include "murosim/random/random_generator.hpp"

#include <ostream>
#include <Eigen/Geometry>

namespace murosim::sensor
{

/// Simulated 2D rotating range sensor mounted at a fixed pose on a robot.
/// Holds the ray geometry (angles/directions) and the distance measurements
/// produced by the collision sweep, plus optional measurement noise. Supports
/// both instantaneous full-sweep scans (SYNCHRONIZED mode) and scans that
/// build up incrementally as the beam rotates over multiple simulation steps
/// (CONTINUOUS mode), see indexWindowForTimeInterval().
///
/// Readings follow ROS REP 117: a ray that hits nothing within range_max
/// reads +inf (no return), and a hit closer than range_min reads -inf (too
/// close); every finite reading lies in [range_min, range_max].
/// clampDistance() maps a reading onto that range for consumers that need
/// finite values.
class Lidar
{
public:
    /// Noise model applied to measured distances by applyNoise().
    enum class NOISE_TYPE{
        NONE,       ///< No noise applied.
        UNIFORM,    ///< Uniform noise in [-noise, noise].
        NORMAL      ///< Gaussian noise with standard deviation `noise`.
    };
private:
    // Ray unit directions in the lidar frame (index = physical scan order).
    murosim::simd::ArrayFloat array_laser_x_;
    murosim::simd::ArrayFloat array_laser_y_;

    // Last fully completed scan: the published distances every consumer reads
    // (State, rewards, visualization, Python bindings). In CONTINUOUS mode it is
    // refreshed only when the beam crosses the scan seam (one full rotation); in
    // SYNCHRONIZED mode it is refreshed after each complete sweep (every step).
    murosim::simd::ArrayFloat array_laser_distance_;

    // Work-in-progress scan the collision sweep writes into. CONTINUOUS-mode
    // partial sweeps update only the ray window the beam covered this step, so
    // mid-rotation this buffer mixes freshly measured rays with the previous
    // rotation's. publishScan() copies it into array_laser_distance_ once the
    // rotation completes.
    murosim::simd::ArrayFloat array_laser_distance_buffer_;

    /**
     * @brief pose_ The pose of the lidar.
     */
    Eigen::Affine2f pose_;

    /**
     * Rays are stored in physical scan order: index i is the i-th ray the
     * beam measures within one period, and the angle of ray i is
     * angle_min_ + i * angle_step_. For clockwise rotation the rays are
     * generated in reversed order, so angle_step_ is negative and
     * angle_min_ (the FIRST ray's angle) is greater than angle_max_ (the
     * LAST ray's angle).
     */

    /**
     * @brief angle_step Signed angle between consecutive rays (negative for clockwise rotation).
     */
    float angle_step_;
    /**
     * @brief angle_min Angle of the first measured ray.
     */
    float angle_min_;
    /**
     * @brief angle_max Angle of the last measured ray.
     */
    float angle_max_;

    /**
     * @brief angle_start Angle of the scan seam for full-circle lidars
     * (the first ray points here; default pi, the -pi/+pi direction).
     * Ignored for smaller FOVs, where the scan starts at a FOV edge.
     */
    float angle_start_;

    /**
     * @brief fov The field of view covered by the rays.
     */
    float fov_;
    /**
     * @brief range_min Minimum range of the lidar.
     */
    float range_min_;
    /**
     * @brief range_max Maximum range of the lidar.
     */
    float range_max_;

    float frequency_;
    /**
     * @brief noise Noise magnitude in metres: the standard deviation for
     * NORMAL, the half-width for UNIFORM.
     */
    float noise_;
    /**
     * @brief noise_type Measurement noise model.
     */
    NOISE_TYPE noise_type_;

    bool rotation_clockwise_;

    /** \brief Persistent scan phase in [0, 1/frequency_): where the beam
     * currently is within its rotation. CONTINUOUS-mode partial sweeps advance
     * it by the elapsed interval, so it stays bounded (the time->index mapping
     * never loses float precision) and the scan continues across simulate()
     * calls instead of restarting from ray 0 each step. */
    float scan_time_;

    void createLaser(const size_t resolution, const float fov);

    /** \brief The reading of a ray whose nearest hit is `distance` (+inf when
     * nothing was hit), disturbed by `noise`: outside [range_min, range_max]
     * it is -inf or +inf (REP 117), and the noisy value is checked again. */
    float reading(const float distance, const float noise) const;

    /** \brief Whether a FOV is laid out as a seamless full circle: within half
     * a full-circle ray step (2*pi / resolution) of 2*pi, or larger, the rays
     * are spread evenly over the whole rotation with no dead zone and no ray
     * measured twice. The single source of truth shared by createLaser (ray
     * placement) and indexWindowForTimeInterval (time mapping). */
    static bool isFullCircle(const float fov, const size_t resolution);

public:
    /// Constructs a lidar with `resolution` rays spread over `fov` radians, no
    /// noise, single-sweep-per-step (non-continuous) scanning.
    /// @param pose Sensor pose relative to the robot frame.
    /// @param fov Field of view in radians. A full turn (2*pi) spreads the rays
    /// evenly around the circle, 2*pi / resolution apart; a smaller FOV
    /// measures both of its edges, fov / (resolution - 1) apart.
    /// @param resolution Number of rays; must be greater than 0.
    /// @param range_min Minimum measurable distance (metres); closer hits read -inf.
    /// @param range_max Maximum measurable distance (metres); rays that hit
    /// nothing within it read +inf.
    /// @throws std::invalid_argument if resolution is 0 or the range is not
    /// finite with 0 <= range_min < range_max.
    Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max);
    /// As above, additionally rotating at `frequency` Hz; a non-positive
    /// frequency disables the time-based partial-sweep behaviour.
    Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max, const float frequency);
    /// As above, applying `noise_type` measurement noise of magnitude `noise`.
    Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max, const float frequency, const NOISE_TYPE noise_type, const float noise);
    /// As above, with the scan seam placed at `angle_start` (radians) and the
    /// rotation direction set by `rotation_clockwise`.
    Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max, const float frequency, const NOISE_TYPE noise_type, const float noise, const float angle_start, const bool rotation_clockwise);

    /// X components of the unit ray directions, in scan order, lidar frame.
    const murosim::simd::ArrayFloat& getArrayLaserX() const;
    /// Y components of the unit ray directions, in scan order, lidar frame.
    const murosim::simd::ArrayFloat& getArrayLaserY() const;
    /** \brief The published last-complete-rotation scan (what consumers read). */
    const murosim::simd::ArrayFloat& getArrayLaserDistance() const;
    /** \brief The work-in-progress scan the collision sweep writes into. */
    murosim::simd::ArrayFloat& getArrayLaserDistanceBuffer();
    const murosim::simd::ArrayFloat& getArrayLaserDistanceBuffer() const;
    /** \brief Copy the work-in-progress buffer into the published complete scan.
     * Called by the collision sweep when a rotation finishes (CONTINUOUS) or
     * after each complete sweep (SYNCHRONIZED). */
    void publishScan();

    /// Reset the whole work-in-progress buffer to +inf (nothing hit yet), the
    /// start value the laser kernels lower to the nearest hit.
    void fillLaserDistanceWithInf();
    /// Turn the nearest hits in the whole work-in-progress buffer into
    /// readings: hits outside [range_min, range_max] become -inf or +inf;
    /// the others get NOISE_TYPE noise drawn from `random_generator`, and a
    /// noisy value outside the range becomes -inf or +inf as well. Runs for
    /// NOISE_TYPE::NONE too (range only); noise is drawn for every ray, so the
    /// random sequence does not depend on the scene.
    void applyNoise(murosim::random::RandomGenerator &random_generator);
    /** \brief Like above, but only for the rays in [index_begin, index_end). */
    void fillLaserDistanceWithInf(const size_t index_begin, const size_t index_end);
    /// Like applyNoise(random_generator), but limited to [index_begin, index_end).
    void applyNoise(murosim::random::RandomGenerator &random_generator, const size_t index_begin, const size_t index_end);

    /// `distance` clamped to [range_min, range_max]: +inf (no return) maps to
    /// range_max and -inf (too close) to range_min, so the result is finite.
    float clampDistance(const float distance) const;

    /** \brief Map a time interval [time_begin, time_end) onto the half-open ray
     * index window [index_begin, index_end) the beam measures during it.
     *
     * Rays are stored in physical scan order, so time maps linearly onto index.
     * For fov < full-circle only the fov/2*pi fraction at the start of each
     * rotation produces measurements; the rest of the period is dead time. The
     * result wraps when index_begin > index_end; index_begin == index_end means
     * no ray was measured (the interval fell entirely in the dead zone).
     *
     * \return true when the interval began in the dead zone and crossed the
     * scan seam: the window then holds only the NEXT rotation's first rays, so
     * callers must publish the completed rotation before sweeping it.
     *
     * \throws std::runtime_error if the frequency is not positive or there are no rays.
     */
    bool indexWindowForTimeInterval(const float time_begin, const float time_end, size_t &index_begin, size_t &index_end) const;

    /** \brief Current scan phase: seconds into the rotation, in [0, 1/frequency_). */
    float getScanTime() const;
    /** \brief Advance the scan phase by dt seconds, wrapping within one rotation
     * period so it stays bounded. No-op when the frequency is not positive. */
    void advanceScanTime(const float dt);

    /** \brief Start over as if just constructed: scan phase 0 and +inf (no
     * return) in the work buffer and the published scan. Robot::reset() calls
     * it, so a new episode never shows rays measured before the reset. */
    void reset();

    /// Angle (radians) of the first measured ray.
    float getAngleMin() const;
    /// Angle (radians) of the last measured ray.
    float getAngleMax() const;
    /// Signed angle (radians) between consecutive rays; negative for clockwise rotation.
    float getAngleStep() const;
    /// Angle (radians) of the scan seam for full-circle lidars.
    float getAngleStart() const;
    /// Minimum measurable distance (metres); closer hits read -inf.
    float getRangeMin() const;
    /// Maximum measurable distance (metres); rays without a hit within it read +inf.
    float getRangeMax() const;
    /// Rotation frequency in Hz; non-positive disables partial-sweep scanning.
    float getFrequency() const;
    /// Noise magnitude used by applyNoise() (meaning depends on NOISE_TYPE).
    float getNoise() const;
    /// Measurement noise model applied to distances.
    NOISE_TYPE getNoiseType() const;
    /// Sensor pose relative to the robot frame.
    const Eigen::Affine2f &getPose() const;
    /// True if rays are generated/scanned in clockwise order.
    bool getRotationClockwise() const;

    /// X component of ray i's unit direction, lidar frame.
    float getLaserX(const size_t i) const;
    /// Y component of ray i's unit direction, lidar frame.
    float getLaserY(const size_t i) const;
    /// Last published reading (metres) of ray i; +inf (no return) or -inf (too close) outside the range.
    float getLaserDistance(const size_t i) const;
    /// Number of rays.
    size_t getLaserSize() const;
    /// Unit direction of ray i, lidar frame.
    Eigen::Vector2f getLaserDirection(const size_t i) const;
    /// Endpoint of ray i (direction scaled by its reading), lidar frame. The
    /// reading is clamped first (clampDistance()), so the point is always
    /// finite: a ray without a return ends at range_max.
    Eigen::Vector2f getLaserPoint(const size_t i) const;

    friend std::ostream& operator<<(std::ostream& os, const Lidar& lidar);
};

/// Prints each ray as "(x,\ty):\tdistance" per line.
std::ostream& operator<<(std::ostream& os, const Lidar& lidar);

} // namespace murosim::sensor

#endif // MUROSIM_SENSOR_LIDAR_HPP_
