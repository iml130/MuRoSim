// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/sensor/lidar.hpp"
#include "murosim/math/constants.hpp"
#include "murosim/random/distribution.hpp"
#include <numbers>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace murosim::sensor
{


Lidar::Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max):
    Lidar(pose, fov, resolution, range_min, range_max, -1.0f, NOISE_TYPE::NONE, 0.0f, murosim::math::PI, false)
{
}

Lidar::Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max, const float frequency):
    Lidar(pose, fov, resolution, range_min, range_max, frequency, NOISE_TYPE::NONE, 0.0f, murosim::math::PI, false)
{
}

Lidar::Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max, const float frequency, const NOISE_TYPE noise_type, const float noise):
    Lidar(pose, fov, resolution, range_min, range_max, frequency, noise_type, noise, murosim::math::PI, false)
{
}

Lidar::Lidar(const Eigen::Affine2f& pose, const float fov, const size_t resolution, const float range_min, const float range_max, const float frequency, const NOISE_TYPE noise_type, const float noise, const float angle_start, const bool rotation_clockwise):
    pose_(pose),
    angle_step_(0.0f),
    angle_min_(0.0f),
    angle_max_(0.0f),
    angle_start_(angle_start),
    range_min_(range_min),
    range_max_(range_max),
    frequency_(frequency),
    noise_(noise),
    noise_type_(noise_type),
    rotation_clockwise_(rotation_clockwise),
    scan_time_(0.0f)
{
    // clampDistance() and the normalizing consumers divide by
    // range_max - range_min, so the range must be a proper interval
    if (!(std::isfinite(range_min) && std::isfinite(range_max) && 0.0f <= range_min && range_min < range_max))
    {
        throw std::invalid_argument("Lidar range must be finite with 0 <= range_min < range_max (got range_min " +
                                    std::to_string(range_min) + ", range_max " + std::to_string(range_max) + ").");
    }

    createLaser(resolution, fov);
}

void Lidar::createLaser(const size_t resolution, const float fov)
{
    fov_ = fov;

    if (resolution == 0)
    {
        throw std::invalid_argument("Lidar resolution must be greater than 0.");
    }

    std::vector<Eigen::Vector2d> vector_laser;

    if (resolution == 1)
    {
        angle_min_ = 0.0f;
        angle_max_ = 0.0f;
        angle_step_ = 0.0f;

        vector_laser.emplace_back(Eigen::Vector2d(1.0, 0.0));
    }
    else
    {
        // Rays are stored in physical scan order so the time-based partial
        // sweep (CONTINUOUS mode) can map time linearly onto indices:
        // counter-clockwise rotation walks the angles upward, clockwise
        // rotation downward (angle_step_ becomes negative).
        //
        // A full circle has no edge to measure twice: its rays split the turn
        // into `resolution` equal steps (as ROS drivers do), starting at the
        // scan seam angle_start_ (default pi, the -pi/+pi direction). A
        // smaller FOV measures both of its edges, so its rays split it into
        // resolution - 1 steps, and the scan starts at an edge.
        const bool full_circle = isFullCircle(fov, resolution);

        // the turn and the seam reference use the float pi of the default
        // angle_start, so the first ray of the default layout points exactly
        // at -fov/2 of a 2*pi FOV
        const double half_turn = static_cast<double>(murosim::math::PI);
        const double angle_step = full_circle
            ? 2.0 * half_turn / static_cast<double>(resolution)
            : static_cast<double>(fov) / static_cast<double>(resolution - 1);
        const double offset = full_circle ? (static_cast<double>(angle_start_) - half_turn) : 0.0;
        const double half_span = full_circle ? half_turn : static_cast<double>(fov) * 0.5;

        const double angle_first = offset + (rotation_clockwise_ ? half_span : -half_span);
        const double angle_step_signed = rotation_clockwise_ ? -angle_step : angle_step;

        // angle_first + i * step rather than a running sum, which drifts: a ray
        // on an axis (ray 180 of 360 at 0 rad) must point exactly along it
        for (size_t i = 0; i < resolution; i++)
        {
            const double angle = angle_first + angle_step_signed * static_cast<double>(i);
            vector_laser.emplace_back(Eigen::Rotation2Dd(angle) * Eigen::Vector2d(1.0, 0.0));
        }

        angle_min_ = static_cast<float>(angle_first);
        angle_max_ = static_cast<float>(angle_first + angle_step_signed * static_cast<double>(resolution - 1));
        angle_step_ = static_cast<float>(angle_step_signed);
    }

    array_laser_x_ = murosim::simd::ArrayFloat(vector_laser.size());
    array_laser_y_ = murosim::simd::ArrayFloat(vector_laser.size());
    array_laser_distance_ = murosim::simd::ArrayFloat(vector_laser.size());
    array_laser_distance_buffer_ = murosim::simd::ArrayFloat(vector_laser.size());

    for (size_t i = 0; i < vector_laser.size(); ++i)
    {
        const Eigen::Vector2d &vec = vector_laser[i];

        array_laser_x_[i] = static_cast<float>(vec.x());
        array_laser_y_[i] = static_cast<float>(vec.y());
        // both the work buffer and the published scan start at +inf, so
        // consumers see "no return" until the first rotation completes
        array_laser_distance_[i] = std::numeric_limits<float>::infinity();
        array_laser_distance_buffer_[i] = std::numeric_limits<float>::infinity();
    }
}

bool Lidar::isFullCircle(const float fov, const size_t resolution)
{
    // A single ray (resolution < 2) has no scan seam; createLaser special-cases
    // it and never treats it as a full circle.
    if (resolution < 2)
    {
        return false;
    }

    // Within half a full-circle ray step of 2*pi the FOV is a full turn: its
    // rays are spread evenly over the whole circle, and the beam measures
    // during the whole rotation. A larger FOV cannot cover more than the
    // circle, so it is a full turn as well.
    const double full_circle_step = 2.0 * std::numbers::pi / static_cast<double>(resolution);
    return (2.0 * std::numbers::pi - static_cast<double>(fov)) < (full_circle_step * 0.5);
}

bool Lidar::indexWindowForTimeInterval(const float time_begin, const float time_end, size_t &index_begin, size_t &index_end) const
{
    const float frequency = frequency_;
    const size_t num_rays = array_laser_distance_.size();

    if (frequency <= 0.0f)
    {
        throw std::runtime_error("Lidar frequency must be positive for time-based index calculation");
    }

    if (num_rays == 0)
    {
        throw std::runtime_error("Lidar has no rays");
    }

    // Time for one complete rotation
    const float scan_duration = 1.0f / frequency;

    // An interval of at least one rotation sweeps every ray
    if ((time_end - time_begin) >= scan_duration)
    {
        index_begin = 0;
        index_end = num_rays;
        return false;
    }

    // The measuring window: the beam covers the FOV at the start of each
    // rotation and spends the rest of the period in the dead zone. A FOV laid
    // out as a seamless full circle (isFullCircle, shared with createLaser) has
    // no dead zone, so its trailing rays keep updating.
    constexpr float TWO_PI = murosim::math::TWO_PI;

    float measuring_duration = scan_duration;
    if (num_rays >= 2 && 0.0f < fov_ && !isFullCircle(fov_, num_rays))
    {
        measuring_duration = scan_duration * (fov_ / TWO_PI);
    }

    const float rays_per_second = static_cast<float>(num_rays) / measuring_duration;

    // Normalize the interval into the current rotation; the end may reach
    // into the next rotation
    const float phase_begin = std::fmod(time_begin, scan_duration);
    const float phase_end = phase_begin + (time_end - time_begin);

    // First measured ray of the segment inside the current rotation's window
    // (the segment end is only needed for the non-wrapping case, computed below)
    size_t segment_begin = 0;
    if (phase_begin < measuring_duration)
    {
        segment_begin = std::min(static_cast<size_t>(phase_begin * rays_per_second), num_rays - 1);
    }

    // Wrap into the next rotation's measuring window
    size_t wrapped_end = 0;
    if (phase_end > scan_duration)
    {
        const float phase_wrapped = phase_end - scan_duration;
        wrapped_end = (phase_wrapped >= measuring_duration)
            ? num_rays
            : std::min(static_cast<size_t>(phase_wrapped * rays_per_second), num_rays);
    }

    if (phase_end > scan_duration && phase_begin < measuring_duration)
    {
        // covers the end of this scan and the start of the next one:
        // wrapped window [segment_begin, num_rays) + [0, wrapped_end)
        if (wrapped_end >= segment_begin)
        {
            // quantization closed the gap: everything was swept
            index_begin = 0;
            index_end = num_rays;
        }
        else
        {
            index_begin = segment_begin;
            index_end = wrapped_end;
        }
    }
    else if (phase_end > scan_duration)
    {
        // the interval began in the dead zone, only the next scan's start
        // counts: the window belongs entirely to the next rotation
        index_begin = 0;
        index_end = wrapped_end;
        return true;
    }
    else
    {
        // single segment, no wrap; empty (begin == end) when the interval is
        // fully inside the dead zone (phase_begin past the measuring window)
        index_begin = segment_begin;
        if (phase_begin >= measuring_duration)
        {
            index_end = segment_begin; // segment_begin == 0 here -> empty window
        }
        else if (phase_end >= measuring_duration)
        {
            index_end = num_rays;
        }
        else
        {
            index_end = std::min(static_cast<size_t>(phase_end * rays_per_second), num_rays);
        }
    }

    return false;
}

const murosim::simd::ArrayFloat &Lidar::getArrayLaserX() const
{
    return array_laser_x_;
}

const murosim::simd::ArrayFloat &Lidar::getArrayLaserY() const
{
    return array_laser_y_;
}

const murosim::simd::ArrayFloat &Lidar::getArrayLaserDistance() const
{
    return array_laser_distance_;
}

std::ostream& operator<<(std::ostream& os, const Lidar& lidar)
{
    for (size_t i = 0; i < lidar.array_laser_distance_.size(); ++i) {
        os << "(" << lidar.array_laser_x_[i] << ",\t" << lidar.array_laser_y_[i] << "):\t" << lidar.array_laser_distance_[i] << "\n";
    }
    return os;
}

murosim::simd::ArrayFloat &Lidar::getArrayLaserDistanceBuffer()
{
    return array_laser_distance_buffer_;
}

const murosim::simd::ArrayFloat &Lidar::getArrayLaserDistanceBuffer() const
{
    return array_laser_distance_buffer_;
}

void Lidar::publishScan()
{
    // ArrayFloat::operator= reuses its allocation when the sizes match
    array_laser_distance_ = array_laser_distance_buffer_;
}

float Lidar::getAngleMin() const
{
    return angle_min_;
}

float Lidar::getAngleMax() const
{
    return angle_max_;
}

float Lidar::getAngleStep() const
{
    return angle_step_;
}

float Lidar::getAngleStart() const
{
    return angle_start_;
}

float Lidar::getRangeMin() const
{
    return range_min_;
}

float Lidar::getRangeMax() const
{
    return range_max_;
}

float Lidar::getFrequency() const
{
    return frequency_;
}

float Lidar::getScanTime() const
{
    return scan_time_;
}

void Lidar::advanceScanTime(const float dt)
{
    if (frequency_ <= 0.0f)
    {
        return;
    }

    const float scan_duration = 1.0f / frequency_;
    scan_time_ = std::fmod(scan_time_ + dt, scan_duration);

    // dt is expected non-negative, but fmod of a negative argument is negative;
    // fold it back into [0, scan_duration) so the phase stays canonical.
    if (scan_time_ < 0.0f)
    {
        scan_time_ += scan_duration;
    }
}

void Lidar::reset()
{
    scan_time_ = 0.0f;
    fillLaserDistanceWithInf();
    publishScan();
}

float Lidar::getNoise() const
{
    return noise_;
}

Lidar::NOISE_TYPE Lidar::getNoiseType() const
{
    return noise_type_;
}

const Eigen::Affine2f &Lidar::getPose() const
{
    return pose_;
}

bool Lidar::getRotationClockwise() const
{
    return rotation_clockwise_;
}

void Lidar::fillLaserDistanceWithInf()
{
    fillLaserDistanceWithInf(0, array_laser_distance_buffer_.size());
}

void Lidar::fillLaserDistanceWithInf(const size_t index_begin, const size_t index_end)
{
    assert(index_begin <= index_end && index_end <= array_laser_distance_buffer_.size() && "Ray window out of bounds in fillLaserDistanceWithInf");
    std::fill(array_laser_distance_buffer_.data() + index_begin,
              array_laser_distance_buffer_.data() + index_end,
              std::numeric_limits<float>::infinity());
}

float Lidar::reading(const float distance, const float noise) const
{
    constexpr float INF = std::numeric_limits<float>::infinity();

    // a hit outside the range cannot be measured, whatever the noise
    if (distance > range_max_)
    {
        return INF;
    }
    if (distance < range_min_)
    {
        return -INF;
    }

    // noise can push a hit out of the range as well
    const float noisy = distance + noise;
    if (noisy > range_max_)
    {
        return INF;
    }
    if (noisy < range_min_)
    {
        return -INF;
    }
    return noisy;
}

void Lidar::applyNoise(murosim::random::RandomGenerator &random_generator)
{
    applyNoise(random_generator, 0, array_laser_distance_buffer_.size());
}

void Lidar::applyNoise(murosim::random::RandomGenerator &random_generator, const size_t index_begin, const size_t index_end)
{
    assert(index_begin <= index_end && index_end <= array_laser_distance_buffer_.size() && "Ray window out of bounds in applyNoise");

    float *distance = array_laser_distance_buffer_.data();

    // Noise is drawn for every ray, also for rays without a hit, so the
    // random sequence does not depend on the scene.
    switch (noise_type_)
    {
    case NOISE_TYPE::NONE:
        for (size_t i = index_begin; i < index_end; ++i)
        {
            distance[i] = reading(distance[i], 0.0f);
        }
        break;

    case NOISE_TYPE::NORMAL:
        {
            murosim::random::NormalDistribution distribution(0.0f, noise_);

            for (size_t i = index_begin; i < index_end; ++i)
            {
                distance[i] = reading(distance[i], distribution(random_generator));
            }
        }
        break;

    case NOISE_TYPE::UNIFORM:
        for (size_t i = index_begin; i < index_end; ++i)
        {
            distance[i] = reading(distance[i], murosim::random::uniformFloat(random_generator, -noise_, noise_));
        }
        break;
    }
}

float Lidar::clampDistance(const float distance) const
{
    return std::clamp(distance, range_min_, range_max_);
}

float Lidar::getLaserX(const size_t i) const
{
    assert(i < array_laser_x_.size() && "Index out of bounds in getLaserX");
    return array_laser_x_[i];
}

float Lidar::getLaserY(const size_t i) const
{
    assert(i < array_laser_y_.size() && "Index out of bounds in getLaserY");
    return array_laser_y_[i];
}

float Lidar::getLaserDistance(const size_t i) const
{
    assert(i < array_laser_distance_.size() && "Index out of bounds in getLaserDistance");
    return array_laser_distance_[i];
}

size_t Lidar::getLaserSize() const
{
    return array_laser_distance_.size();
}

Eigen::Vector2f Lidar::getLaserDirection(const size_t i) const
{
    assert(i < array_laser_x_.size() && "Index out of bounds in getLaserDirection");
    return Eigen::Vector2f(array_laser_x_[i], array_laser_y_[i]);
}

Eigen::Vector2f Lidar::getLaserPoint(const size_t i) const
{
    assert(i < array_laser_distance_.size() && "Index out of bounds in getLaserPoint");
    return clampDistance(array_laser_distance_[i]) * Eigen::Vector2f(array_laser_x_[i], array_laser_y_[i]);
}

} // namespace murosim::sensor
