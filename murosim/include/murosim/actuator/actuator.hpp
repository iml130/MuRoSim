// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_ACTUATOR_ACTUATOR_HPP_
#define MUROSIM_ACTUATOR_ACTUATOR_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <ostream>

namespace murosim::actuator
{

/** \brief Control mode for actuator commands.
 *
 * VELOCITY: RL sets target velocity, system accelerates toward it using acceleration_max_
 * ACCELERATION: RL sets acceleration directly, applied to velocity each timestep
 */
enum class CONTROL_MODE {
    VELOCITY,
    ACCELERATION
};

/** \brief Base class for all actuators. */
class Actuator
{
protected:

    Eigen::Vector3f velocity_current_;
    Eigen::Vector3f velocity_min_;
    Eigen::Vector3f velocity_max_;
    Eigen::Vector3f acceleration_max_;
    Eigen::Vector3f command_velocity_;
    Eigen::Vector3f command_acceleration_;
    CONTROL_MODE control_mode_;

    /** \brief Apply the current control mode to a velocity, per axis.
     *
     * VELOCITY: accelerate toward command_velocity_ limited by acceleration_max_
     * (axes without an acceleration limit snap to the command).
     * ACCELERATION: integrate command_acceleration_, clamped to the velocity limits.
     */
    Eigen::Vector3f velocityFromControlMode(const Eigen::Vector3f &velocity_current, const float dt) const;

    /** \brief Zero the axes the actuator TYPE does not support (X_ANGULAR has
     * no y, X_Y has no angular; an unknown type zeroes everything). The single
     * place the per-TYPE axis capability lives — command clamping and velocity
     * limit correction both apply it.
     */
    void zeroUnsupportedAxes(Eigen::Vector3f &vector) const;

    /** \brief Build an incremental pose delta from a body-frame velocity:
     * translate by velocity*dt, then rotate by the angular velocity*dt
     * (geometry::primitives::pose), so every actuator integrates motion
     * identically.
     */
    static Eigen::Affine2f poseDelta(const Eigen::Vector3f &velocity, const float dt);

    /** \brief Constructor helpers establishing the limit invariants every
     * actuator must uphold on an axis (0 = linear x, 1 = linear y,
     * 2 = angular): velocity_min_ <= velocity_max_ and a non-negative
     * acceleration. setCommandVelocity clamps against these ranges, and
     * std::clamp with an inverted range is undefined behavior — so all
     * constructors sanitize through this single place.
     */
    void setVelocityLimits(const size_t axis, const float min, const float max);
    /** \brief Symmetric velocity range -max(0, limit) .. +max(0, limit). */
    void setVelocityLimitsSymmetric(const size_t axis, const float limit);
    void setAccelerationLimit(const size_t axis, const float max);
    /** \brief Like setAccelerationLimit, additionally capped at `cap` (for
     * actuators modeling a fixed physical maximum). */
    void setAccelerationLimit(const size_t axis, const float max, const float cap);

    /** \brief Advance the actuator by dt and return the realized body-frame
     * velocity (linear x, linear y, angular). step() integrates this into the
     * pose via poseDelta(), so implementations model kinematics only and never
     * build a transform. Called by step() only after it guaranteed dt > 0, so
     * implementations need not guard against a non-positive or NaN timestep.
     */
    virtual Eigen::Vector3f move(const float dt) = 0;
public:
    /// Default-constructs an actuator with zeroed velocities/accelerations and CONTROL_MODE::VELOCITY.
    Actuator();
    /// Which body-frame axes an actuator implementation actually drives.
    enum class TYPE{
        X_ANGULAR,    ///< Differential-drive style: forward/backward (x) and rotation only, no lateral motion.
        X_Y_ANGULAR,  ///< Fully holonomic: independent x, y and angular velocity.
        X_Y           ///< Omnidirectional translation only, no rotation.
    };

    /** \brief Map a value between its absolute range [min, max] and the
     * relative range [-1, 1] (linear; a zero-width absolute range maps to 0).
     * Also used by murosimrl to normalize velocity observations and actions.
     */
    static float fromRelativeToAbsolute(const float relative, const float absolute_min, const float absolute_max);
    /// Inverse of fromRelativeToAbsolute: maps `absolute` in [absolute_min, absolute_max] to [-1, 1]. Returns 0 if the range has zero width.
    static float fromAbsoluteToRelative(const float absolute, const float absolute_min, const float absolute_max);

    /// Sets the commanded body-frame velocity, clamped to [velocity_min_, velocity_max_] per axis and zeroed on axes the concrete TYPE does not support.
    void setCommandVelocity(const Eigen::Vector3f &cmd_vel);
    /// @copydoc setCommandVelocity(const Eigen::Vector3f&)
    void setCommandVelocity(const float x, const float y, const float angular);
    /// Sets the commanded body-frame acceleration, clamped to +-acceleration_max_ per axis and zeroed on axes the concrete TYPE does not support.
    void setCommandAcceleration(const Eigen::Vector3f &cmd_acc);
    /// @copydoc setCommandAcceleration(const Eigen::Vector3f&)
    void setCommandAcceleration(const float x, const float y, const float angular);

    /// Reads the last commanded velocity (x, y, angular) via out-parameters.
    void getCommandVelocity(float &x, float &y, float &angular) const;
    /// @return the last commanded velocity (x, y, angular).
    Eigen::Vector3f getCommandVelocity() const;
    /// Reads the last commanded acceleration (x, y, angular) via out-parameters.
    void getCommandAcceleration(float &x, float &y, float &angular) const;
    /// @return the last commanded acceleration (x, y, angular).
    Eigen::Vector3f getCommandAcceleration() const;

    /// Reads the current realized velocity (x, y, angular) via out-parameters.
    void getVelocity(float &x, float &y, float &angular) const;
    /// @return the current realized velocity (x, y, angular).
    Eigen::Vector3f getVelocity() const;
    /// Reads the per-axis maximum velocity via out-parameters.
    void getVelocityMax(float &x, float &y, float &angular) const;
    /// @return the per-axis maximum velocity.
    Eigen::Vector3f getVelocityMax() const;
    /// Reads the per-axis minimum velocity via out-parameters.
    void getVelocityMin(float &x, float &y, float &angular) const;
    /// @return the per-axis minimum velocity.
    Eigen::Vector3f getVelocityMin() const;
    /// Reads the per-axis maximum acceleration magnitude via out-parameters.
    void getAccelerationMax(float &x, float &y, float &angular) const;
    /// @return the per-axis maximum acceleration magnitude.
    Eigen::Vector3f getAccelerationMax() const;

    /// Selects whether setCommandVelocity or setCommandAcceleration drives motion (see CONTROL_MODE).
    void setControlMode(const CONTROL_MODE mode);
    /// @return the current CONTROL_MODE.
    CONTROL_MODE getControlMode() const;

    /// @return the human-readable name of a TYPE value ("UNKNOWN" if unrecognized).
    static std::string getTypeAsString(const TYPE type);
    /// @return the human-readable name of this actuator's getType().
    std::string getTypeAsString() const;
    /// Parses a TYPE from its name (case-insensitive). @throws std::runtime_error if unrecognized.
    static TYPE getTypeFromString(const std::string type);

    /// @return the human-readable name of a CONTROL_MODE value ("UNKNOWN" if unrecognized).
    static std::string getControlModeAsString(const CONTROL_MODE mode);
    /// @return the human-readable name of this actuator's current control mode.
    std::string getControlModeAsString() const;
    /// Parses a CONTROL_MODE from its name (case-insensitive). @throws std::runtime_error if unrecognized.
    static CONTROL_MODE getControlModeFromString(std::string mode);

    /** \brief Advance the actuator by dt seconds, returning the incremental
     * pose change. A dt of zero is a no-op that returns the identity transform;
     * a negative or NaN dt throws std::invalid_argument. Builds the pose delta
     * from move()'s realized velocity via poseDelta().
     */
    Eigen::Affine2f step(const float dt);

    virtual ~Actuator();
    /// @return a deep copy of this actuator, preserving concrete type, limits and current state.
    virtual std::unique_ptr<Actuator> clone() const = 0;
    /// Resets commanded and realized velocity/acceleration (and any implementation-specific internal state, e.g. wheel speeds) to zero.
    virtual void reset() = 0;

    /// @return which body-frame axes this concrete actuator drives (see TYPE).
    virtual TYPE getType() const = 0;
    /// Writes a human-readable summary of this actuator's type, limits and state to `out`.
    virtual std::ostream& print(std::ostream& out) const = 0;
    /// Streams print() to `os`.
    friend std::ostream& operator<<(std::ostream &os, const Actuator& actuator);
};

} // namespace murosim::actuator

#endif // MUROSIM_ACTUATOR_ACTUATOR_HPP_
