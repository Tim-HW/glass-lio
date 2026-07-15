#ifndef GLASS_CORE_IMU_INIT_HPP
#define GLASS_CORE_IMU_INIT_HPP

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.hpp"

namespace glass_core
{

/// Standard gravity, m/s^2.
inline constexpr double kGravity = 9.80665;

/// One IMU reading, in the engine's own ROS-free vocabulary. Whoever owns the
/// sensor (a ROS node, a bag reader, a test) fills this at the boundary; the
/// engine never sees sensor_msgs. `accel` is raw sensor units -- see accel_scale.
struct ImuSample
{
  Eigen::Vector3d accel;   ///< linear acceleration, raw units (g on Livox)
  Eigen::Vector3d gyro;    ///< angular velocity, rad/s
};

/// Estimates gyro bias and gravity from a STATIC window of IMU samples, and
/// derives the initial gravity-aligned orientation.
///
/// Static detection is the point. Initializing while the sensor is moving folds
/// real rotation into the "bias" and real acceleration into "gravity", and the
/// estimator is then permanently, silently wrong with no error to chase. So a
/// window that fails the motion check is DISCARDED and we keep waiting.
///
/// Units: the Livox driver publishes linear_acceleration in *g*, not m/s^2.
/// `accel_scale` converts the raw ImuSample::accel to SI.
class ImuInit
{
public:
  /// `num_samples`   : samples the static window must contain (200 = 1 s @ 200 Hz)
  /// `max_gyro`      : rad/s; any |w| above this means we are moving
  /// `max_accel_sd`  : m/s^2; per-axis accel std-dev above this means we are moving
  /// `accel_scale`   : multiply raw accel by this to get m/s^2 (9.80665 if in g)
  ImuInit(int num_samples, double max_gyro, double max_accel_sd, double accel_scale);

  /// Feed one sample. Returns true once initialization has completed.
  bool add(const ImuSample & s);

  bool initialized() const {return initialized_;}

  /// Mean angular velocity over the static window (rad/s).
  const Eigen::Vector3d & gyro_bias() const {return gyro_bias_;}

  /// Mean specific force over the static window (m/s^2, IMU frame). At rest this
  /// points along the sensor's "up" axis with magnitude ~9.81.
  const Eigen::Vector3d & gravity() const {return gravity_;}

  /// Rotation IMU -> world, aligning measured gravity with world +Z. Yaw is
  /// unobservable from gravity alone and is left at zero.
  const Sophus::SO3d & initial_rotation() const {return R_wi_;}

  /// How many times a candidate window was rejected for motion.
  int rejected_windows() const {return rejected_;}

  /// Convert a raw sample's accel into SI units (m/s^2).
  Eigen::Vector3d accel_si(const ImuSample & s) const;

private:
  void evaluate();

  std::size_t num_samples_;
  double max_gyro_;
  double max_accel_sd_;
  double accel_scale_;

  std::vector<Eigen::Vector3d> gyros_;
  std::vector<Eigen::Vector3d> accels_;   // already scaled to m/s^2

  bool initialized_ = false;
  int rejected_ = 0;
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
  Sophus::SO3d R_wi_;
};

}  // namespace glass_core

#endif  // GLASS_CORE_IMU_INIT_HPP
