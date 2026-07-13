#ifndef GLASSLIO_GYR_INT_H
#define GLASSLIO_GYR_INT_H

#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include "sophus/so3.hpp"

namespace glasslio
{

/// Integrates gyroscope measurements into time-stamped orientation knots
/// R(t), with R(start) = identity, and lets you query the interpolated
/// rotation at any point time via SLERP. Rotations are in the IMU frame.
class GyrInt
{
public:
  GyrInt();

  /// Begin a new integration window anchored at `start_timestamp` (seconds).
  /// `last_imu` is a sample at or before the start, used to interpolate the
  /// angular velocity exactly at the anchor.
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr & last_imu);

  /// Fold one IMU sample into the integration (must be called in time order).
  void Integrate(const sensor_msgs::msg::Imu::ConstSharedPtr & imu);

  /// Interpolated rotation R(t) in [start, last]; clamps outside the range.
  Sophus::SO3d GetRotAt(double t) const;

  /// Total integrated rotation at the last knot (for logging).
  Sophus::SO3d GetRot() const;

  bool empty() const {return v_rot_.empty();}

  /// Gyro bias (rad/s), subtracted from every sample. From ImuInit.
  void set_bias(const Eigen::Vector3d & bias) {bias_ = bias;}

private:
  Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
  double start_timestamp_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;

  /// Parallel arrays: v_rot_[i] = R at absolute time t_knots_[i].
  std::vector<double> t_knots_;
  std::vector<Sophus::SO3d> v_rot_;
  Eigen::Vector3d last_gyr_;
};

}  // namespace glasslio

#endif  // GLASSLIO_GYR_INT_H
