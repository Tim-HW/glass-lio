#include "glasslio/gyr_int.h"

#include <algorithm>

#include "glasslio/ros_time.hpp"

namespace glasslio
{

using Sophus::SO3d;

GyrInt::GyrInt()
: start_timestamp_(-1), last_imu_(nullptr), last_gyr_(Eigen::Vector3d::Zero()) {}

void GyrInt::Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr & last_imu)
{
  start_timestamp_ = start_timestamp;
  last_imu_ = last_imu;
  t_knots_.clear();
  v_rot_.clear();
  last_gyr_.setZero();
}

static Eigen::Vector3d toGyr(const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{
  return {imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z};
}

void GyrInt::Integrate(const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{
  const double t = stamp_sec(imu);
  const Eigen::Vector3d gyr = toGyr(imu) - bias_;

  if (v_rot_.empty()) {
    // Anchor identity at the window start, interpolating gyro at that instant.
    Eigen::Vector3d gyr_start = gyr;
    if (last_imu_) {
      const double t_prev = stamp_sec(last_imu_);
      const double span = t - t_prev;
      if (span > 1e-9) {
        const double w = (t - start_timestamp_) / span;  // weight on previous
        gyr_start = w * (toGyr(last_imu_) - bias_) + (1.0 - w) * gyr;
      }
    }
    t_knots_.push_back(start_timestamp_);
    v_rot_.push_back(SO3d());
    last_gyr_ = gyr_start;
  }

  const double dt = t - t_knots_.back();
  if (dt <= 0) {
    return;  // out-of-order or duplicate sample
  }
  // Trapezoidal integration of angular velocity.
  const Eigen::Vector3d delta_angle = dt * 0.5 * (gyr + last_gyr_);
  const SO3d rot = v_rot_.back() * SO3d::exp(delta_angle);

  t_knots_.push_back(t);
  v_rot_.push_back(rot);
  last_gyr_ = gyr;
}

Sophus::SO3d GyrInt::GetRotAt(double t) const
{
  if (v_rot_.empty()) {
    return SO3d();
  }
  if (t <= t_knots_.front()) {
    return v_rot_.front();
  }
  if (t >= t_knots_.back()) {
    return v_rot_.back();
  }
  const auto it = std::upper_bound(t_knots_.begin(), t_knots_.end(), t);
  const size_t hi = static_cast<size_t>(it - t_knots_.begin());
  const size_t lo = hi - 1;
  const double ratio = (t - t_knots_[lo]) / (t_knots_[hi] - t_knots_[lo]);
  const Eigen::Quaterniond q =
    v_rot_[lo].unit_quaternion().slerp(ratio, v_rot_[hi].unit_quaternion());
  return SO3d(q);
}

Sophus::SO3d GyrInt::GetRot() const
{
  return v_rot_.empty() ? SO3d() : v_rot_.back();
}

}  // namespace glasslio
