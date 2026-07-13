#include "glasslio/imu_init.hpp"

#include <cmath>

namespace glasslio
{

ImuInit::ImuInit(int num_samples, double max_gyro, double max_accel_sd, double accel_scale)
: num_samples_(static_cast<std::size_t>(num_samples)),
  max_gyro_(max_gyro),
  max_accel_sd_(max_accel_sd),
  accel_scale_(accel_scale)
{
}

Eigen::Vector3d ImuInit::accel_si(const sensor_msgs::msg::Imu & msg) const
{
  return Eigen::Vector3d(
    msg.linear_acceleration.x,
    msg.linear_acceleration.y,
    msg.linear_acceleration.z) * accel_scale_;
}

bool ImuInit::add(const sensor_msgs::msg::Imu & msg)
{
  if (initialized_) {
    return true;
  }

  gyros_.emplace_back(
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  accels_.push_back(accel_si(msg));

  if (gyros_.size() < num_samples_) {
    return false;
  }

  evaluate();
  return initialized_;
}

void ImuInit::evaluate()
{
  const auto n = static_cast<double>(gyros_.size());

  Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < gyros_.size(); ++i) {
    gyro_mean += gyros_[i];
    accel_mean += accels_[i];
  }
  gyro_mean /= n;
  accel_mean /= n;

  // --- Motion check 1: no sample may exceed the rotation-rate limit.
  double gyro_max = 0.0;
  for (const auto & g : gyros_) {
    gyro_max = std::max(gyro_max, g.norm());
  }

  // --- Motion check 2: per-axis accelerometer std-dev must be small. Catches
  // linear motion, which a gyro-only check would miss entirely.
  Eigen::Vector3d var = Eigen::Vector3d::Zero();
  for (const auto & a : accels_) {
    var += (a - accel_mean).cwiseAbs2();
  }
  const Eigen::Vector3d sd = (var / n).cwiseSqrt();

  if (gyro_max > max_gyro_ || sd.maxCoeff() > max_accel_sd_) {
    // Moving: throw the window away and wait for a quiet one. Do NOT initialize
    // from it -- a bad init is worse than no init, because nothing reports it.
    ++rejected_;
    gyros_.clear();
    accels_.clear();
    return;
  }

  gyro_bias_ = gyro_mean;
  gravity_ = accel_mean;

  // Gravity-align: rotate the measured "up" onto world +Z. Yaw is unobservable
  // from gravity, so FromTwoVectors' minimal rotation (zero yaw) is exactly right.
  const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
    gravity_.normalized(), Eigen::Vector3d::UnitZ());
  R_wi_ = Sophus::SO3d(q.normalized());

  initialized_ = true;
  gyros_.clear();
  accels_.clear();
}

}  // namespace glasslio
