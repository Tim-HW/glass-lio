#include "glasslio/deskew.hpp"

#include <limits>

#include <pcl_conversions/pcl_conversions.h>

#include "glasslio/ros_time.hpp"

namespace glasslio
{

using Sophus::SO3d;

Deskew::Deskew(rclcpp::Logger logger)
: R_il_(SO3d()), logger_(logger) {}

CloudXYZI::Ptr Deskew::process(const MeasureGroup & meas)
{
  if (meas.imu.empty() || meas.lidar == nullptr) {
    return nullptr;
  }

  LivoxCloud::Ptr cloud(new LivoxCloud());
  pcl::fromROSMsg(*meas.lidar, *cloud);
  if (cloud->empty()) {
    return nullptr;
  }

  // Per-point `timestamp` is absolute nanoseconds; work in seconds to match IMU.
  auto pt_sec = [](const LivoxPoint & p) {return p.timestamp * 1e-9;};

  // Scan time span from the per-point timestamps.
  double t0 = std::numeric_limits<double>::max();
  double t1 = std::numeric_limits<double>::lowest();
  for (const auto & pt : cloud->points) {
    t0 = std::min(t0, pt_sec(pt));
    t1 = std::max(t1, pt_sec(pt));
  }

  // Integrate gyro across the scan, anchored at scan start.
  gyr_int_.reset(t0, meas.imu.front());
  for (const auto & imu : meas.imu) {
    gyr_int_.integrate(imu);
  }
  if (gyr_int_.empty()) {
    return nullptr;
  }

  // Orientation of the lidar frame at time t, relative to scan start:
  //   R_L(t) = R_il^{-1} * R_I(t) * R_il
  auto lidar_rot_at = [&](double t) {
      return R_il_.inverse() * gyr_int_.rotationAt(t) * R_il_;
    };

  // Rotation of the lidar across this scan: the scan-end frame expressed in the
  // scan-start frame. Since scans are contiguous, this is also the rotation since
  // the previous pose -- the prediction registration will start from.
  const SO3d R_end = lidar_rot_at(t1);
  last_delta_rot_ = R_end;
  last_dt_ = t1 - t0;
  last_t1_ = t1;

  // Compensate every point into the scan-end frame:
  //   p_end = R_L(t1)^{-1} * R_L(t_i) * p_i
  const SO3d R_end_inv = R_end.inverse();

  CloudXYZI::Ptr out(new CloudXYZI());
  out->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    const SO3d R_i = lidar_rot_at(pt_sec(pt));
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    const Eigen::Vector3d pc = R_end_inv * (R_i * p);

    pcl::PointXYZI o;
    o.x = static_cast<float>(pc.x());
    o.y = static_cast<float>(pc.y());
    o.z = static_cast<float>(pc.z());
    o.intensity = pt.intensity;
    out->push_back(o);
  }
  out->height = 1;
  out->width = static_cast<std::uint32_t>(out->size());
  out->is_dense = false;

  const SO3d total = gyr_int_.totalRotation();
  RCLCPP_DEBUG(
    logger_, "deskew: %zu pts, scan %.3fs, gyro rot [x,y,z] deg [%.2f, %.2f, %.2f]",
    out->size(), t1 - t0,
    total.angleX() * 180.0 / M_PI,
    total.angleY() * 180.0 / M_PI,
    total.angleZ() * 180.0 / M_PI);

  return out;
}

}  // namespace glasslio
