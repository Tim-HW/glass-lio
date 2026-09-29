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

  // SWEEP vs SNAPSHOT. A Livox spreads its points across ~100 ms, so the per-point time
  // span (t1 - t0) is the real scan duration and t1 is the acquisition instant. A ToF is a
  // global-shutter SNAPSHOT: every point shares one instant, and this driver ships no
  // per-point time at all, so t0 == t1 == 0. For a snapshot the acquisition time is the
  // message HEADER stamp -- for every point, since a 0 must never be looked up on the gyro
  // timeline -- and consecutive headers give the true pose-to-pose interval.
  const bool snapshot = (t1 - t0) < 1e-6;
  const double t_end = snapshot ? stamp_sec(meas.lidar) : t1;
  const double t_start = snapshot ? t_end : t0;
  auto point_time = [&](const LivoxPoint & p) {return snapshot ? t_end : pt_sec(p);};

  // ANCHOR the gyro integration at the PREVIOUS scan's end when we can, not at this scan's
  // start: last_delta_rot_ must be the rotation since the last POSE, which is what the loose
  // path's guess needs. From this scan's start it missed a dropped scan's rotation entirely,
  // and for a snapshot (start == end) it was always identity. Only when the group's IMU
  // actually reaches back that far (sync keeps it from the previous scan's start;
  // mergeDroppedImu covers drops); otherwise -- first scan, a gap -- fall back to t_start.
  const double prev_end = last_t1_;
  const bool chain = prev_end > 0.0 && prev_end <= t_start && t_start - prev_end < 1.0 &&
    stamp_sec(meas.imu.front()) <= prev_end + kMaxAnchorGapSec;
  const double anchor = chain ? prev_end : t_start;

  // The anchor bracket is the LAST sample at or before the anchor, handed to reset() rather
  // than integrated: GyrInt interpolates omega at the anchor between the bracket and the
  // first sample after it. (Integrating the bracket itself made that span zero, so the
  // interpolation never ran.)
  std::size_t k = 0;
  while (k + 1 < meas.imu.size() && stamp_sec(meas.imu[k + 1]) <= anchor) {
    ++k;
  }
  gyr_int_.reset(anchor, meas.imu[k]);
  for (std::size_t i = k + 1; i < meas.imu.size(); ++i) {
    gyr_int_.integrate(meas.imu[i]);
  }
  if (gyr_int_.empty()) {
    return nullptr;
  }

  // Orientation of the lidar frame at time t, relative to the anchor:
  //   R_L(t) = R_il^{-1} * R_I(t) * R_il
  auto lidar_rot_at = [&](double t) {
      return R_il_.inverse() * gyr_int_.rotationAt(t) * R_il_;
    };

  // The scan-end frame expressed in the anchor frame: the rotation since the previous pose
  // -- the prediction registration will start from.
  const SO3d R_end = lidar_rot_at(t_end);
  last_delta_rot_ = R_end;
  last_dt_ = t1 - t0;   // 0 for a snapshot
  last_t1_ = t_end;

  // Compensate every point into the scan-end frame. Relative, so the anchor cancels out:
  //   p_end = R_L(t_end)^{-1} * R_L(t_i) * p_i
  const SO3d R_end_inv = R_end.inverse();

  CloudXYZI::Ptr out(new CloudXYZI());
  out->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    const SO3d R_i = lidar_rot_at(point_time(pt));
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
