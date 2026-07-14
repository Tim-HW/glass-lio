#include "glasslio/sync.hpp"

#include "glasslio/ros_time.hpp"

namespace glasslio
{

MeasureSync::TimeJump MeasureSync::check(double t, double & last_t)
{
  if (last_t < 0.0 || t >= last_t) {
    last_t = t;
    return TimeJump::None;
  }

  if (last_t - t > kRestartJumpSec) {
    // The source restarted. Our buffers describe a world we are no longer in.
    clear();
    last_t = t;
    return TimeJump::Restart;
  }

  // Out of order by a hair. Drop just this one message; do NOT nuke the buffer -- that
  // would turn a single stray packet into a pipeline stall.
  return TimeJump::OutOfOrder;
}

void MeasureSync::clear()
{
  imu_.clear();
  lidar_.clear();
  last_imu_time_ = -1.0;
  last_lidar_time_ = -1.0;
}

bool MeasureSync::next(MeasureGroup & meas, bool * dropped_no_imu)
{
  if (dropped_no_imu) {
    *dropped_no_imu = false;
  }
  if (lidar_.empty() || imu_.empty()) {
    return false;
  }

  const double scan_t = stamp_sec(lidar_.front());
  const double scan_end = scan_t + scan_guard_;

  // We need an IMU sample BEFORE the scan, to interpolate omega exactly at the anchor.
  // Without one there is nothing to integrate from, so the frame is unusable.
  if (stamp_sec(imu_.front()) > scan_t) {
    lidar_.pop_front();
    if (dropped_no_imu) {
      *dropped_no_imu = true;
    }
    return false;
  }

  // Wait until the IMU covers the END of the scan. Release it early and the tail of the
  // scan -- where the correction relative to the scan-end reference is LARGEST -- clamps
  // to the last knot and is quietly under-corrected.
  if (stamp_sec(imu_.back()) < scan_end) {
    return false;
  }

  meas.lidar = lidar_.front();
  lidar_.pop_front();

  // Copy the IMU covering the scan, plus one bracket sample past the end.
  meas.imu.clear();
  for (const auto & imu : imu_) {
    meas.imu.push_back(imu);
    if (stamp_sec(imu) >= scan_end) {
      break;
    }
  }

  // CONSUMED IMU IS NOT EAGERLY DROPPED. Scans are contiguous: the samples just inside the
  // end of this scan's window are exactly the ones that will BRACKET THE START of the next
  // one. Delete them and every subsequent scan loses its leading bracket.
  //
  // So prune only what is strictly older than THIS scan's start, and keep size() > 1 so we
  // can never empty the buffer of the sample that brackets the next scan.
  while (imu_.size() > 1 && stamp_sec(imu_[1]) <= scan_t) {
    imu_.pop_front();
  }
  return true;
}

}  // namespace glasslio
