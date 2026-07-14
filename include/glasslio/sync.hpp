#ifndef GLASSLIO_SYNC_HPP
#define GLASSLIO_SYNC_HPP

#include <deque>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "glasslio/types.hpp"   // MeasureGroup

namespace glasslio
{

/// [2] SYNC -- pair a LiDAR scan with the IMU samples that span it.
/// See doc/2-sync.md.
///
/// Trivial-looking, and it is where two of the nastier bugs in this pipeline can hide --
/// both of which produce a deskew that SILENTLY DOES NOTHING rather than an error.
///
/// THE REQUIREMENT. Deskew needs R(t) at the acquisition time of every point, and GyrInt
/// CLAMPS outside its knot range rather than extrapolating. So any point outside the IMU
/// coverage silently gets the endpoint rotation -- i.e. no correction, and no complaint.
/// A scan is therefore released only once the IMU brackets it on BOTH sides:
///
///     IMU:   *----*----*----*----*----*----*----*
///                ^                          ^
///            before t0                  past t1
///     scan:      |--------------------------|
///
/// The release decision has to be made BEFORE the cloud is parsed, and t1 is only knowable
/// by parsing it -- so we wait for coverage past a conservative proxy,
/// `header.stamp + scan_guard_sec`.
///
/// This class is deliberately ROS-LOGGING-FREE: it reports what happened by return value
/// and lets the node do the talking. That is what makes it testable without spinning up a
/// node (test_sync.cpp).
class MeasureSync
{
public:
  /// A timestamp older than the last one seen means one of three different things, and
  /// conflating them causes real damage.
  enum class TimeJump
  {
    /// Normal: time moved forward.
    None,
    /// Backwards by a hair: one out-of-order message (or two publishers racing). DROP just
    /// that message. Clearing the buffers here would throw away good data.
    OutOfOrder,
    /// Backwards by more than kRestartJumpSec: the SOURCE RESTARTED (a bag loop, a replay).
    /// The buffers are cleared -- but the caller must also reset the ESTIMATOR, because the
    /// pose and map now describe a world we are no longer in.
    Restart
  };

  /// A backwards jump larger than this means the source restarted, not a stray packet.
  static constexpr double kRestartJumpSec = 1.0;

  /// `scan_guard_sec` must EXCEED the scan period (0.1 s at 10 Hz), or the tail of every
  /// scan is under-corrected -- invisibly.
  explicit MeasureSync(double scan_guard_sec)
  : scan_guard_(scan_guard_sec) {}

  /// --- The time-jump watchdog. Call BEFORE pushing, and act on the result.
  ///
  /// Kept separate from push*() on purpose: the node must NOT buffer IMU before
  /// initialization has finished (those samples are consumed by the init window), but it
  /// must still watch the clock during it. One call does the watching, the other the
  /// buffering.
  TimeJump checkImu(double t) {return check(t, last_imu_time_);}
  TimeJump checkLidar(double t) {return check(t, last_lidar_time_);}

  void pushImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg) {imu_.push_back(msg);}
  void pushLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    lidar_.push_back(msg);
  }

  /// Pop the next MeasureGroup whose IMU coverage is complete.
  ///
  /// Returns false when nothing is ready yet -- which includes the case where a lidar frame
  /// had to be DISCARDED because no IMU precedes it (there is nothing to integrate from).
  /// `dropped_no_imu` distinguishes the two, so the caller can warn.
  bool next(MeasureGroup & meas, bool * dropped_no_imu = nullptr);

  /// The world we buffered is gone (a restart). Drops everything.
  void clear();

  std::size_t imu_count() const {return imu_.size();}
  std::size_t lidar_count() const {return lidar_.size();}

private:
  TimeJump check(double t, double & last_t);

  double scan_guard_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> lidar_;
  double last_imu_time_ = -1.0;
  double last_lidar_time_ = -1.0;
};

}  // namespace glasslio

#endif  // GLASSLIO_SYNC_HPP
