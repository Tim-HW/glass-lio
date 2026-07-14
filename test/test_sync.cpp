// Self-check for [2] SYNC -- pairing a scan with the IMU that brackets it.
//
// This stage had a whole doc chapter and no test, because it was welded inside the ROS
// node. Its failures are the quiet kind: release a scan whose IMU does not cover it and
// GyrInt CLAMPS instead of extrapolating, so the tail of every scan is silently
// under-corrected. No error, no crash -- just a slightly warped cloud.
#include <cassert>
#include <cstdio>

#include "glasslio/sync.hpp"

using namespace glasslio;
using TimeJump = MeasureSync::TimeJump;

static sensor_msgs::msg::Imu::ConstSharedPtr imuAt(double t)
{
  auto m = std::make_shared<sensor_msgs::msg::Imu>();
  m->header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
  return m;
}

static sensor_msgs::msg::PointCloud2::ConstSharedPtr scanAt(double t)
{
  auto m = std::make_shared<sensor_msgs::msg::PointCloud2>();
  m->header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
  return m;
}

// =================================================================================
// 1. A scan is released ONLY when the IMU brackets it on BOTH sides.
// =================================================================================
static void testBracketsOnBothSides()
{
  MeasureSync sync(0.12);          // guard: a 0.10 s scan plus margin
  MeasureGroup meas;

  sync.pushLidar(scanAt(1.00));
  assert(!sync.next(meas) && "no IMU at all -> not ready");

  // IMU that STARTS after the scan: no sample to interpolate the anchor from. The frame
  // is unusable and must be discarded, not held forever.
  bool dropped = false;
  sync.pushImu(imuAt(1.05));
  assert(!sync.next(meas, &dropped));
  assert(dropped && "a scan with no PRECEDING imu must be dropped, not stall the pipeline");

  // Now do it properly: a sample before the scan, and coverage past scan + guard.
  MeasureSync s2(0.12);
  s2.pushImu(imuAt(0.99));         // the bracket BEFORE t0
  s2.pushLidar(scanAt(1.00));
  for (double t = 1.00; t <= 1.10; t += 0.01) {
    s2.pushImu(imuAt(t));
  }
  assert(!s2.next(meas) && "IMU does not yet cover scan_t + guard (1.12) -> hold");

  s2.pushImu(imuAt(1.13));         // now past the guard
  assert(s2.next(meas) && "bracketed on both sides -> release");
  assert(meas.lidar != nullptr);
  assert(meas.imu.size() >= 2);
  std::printf("  brackets on both sides      : held until covered, then released  OK\n");
}

// =================================================================================
// 2. CONSUMED IMU IS NOT EAGERLY DROPPED.
//
//    Scans are contiguous: the samples just inside the END of one scan's window are
//    exactly the ones that BRACKET THE START of the next. Delete them and every
//    subsequent scan loses its leading bracket -- and deskews against nothing.
// =================================================================================
static void testImuSurvivesForTheNextScan()
{
  MeasureSync sync(0.12);
  MeasureGroup meas;

  sync.pushImu(imuAt(0.99));
  for (double t = 1.00; t <= 1.30; t += 0.01) {
    sync.pushImu(imuAt(t));
  }
  sync.pushLidar(scanAt(1.00));
  sync.pushLidar(scanAt(1.10));    // the NEXT scan starts inside the first one's window

  assert(sync.next(meas));         // first scan out
  assert(sync.imu_count() > 0 && "the buffer must not have been emptied");

  // The second scan must STILL find a bracket before its own start (1.10).
  MeasureGroup meas2;
  assert(sync.next(meas2) && "the next scan lost its leading IMU bracket");
  assert(!meas2.imu.empty());
  std::printf("  imu kept for the next scan  : %zu samples survived              OK\n",
    sync.imu_count());
}

// =================================================================================
// 3. The three time-jump cases, which must NOT be conflated.
// =================================================================================
static void testTimeJumps()
{
  MeasureSync sync(0.12);

  assert(sync.checkImu(1.00) == TimeJump::None);
  assert(sync.checkImu(1.01) == TimeJump::None);

  // A hair backwards: one stray/out-of-order packet. Drop THAT MESSAGE only -- clearing
  // the buffer would turn a single bad packet into a pipeline stall.
  assert(sync.checkImu(1.005) == TimeJump::OutOfOrder);

  // A big jump backwards: the source RESTARTED (a bag loop). Buffers are gone.
  sync.pushImu(imuAt(1.01));
  sync.pushLidar(scanAt(1.01));
  assert(sync.imu_count() > 0 && sync.lidar_count() > 0);

  assert(sync.checkImu(0.001) == TimeJump::Restart);
  assert(sync.imu_count() == 0 && sync.lidar_count() == 0 &&
    "a restart must drop the buffers -- they describe a world we have left");

  // ...and the clock must have re-anchored, so the new run is not seen as one long
  // backwards jump.
  assert(sync.checkImu(0.002) == TimeJump::None);

  std::printf("  time jumps: none/stray/restart, all distinguished             OK\n");
}

// =================================================================================
// 4. scan_guard_sec must EXCEED the scan period, or the tail of every scan is silently
//    under-corrected. Pin that a too-small guard releases a scan EARLY.
// =================================================================================
static void testGuardTooSmallReleasesEarly()
{
  // A guard of 0.0 releases as soon as any IMU passes the scan's own stamp -- long before
  // the scan has actually finished sweeping. This is the bug the parameter exists to
  // prevent, and it is invisible in the logs.
  MeasureSync sloppy(0.0);
  MeasureGroup meas;
  sloppy.pushImu(imuAt(0.99));
  sloppy.pushLidar(scanAt(1.00));
  sloppy.pushImu(imuAt(1.00));
  assert(sloppy.next(meas) && "guard 0 releases immediately -- that is the hazard");

  // A correct guard holds the same scan back until the IMU really covers it.
  MeasureSync careful(0.12);
  careful.pushImu(imuAt(0.99));
  careful.pushLidar(scanAt(1.00));
  careful.pushImu(imuAt(1.00));
  assert(!careful.next(meas) && "a proper guard must NOT release on a single sample");

  std::printf("  scan_guard: too small releases early, correct one holds       OK\n");
}

int main()
{
  std::printf("test_sync: pairing a scan with the IMU that spans it\n");
  testBracketsOnBothSides();
  testImuSurvivesForTheNextScan();
  testTimeJumps();
  testGuardTooSmallReleasesEarly();
  std::printf("test_sync: all checks passed\n");
  return 0;
}
