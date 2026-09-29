// Smoke test for LioEstimator -- the whole of [2] sync -> [6] local map, driven from a test.
//
// The estimator was split out of the node precisely so this could exist, and until it did
// the pipeline's glue had no safety net: a loose velocity divided by the scan DURATION
// (zero on a snapshot sensor, so v stayed 0 forever) passed every unit test there was.
//
// Scene: a closed synthetic room. Sensor: a ToF-style SNAPSHOT (every point shares one
// timestamp) at 10 Hz, gliding along +x at a constant 1 m/s with no rotation. IMU: 200 Hz,
// stationary rotation, reporting only gravity's reaction, in g (Livox convention).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "glasslio/lio_estimator.hpp"
#include "glasslio/ros_time.hpp"
#include "glasslio/sync.hpp"

using namespace glasslio;

static constexpr double kV = 1.0;          // m/s along +x
static constexpr double kX0 = -2.0;        // sensor x at the first scan (world)
static constexpr double kT0 = 1.0;         // first scan time (s)
static constexpr double kScanDt = 0.1;

/// Floor, ceiling and four walls, 20 x 20 x 3 m. Closed, so no direction is degenerate.
static std::vector<Eigen::Vector3d> makeRoom()
{
  std::vector<Eigen::Vector3d> pts;
  const double L = 10.0, H = 3.0, s = 0.2;
  for (double a = -L; a <= L; a += s) {
    for (double b = -L; b <= L; b += s) {
      pts.emplace_back(a, b, 0.0);
      pts.emplace_back(a, b, H);
    }
    for (double z = 0.0; z <= H; z += s) {
      pts.emplace_back(a, -L, z);
      pts.emplace_back(a, L, z);
      pts.emplace_back(-L, a, z);
      pts.emplace_back(L, a, z);
    }
  }
  return pts;
}

static builtin_interfaces::msg::Time stamp(double t)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)));
}

static double sensorX(double t) {return kX0 + kV * (t - kT0);}

/// `max_range` > 0 crops the scan, so a moving sensor keeps seeing NEW geometry.
static sensor_msgs::msg::PointCloud2::ConstSharedPtr scanAt(
  const std::vector<Eigen::Vector3d> & room, double t, double max_range = 0.0)
{
  LivoxCloud c;
  const Eigen::Vector3d origin(sensorX(t), 0.0, 1.5);
  for (const auto & p : room) {
    const Eigen::Vector3d q = p - origin;   // no rotation: world -> sensor is a shift
    if (max_range > 0.0 && q.norm() > max_range) {
      continue;
    }
    LivoxPoint lp;
    lp.x = static_cast<float>(q.x());
    lp.y = static_cast<float>(q.y());
    lp.z = static_cast<float>(q.z());
    lp.intensity = 0.0f;
    lp.tag = 0;
    lp.line = 0;
    lp.timestamp = std::llround(t * 1e9);  // SNAPSHOT: one instant for every point
    c.push_back(lp);
  }
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(c, *msg);
  msg->header.stamp = stamp(t);
  return msg;
}

static sensor_msgs::msg::Imu::ConstSharedPtr imuAt(double t, double yaw_rate = 0.0)
{
  auto m = std::make_shared<sensor_msgs::msg::Imu>();
  m->header.stamp = stamp(t);
  m->linear_acceleration.z = 1.0;          // at rest in g: gravity's reaction only
  m->angular_velocity.z = yaw_rate;
  return m;
}

/// Deskew's rotation guess must span the gap since the LAST POSE -- across a dropped scan,
/// and on a snapshot sensor, where "scan start to scan end" is zero and the guess used to be
/// identity forever.
static void testRotationGuessSpansTheGap()
{
  const double w = 0.5;                    // rad/s yaw
  const auto room = makeRoom();
  MeasureSync sync(0.12);
  std::vector<MeasureGroup> groups;
  int scan_idx = 0;
  for (double t = kT0 - 0.2; t < kT0 + 0.6; t += 0.005) {
    sync.pushImu(imuAt(t, w));
    if (scan_idx < 3 && t >= kT0 + kScanDt * scan_idx) {
      sync.pushLidar(scanAt(room, kT0 + kScanDt * scan_idx));
      ++scan_idx;
    }
    MeasureGroup meas;
    while (sync.next(meas)) {
      groups.push_back(meas);
      meas = MeasureGroup();
    }
  }
  assert(groups.size() == 3);

  Deskew deskew(rclcpp::get_logger("test_estimator"));
  assert(deskew.process(groups[0]));
  mergeDroppedImu(groups[1], groups[2]);   // scan 1 dropped, as the node does
  assert(deskew.process(groups[2]));

  const double yaw = deskew.last_delta_rot().log().z();
  std::printf("  rotation guess across a dropped snapshot scan: %.4f rad (true %.4f)\n",
    yaw, w * 2 * kScanDt);
  std::fflush(stdout);
  assert(std::abs(yaw - w * 2 * kScanDt) < 1e-3 && "guess must cover the whole gap");
}

/// Tight coupling assumes R_il = I. A rotated extrinsic must be refused at construction --
/// not fused into a confident wrong pose. Loose uses the extrinsic properly and must accept it.
static void testTightRefusesRotatedExtrinsic()
{
  EstimatorParams p;
  p.extrinsic_q_il = Eigen::Quaterniond(Eigen::AngleAxisd(10.0 * M_PI / 180.0,
    Eigen::Vector3d::UnitZ()));
  const auto logger = rclcpp::get_logger("test_estimator");

  p.use_tight = true;
  bool threw = false;
  try {
    LioEstimator est(p, logger);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "tight + rotated extrinsic must be refused");

  p.use_tight = false;
  LioEstimator loose(p, logger);   // must not throw

  p.use_tight = true;
  p.extrinsic_q_il = Eigen::Quaterniond::Identity();
  LioEstimator tight(p, logger);   // identity: fine
  std::printf("  tight + 10 deg extrinsic refused; loose and identity accepted\n");
}

/// Simulate `n_scans` of the synthetic room through a real MeasureSync, handing every group
/// it releases to `on_group`, in time order.
static void feedScans(
  int n_scans, const std::function<void(MeasureGroup &)> & on_group, double max_range = 0.0)
{
  const auto room = makeRoom();
  MeasureSync sync(0.12);
  int scan_idx = 0;
  const double t_stop = kT0 + kScanDt * n_scans + 0.3;   // IMU past the last scan's guard
  for (double t = kT0 - 0.2; t < t_stop; t += 0.005) {
    sync.pushImu(imuAt(t));
    if (scan_idx < n_scans && t >= kT0 + kScanDt * scan_idx) {
      sync.pushLidar(scanAt(room, kT0 + kScanDt * scan_idx, max_range));
      ++scan_idx;
    }
    MeasureGroup meas;
    while (sync.next(meas)) {
      on_group(meas);
      meas = MeasureGroup();
    }
  }
}

/// While the tight path COASTS, the carried covariance must grow: a dead-reckoned state is
/// less certain than the solved one it came from. Every scan is forced to coast here.
static void testCoastGrowsCovariance()
{
  EstimatorParams p;
  p.use_tight = true;
  p.tight.reg = p.reg;
  p.tight_warmup_scans = 3;
  p.max_rmse = 1e-9;                       // reject every solve: pure dead reckoning
  LioEstimator est(p, rclcpp::get_logger("test_estimator"));
  est.initialize(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, -kGravity));

  int n = 0;
  double prev = -1.0, first = 0.0, last = 0.0;
  feedScans(12, [&](MeasureGroup & meas) {
      est.processScan(meas);
      if (++n <= p.tight_warmup_scans + 1) {
        return;   // warm-up is loose, and the first tight scan has no IMU window yet
      }
      const double tr = est.navCovariance().block<3, 3>(kIdxPos, kIdxPos).trace();
      assert(tr > prev && "position covariance must grow on every coasting scan");
      if (prev < 0.0) {first = tr;}
      prev = last = tr;
    });
  std::printf("  coasting tight: position variance trace %.2e -> %.2e over %d scans\n",
    first, last, n - p.tight_warmup_scans - 1);
  assert(last > 2.0 * first);
}

/// BOOTSTRAP: until one registration has actually succeeded against a non-empty map, the map
/// must keep growing even when registration fails -- otherwise sparse map -> ICP refused ->
/// nothing inserted -> map stays sparse, forever. The first scan used to END the bootstrap
/// (it reports "trusted" only because an empty map has nothing to fail against), so the map
/// froze at one scan. Here every registration is refused, so the pose stays at the origin;
/// the map must still grow as the approaching +x wall enters the sensor's range.
static void testBootstrapKeepsFeedingTheMap()
{
  EstimatorParams p;
  p.reg.min_correspondences = 1000000000;   // every ICP refused: under-constrained
  LioEstimator est(p, rclcpp::get_logger("test_estimator"));
  est.initialize(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, -kGravity));

  int n = 0, trusted = 0;
  std::size_t after_first = 0;
  feedScans(20, [&](MeasureGroup & meas) {
      const ScanResult r = est.processScan(meas);
      trusted += (n > 0 && r.pose_trusted) ? 1 : 0;
      if (n++ == 0) {
        after_first = est.map().num_voxels();
      }
    }, 11.0);   // the +x wall starts 12 m away and comes within range as we approach
  const std::size_t final_voxels = est.map().num_voxels();
  std::printf("  bootstrap, every ICP refused: map %zu -> %zu voxels over %d scans\n",
    after_first, final_voxels, n);
  std::fflush(stdout);
  assert(trusted == 0 && "the setup must actually refuse every registration");
  assert(final_voxels > after_first && "the map froze after the first scan");
}

/// Run the pipeline for `n_scans`. With `drop_every_other`, every second scan is dropped the
/// way the node drops one when the worker falls behind: its IMU handed to the next group.
static void run(bool tight, bool drop_every_other)
{
  EstimatorParams p;
  p.use_tight = tight;
  p.tight.imu_prior_weight = 1.0;
  p.tight.reg = p.reg;
  p.tight_warmup_scans = 5;
  LioEstimator est(p, rclcpp::get_logger("test_estimator"));
  est.initialize(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, -kGravity));

  int processed = 0, rejected = 0;
  double last_t = 0.0;
  MeasureGroup pending_drop;
  bool have_drop = false;

  feedScans(30, [&](MeasureGroup & meas) {
      const double ts = stamp_sec(meas.lidar);
      // Never drop the first scans: the map and the tight warm-up need them.
      const bool drop = drop_every_other && ts > kT0 + 1.0 && !have_drop &&
        std::lround((ts - kT0) / kScanDt) % 2 == 1;
      if (drop) {
        pending_drop = meas;
        have_drop = true;
        return;
      }
      if (have_drop) {
        mergeDroppedImu(pending_drop, meas);
        have_drop = false;
      }
      const ScanResult r = est.processScan(meas);
      assert(r.ok);
      rejected += r.pose_trusted ? 0 : 1;
      ++processed;
      last_t = ts;
    });

  // The estimator's world origin is the first scan's pose, so compare displacements.
  const double x_est = est.pose().translation().x();
  const double x_true = sensorX(last_t) - kX0;
  const double v_est = est.velocity().x();
  std::printf(
    "  %-5s drop=%d: %d scans, %d rejected | x %.3f (true %.3f) | v %.3f (true %.3f)\n",
    tight ? "tight" : "loose", drop_every_other, processed, rejected,
    x_est, x_true, v_est, kV);
  std::fflush(stdout);   // an assert below aborts, which would discard buffered output

  assert(processed >= (drop_every_other ? 15 : 29));
  assert(rejected == 0);
  assert(std::abs(x_est - x_true) < 0.1 && "pose must follow the motion");
  // THE regression: on a snapshot sensor the loose velocity was divided by a zero scan
  // duration, skipped, and left at 0. And without mergeDroppedImu, a dropped scan left the
  // tight IMU factor integrating half the gap: v came out ~2.0.
  assert(std::abs(v_est - kV) < 0.15 && "velocity: not stuck at 0 (snapshot dt), not doubled (dropped scan)");
  assert(std::abs(est.pose().translation().y()) < 0.1);
  assert(std::abs(est.pose().translation().z()) < 0.1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testRotationGuessSpansTheGap();
  testTightRefusesRotatedExtrinsic();
  testCoastGrowsCovariance();
  testBootstrapKeepsFeedingTheMap();
  run(false, false);
  run(false, true);
  run(true, false);
  run(true, true);
  rclcpp::shutdown();
  std::printf("test_estimator: all checks passed\n");
  return 0;
}
