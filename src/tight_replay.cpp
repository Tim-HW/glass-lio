// Deterministic offline driver for the estimator -- Phase 0 of the preintegration roadmap
// (doc/7-tight-coupling.md). It reads a bag and feeds scans + IMU to LioEstimator
// SYNCHRONOUSLY, in bag order, on one thread -- so a run is REPRODUCIBLE. The live node is
// not: its worker thread drops scans under load and interleaves nondeterministically, so
// the trajectory differs run to run and you cannot tell a fix from noise. This can.
//
// It ends with the SCALE GATE -- the check that was skipped when a 20 m drift got mistaken
// for a 5 m room: for a bounded scene the pose trajectory must stay within a few times the
// per-scan extent. Tight coupling today blows this by ~4x; the gate makes that a hard,
// scriptable failure (nonzero exit) instead of a pretty picture.
//
//   tight_replay <bag_dir> [imu_topic] [lidar_topic] [imu_prior_weight]
//
// imu_prior_weight > 0 selects the tight path (default 1.0); 0 runs loose for comparison.

#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "glass_core/imu_init.hpp"
#include "glasslio/lio_estimator.hpp"
#include "glasslio/sync.hpp"
#include "glasslio/types.hpp"

using glasslio::EstimatorParams;
using glasslio::LioEstimator;
using glasslio::MeasureGroup;
using glasslio::MeasureSync;
using glasslio::ScanResult;
using glass_core::ImuInit;
using glass_core::ImuSample;
using glass_core::kGravity;

static ImuSample toSample(const sensor_msgs::msg::Imu & m)
{
  return ImuSample{
    {m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z},
    {m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z}};
}

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr,
      "usage: tight_replay <bag_dir> [imu_topic] [lidar_topic] [imu_prior_weight]\n");
    return 2;
  }
  const std::string bag = argv[1];
  const std::string imu_topic = argc > 2 ? argv[2] : "/asdt1_driver/imu";
  const std::string lidar_topic = argc > 3 ? argv[3] : "/asdt1_driver/point_cloud";
  const double imu_prior_weight = argc > 4 ? std::stod(argv[4]) : 1.0;

  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("tight_replay");

  // Params: this ToF sensor (accel already in m/s^2) with the sparse-cloud tuning from
  // config/asdt1.yaml, and the tight path selected by imu_prior_weight.
  EstimatorParams p;
  p.voxel_leaf_size = 0.10;
  p.map_voxel_size = 0.30;
  p.map_min_points_for_plane = 4;
  p.reg.min_correspondences = 20;
  p.reg.max_correspondence_distance = 0.5;
  p.accel_scale = 1.0;                       // m/s^2, not g
  p.use_tight = imu_prior_weight > 0.0;
  p.tight.imu_prior_weight = imu_prior_weight;

  LioEstimator est(p, logger);
  ImuInit init(200, 0.1, 0.5, 1.0);          // accel_scale 1.0 (m/s^2)
  MeasureSync sync(0.12);

  bool inited = false;
  int scan_idx = 0;
  Eigen::Vector3d tmin = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d tmax = -tmin;
  double scan_extent_sum = 0.0;              // sum of per-scan bbox diagonals
  int scan_extent_n = 0;

  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;

  auto drain = [&]() {
      MeasureGroup meas;
      while (sync.next(meas)) {
        const ScanResult r = est.processScan(meas);
        const Eigen::Vector3d t = est.pose().translation();
        tmin = tmin.cwiseMin(t);
        tmax = tmax.cwiseMax(t);
        std::printf(
        "scan %4d | pose [%+7.2f %+7.2f %+7.2f] | rmse %.3f (%d corr)%s | map %zu vox\n",
        scan_idx++, t.x(), t.y(), t.z(), r.rmse, r.correspondences,
        r.pose_trusted ? "" : " COAST", est.map().num_voxels());
      }
    };

  while (reader.has_next()) {
    auto bm = reader.read_next();
    rclcpp::SerializedMessage ser(*bm->serialized_data);

    if (bm->topic_name == imu_topic) {
      auto m = std::make_shared<sensor_msgs::msg::Imu>();
      imu_ser.deserialize_message(&ser, m.get());
      if (!inited) {
        if (init.add(toSample(*m))) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.linear() = init.initial_rotation().matrix();
          est.initialize(pose, init.gyro_bias(), Eigen::Vector3d(0, 0, -kGravity));
          inited = true;
          const Eigen::Vector3d g = init.gravity();
          std::printf(
            "IMU init: |g|=%.3f  tilt=%.1f deg  bias=[%+.4f %+.4f %+.4f]  (%d rejected)\n",
            g.norm(), std::acos(std::clamp(g.normalized().z(), -1.0, 1.0)) * 180.0 / M_PI,
            init.gyro_bias().x(), init.gyro_bias().y(), init.gyro_bias().z(),
            init.rejected_windows());
        }
        continue;   // do not buffer IMU into sync until init completes
      }
      sync.pushImu(m);
      drain();
    } else if (bm->topic_name == lidar_topic) {
      if (!inited) {
        continue;
      }
      auto m = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pc_ser.deserialize_message(&ser, m.get());
      // Measure the raw scan extent (= how big the observed scene really is).
      pcl::PointCloud<pcl::PointXYZ> c;
      pcl::fromROSMsg(*m, c);
      if (!c.empty()) {
        Eigen::Vector4f lo, hi;
        pcl::getMinMax3D(c, lo, hi);
        scan_extent_sum += (hi.head<3>() - lo.head<3>()).norm();
        ++scan_extent_n;
      }
      sync.pushLidar(m);
      drain();
    }
  }

  if (scan_idx == 0 || scan_extent_n == 0) {
    std::fprintf(stderr, "no scans processed (init never fired? wrong topics?)\n");
    rclcpp::shutdown();
    return 2;
  }

  // --- THE SCALE GATE, two-sided. In a bounded room the trajectory must be BOTH:
  //   * not larger than the scene the sensor sees -- you move WITHIN what you can see, so
  //     ratio < ~1; wandering past it (loose: 2.9x) is drift, and
  //   * not ~zero -- a 30 deg FOV cannot map a room from one spot, so the pose MUST travel
  //     through it; a frozen pose (tight: 0.0x) has not tracked at all.
  // Heuristic, not ground truth (the bag has none) -- a proxy for "stayed in the room and
  // actually moved through it". Both current paths fail it, from opposite ends.
  const double traj = (tmax - tmin).norm();
  const double scan = scan_extent_sum / scan_extent_n;
  const double ratio = traj / scan;
  const double kMinRatio = 0.15;
  const double kMaxRatio = 1.50;
  const char * verdict =
    ratio > kMaxRatio ? "FAIL -- drift: pose wandered past the scene (runaway)" :
    ratio < kMinRatio ? "FAIL -- frozen: pose barely moved (not tracking)" :
    "PASS -- bounded and moving";
  const bool pass = ratio >= kMinRatio && ratio <= kMaxRatio;

  std::printf("\n--- SCALE GATE ---\n");
  std::printf("  scans processed      : %d\n", scan_idx);
  std::printf("  per-scan extent (med): %.1f m   (the scene the sensor sees)\n", scan);
  std::printf("  trajectory extent    : %.1f m\n", traj);
  std::printf("  ratio                : %.2fx   (pass band %.2f--%.2fx)\n", ratio, kMinRatio,
    kMaxRatio);
  std::printf("  %s\n", verdict);

  rclcpp::shutdown();
  return pass ? 0 : 1;
}
