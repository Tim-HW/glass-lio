#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "lidar_odom/data_process.h"
#include "lidar_odom/imu_init.hpp"
#include "lidar_odom/local_map.hpp"
#include "lidar_odom/ros_time.hpp"

namespace lidar_odom
{

/// LiDAR-inertial odometry node.
///
/// Pipeline per scan:
///   sync(lidar, imu) -> deskew -> downsample -> [register -> odom]
///
/// Deskew and downsample are stages inside this node, not separate nodes: the
/// intermediate clouds are only ever consumed here, so publishing them would
/// cost a serialize/deserialize round trip for nothing. They are exposed on
/// debug topics purely for RViz inspection.
class LioNode : public rclcpp::Node
{
public:
  LioNode()
  : Node("lio_node")
  {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    output_frame_ = declare_parameter<std::string>("output_frame", "livox_frame");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    // Extra IMU we require past a scan's header stamp before we deskew it, so
    // the gyro integration fully covers the scan (10 Hz -> ~0.1 s scans).
    scan_guard_ = declare_parameter<double>("scan_guard_sec", 0.12);
    // Voxel edge length (m). Larger = fewer points = faster registration, but
    // coarser geometry. ~0.5 m is a sane start for indoor/outdoor Livox.
    voxel_leaf_ = declare_parameter<double>("voxel_leaf_size", 0.5);

    // Extrinsic lidar -> IMU. Nested YAML maps to dot-separated param names.
    const auto q_xyzw = declare_parameter<std::vector<double>>(
      "extrinsic.lidar_to_imu.quat_xyzw", {0.0, 0.0, 0.0, 1.0});
    const auto t_xyz = declare_parameter<std::vector<double>>(
      "extrinsic.lidar_to_imu.xyz", {0.0, 0.0, 0.0});

    const double map_voxel = declare_parameter<double>("map.voxel_size", 0.5);
    const int map_max_pts = declare_parameter<int>("map.max_points_per_voxel", 20);
    const double map_range = declare_parameter<double>("map.max_range", 100.0);

    // IMU initialization. Livox publishes accel in *g*, not m/s^2 (spec violation),
    // so scale it. Get this wrong and every acceleration is 9.81x off.
    const bool accel_in_g = declare_parameter<bool>("imu.accel_in_g", true);
    const int init_samples = declare_parameter<int>("imu.init.num_samples", 200);
    const double init_max_gyro = declare_parameter<double>("imu.init.max_gyro", 0.1);
    const double init_max_acc_sd = declare_parameter<double>("imu.init.max_accel_sd", 0.5);

    q_il_ = parseQuat(q_xyzw);
    imu_proc_ = std::make_shared<ImuProcess>(get_logger());
    imu_proc_->set_extrinsic(q_il_, parseVec3(t_xyz));

    imu_init_ = std::make_unique<ImuInit>(
      init_samples, init_max_gyro, init_max_acc_sd, accel_in_g ? kGravity : 1.0);

    map_ = std::make_unique<LocalMap>(map_voxel, map_max_pts, map_range);

    // Registration (GICP). max_corr_dist is the important knob: too small and a
    // fast motion falls outside the correspondence radius and never converges;
    // too large and it happily matches a wall to the wrong wall.
    const double max_corr = declare_parameter<double>(
      "registration.max_correspondence_distance", 1.0);
    const int max_iter = declare_parameter<int>("registration.max_iterations", 30);
    const double eps = declare_parameter<double>("registration.transformation_epsilon", 1e-3);
    max_fitness_ = declare_parameter<double>("registration.max_fitness_score", 2.0);
    // Constant-velocity translation prior. DEFAULT OFF: it caused a runaway --
    // GICP "converges" near a too-far guess, that bad scan is inserted into the
    // map, the map drifts with it, and velocity grows without bound. Only safe
    // once max_correspondence_distance is tight enough to reject a bad guess.
    use_const_vel_ = declare_parameter<bool>("registration.use_constant_velocity", false);

    max_corr_ = max_corr;
    gicp_.setMaxCorrespondenceDistance(max_corr);
    gicp_.setMaximumIterations(max_iter);
    gicp_.setTransformationEpsilon(eps);

    voxel_.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);

    auto qos = rclcpp::SensorDataQoS();
    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, qos,
      std::bind(&LioNode::lidarCallback, this, std::placeholders::_1));
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      std::bind(&LioNode::imuCallback, this, std::placeholders::_1));

    pub_deskewed_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed", 10);
    pub_downsampled_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/downsampled", 10);
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/local_map", 1);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(), "lio_node up. lidar='%s' imu='%s' voxel=%.2fm",
      lidar_topic_.c_str(), imu_topic_.c_str(), voxel_leaf_);
    RCLCPP_INFO(
      get_logger(), "extrinsic lidar->imu: q_xyzw=[%.4f %.4f %.4f %.4f] t=[%.4f %.4f %.4f]",
      q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3], t_xyz[0], t_xyz[1], t_xyz[2]);
    RCLCPP_INFO(
      get_logger(), "local map: voxel=%.2fm max_pts/voxel=%d range=%.1fm",
      map_voxel, map_max_pts, map_range);
  }

private:
  /// A malformed extrinsic silently corrupts every scan, so fail loudly here.
  Eigen::Quaterniond parseQuat(const std::vector<double> & q_xyzw)
  {
    if (q_xyzw.size() != 4) {
      throw std::runtime_error("extrinsic.lidar_to_imu.quat_xyzw must have 4 elements (x,y,z,w)");
    }
    const Eigen::Quaterniond q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);  // (w,x,y,z) ctor
    if (q.norm() < 1e-6) {
      throw std::runtime_error("extrinsic.lidar_to_imu.quat_xyzw is degenerate (zero norm)");
    }
    return q.normalized();
  }

  Eigen::Vector3d parseVec3(const std::vector<double> & v)
  {
    if (v.size() != 3) {
      throw std::runtime_error("extrinsic.lidar_to_imu.xyz must have 3 elements");
    }
    return {v[0], v[1], v[2]};
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buf_mutex_);
    const double t = stamp_sec(msg);
    if (t < last_imu_time_) {
      RCLCPP_WARN(get_logger(), "IMU time went backwards, clearing buffer");
      imu_buffer_.clear();
    }
    last_imu_time_ = t;

    if (!imu_init_->initialized()) {
      if (imu_init_->add(*msg)) {
        onInitialized();
      }
      return;   // no scans are processed until the IMU is initialized
    }

    imu_buffer_.push_back(msg);
    tryProcess();
  }

  /// Apply the estimated bias and gravity alignment, then let scans through.
  void onInitialized()
  {
    imu_proc_->set_gyro_bias(imu_init_->gyro_bias());

    // The world frame is gravity-aligned (Z up). The initial LIDAR pose is
    // therefore R_wl = R_wi * R_il -- the extrinsic still has to be applied,
    // because gravity was measured in the IMU frame, not the lidar frame.
    pose_ = Eigen::Isometry3d::Identity();
    pose_.linear() =
      (imu_init_->initial_rotation().unit_quaternion() * q_il_).toRotationMatrix();

    const auto & b = imu_init_->gyro_bias();
    const auto & g = imu_init_->gravity();
    const double tilt =
      std::acos(std::clamp(g.normalized().z(), -1.0, 1.0)) * 180.0 / M_PI;

    RCLCPP_INFO(get_logger(), "IMU initialized (%d window(s) rejected for motion)",
      imu_init_->rejected_windows());
    RCLCPP_INFO(get_logger(), "  gyro bias : [%+.5f %+.5f %+.5f] rad/s", b.x(), b.y(), b.z());
    RCLCPP_INFO(get_logger(), "  gravity   : [%+.3f %+.3f %+.3f] m/s^2  |g|=%.3f",
      g.x(), g.y(), g.z(), g.norm());
    RCLCPP_INFO(get_logger(), "  mount tilt: %.2f deg from vertical", tilt);

    if (std::abs(g.norm() - kGravity) > 1.0) {
      RCLCPP_ERROR(
        get_logger(),
        "|gravity| = %.3f m/s^2, expected ~%.2f. Check imu.accel_in_g -- wrong units "
        "will make every acceleration 9.81x off.", g.norm(), kGravity);
    }
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buf_mutex_);

    if (!imu_init_->initialized()) {
      // Drop scans rather than buffer them: they predate initialization, and the
      // IMU samples that would deskew them were consumed by the init window.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for IMU initialization (hold still)...");
      return;
    }

    const double t = stamp_sec(msg);
    if (t < last_lidar_time_) {
      RCLCPP_WARN(get_logger(), "lidar time went backwards, clearing buffer");
      lidar_buffer_.clear();
    }
    last_lidar_time_ = t;
    lidar_buffer_.push_back(msg);
    tryProcess();
  }

  /// Run the pipeline on every scan the IMU already fully covers.
  void tryProcess()
  {
    MeasureGroup meas;
    while (syncMeasure(meas)) {
      // --- Stage 1: deskew (motion compensation, se1e doc/deskew.md) ---
      auto deskewed = imu_proc_->Process(meas);
      if (!deskewed || deskewed->empty()) {
        continue;
      }

      // --- Stage 2: downsample ---
      CloudXYZI::Ptr downsampled(new CloudXYZI());
      voxel_.setInputCloud(deskewed);
      voxel_.filter(*downsampled);
      
      // --- Stage 3: register against the local map -> pose_ ---
      const bool ok = registerScan(downsampled);

      // --- Stage 4: fold the aligned scan into the map, in the world frame ---
      CloudXYZI::Ptr scan_world(new CloudXYZI());
      pcl::transformPointCloud(*downsampled, *scan_world, pose_.matrix());
      map_->insert(*scan_world);
      map_->prune(pose_.translation());

      const Eigen::Vector3d & t = pose_.translation();
      RCLCPP_INFO(
        get_logger(),
        "scan %zu->%zu pts | pose [%+.2f %+.2f %+.2f] | fit %.3f%s | map %zu vox %zu pts",
        deskewed->size(), downsampled->size(),
        t.x(), t.y(), t.z(), last_fitness_, ok ? "" : " (DIVERGED)",
        map_->num_voxels(), map_->num_points());

      publishOdom(meas.lidar->header);
      publish(pub_deskewed_, deskewed, meas.lidar->header);
      publish(pub_downsampled_, downsampled, meas.lidar->header);
      publishMap(meas.lidar->header);
    }
  }

  /// Align `scan` (sensor frame) to the local map and update pose_.
  /// Returns false if GICP diverged, in which case we keep the IMU prediction.
  bool registerScan(const CloudXYZI::Ptr & scan)
  {
    // Bootstrap: nothing to align against. The first scan DEFINES the origin --
    // pose_ keeps the gravity-aligned orientation from init, translation zero.
    if (map_->empty()) {
      RCLCPP_INFO(get_logger(), "first scan: seeding map, origin defined");
      last_fitness_ = 0.0;
      return true;
    }

    // --- Predict. ICP is a LOCAL optimizer: hand it a guess a few degrees off
    // and it will happily lock onto the wrong wall and report a confident fit.
    // Rotation comes from the gyro (accurate); translation from a constant-
    // velocity model (we have no accel-derived velocity yet).
    const double dt = imu_proc_->last_scan_duration();
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.linear() = pose_.linear() * imu_proc_->last_delta_rot().matrix();
    guess.translation() = use_const_vel_
      ? (pose_.translation() + velocity_ * dt).eval()
      : pose_.translation();

    gicp_.setInputSource(scan);
    gicp_.setInputTarget(map_->target());

    CloudXYZI aligned;
    gicp_.align(aligned, guess.matrix().cast<float>());

    // Score only INLIERS. The no-arg getFitnessScore() averages over every source
    // point, including ones beyond the map's radius that have no correspondence at
    // all -- that measures map coverage, not alignment quality, and falsely
    // rejects good fits.
    last_fitness_ = gicp_.getFitnessScore(max_corr_);

    if (!gicp_.hasConverged() || last_fitness_ > max_fitness_) {
      RCLCPP_WARN(
        get_logger(), "GICP %s (fitness %.3f > %.3f) -- coasting on IMU prediction",
        gicp_.hasConverged() ? "fit too poor" : "did not converge",
        last_fitness_, max_fitness_);
      // Coast: trust the prediction rather than a bad alignment. A wrong pose
      // poisons the map permanently; a slightly stale one recovers next scan.
      updatePose(guess, dt);
      return false;
    }

    Eigen::Isometry3d aligned_pose(gicp_.getFinalTransformation().cast<double>());
    updatePose(aligned_pose, dt);
    return true;
  }

  /// Commit a new pose and refresh the constant-velocity estimate.
  void updatePose(const Eigen::Isometry3d & new_pose, double dt)
  {
    if (dt > 1e-6) {
      velocity_ = (new_pose.translation() - pose_.translation()) / dt;
    }
    pose_ = new_pose;
    // Re-orthonormalize: repeated float<->double round trips through PCL slowly
    // erode the rotation block away from SO(3).
    Eigen::Quaterniond q(pose_.linear());
    pose_.linear() = q.normalized().toRotationMatrix();
  }

  void publishOdom(const std_msgs::msg::Header & header)
  {
    const Eigen::Quaterniond q(pose_.linear());
    const Eigen::Vector3d & t = pose_.translation();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = output_frame_;
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // Velocity is in the world frame; twist is specified in child_frame_id.
    const Eigen::Vector3d v_body = pose_.linear().transpose() * velocity_;
    odom.twist.twist.linear.x = v_body.x();
    odom.twist.twist.linear.y = v_body.y();
    odom.twist.twist.linear.z = v_body.z();
    pub_odom_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = header.stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = output_frame_;
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  void publish(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const CloudXYZI::ConstPtr & cloud,
    const std_msgs::msg::Header & header,
    const std::string & frame)
  {
    if (pub->get_subscription_count() == 0) {
      return;  // debug topic: skip the serialize cost when nobody is looking
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    msg.header.frame_id = frame;
    pub->publish(msg);
  }

  void publish(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const CloudXYZI::ConstPtr & cloud,
    const std_msgs::msg::Header & header)
  {
    publish(pub, cloud, header, output_frame_);  // sensor-frame clouds
  }

  /// The map lives in the world frame, not the sensor frame.
  void publishMap(const std_msgs::msg::Header & header)
  {
    publish(pub_map_, map_->target(), header, world_frame_);
  }

  /// Assemble one lidar frame with the IMU samples spanning it. Caller holds
  /// buf_mutex_. Requires an IMU sample before the scan start and IMU coverage
  /// past scan_start + scan_guard_.
  bool syncMeasure(MeasureGroup & meas)
  {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
      return false;
    }
    const double scan_t = stamp_sec(lidar_buffer_.front());
    const double scan_end = scan_t + scan_guard_;

    // Need an IMU sample before the scan to bracket the start.
    if (stamp_sec(imu_buffer_.front()) > scan_t) {
      RCLCPP_WARN(get_logger(), "dropping lidar frame with no preceding IMU");
      lidar_buffer_.pop_front();
      return false;
    }
    // Wait until IMU covers the end of the scan.
    if (stamp_sec(imu_buffer_.back()) < scan_end) {
      return false;
    }

    meas.lidar = lidar_buffer_.front();
    lidar_buffer_.pop_front();

    // Copy IMU covering the scan (keep them buffered for the next scan, which
    // starts inside this window). Include one bracket sample past scan_end.
    meas.imu.clear();
    for (const auto & imu : imu_buffer_) {
      meas.imu.push_back(imu);
      if (stamp_sec(imu) >= scan_end) {
        break;
      }
    }
    // Drop IMU strictly older than this scan start; the rest brackets the next.
    while (imu_buffer_.size() > 1 && stamp_sec(imu_buffer_[1]) <= scan_t) {
      imu_buffer_.pop_front();
    }
    return true;
  }

  std::string lidar_topic_, imu_topic_, output_frame_, world_frame_;
  double scan_guard_ = 0.12;
  double voxel_leaf_ = 0.5;

  std::mutex buf_mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> lidar_buffer_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
  double last_lidar_time_ = -1.0;
  double last_imu_time_ = -1.0;

  std::shared_ptr<ImuProcess> imu_proc_;
  std::unique_ptr<ImuInit> imu_init_;
  Eigen::Quaterniond q_il_ = Eigen::Quaterniond::Identity();  ///< extrinsic lidar -> IMU
  pcl::VoxelGrid<pcl::PointXYZI> voxel_;
  std::unique_ptr<LocalMap> map_;

  /// LIDAR pose in the world frame, set by registration each scan.
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
  /// World-frame linear velocity, from the last pose delta. Feeds the prediction.
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp_;
  double max_fitness_ = 2.0;
  double max_corr_ = 1.0;
  double last_fitness_ = 0.0;
  bool use_const_vel_ = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_deskewed_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace lidar_odom

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_odom::LioNode>());
  rclcpp::shutdown();
  return 0;
}
