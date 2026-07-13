#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_odom/data_process.h"
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

    imu_proc_ = std::make_shared<ImuProcess>(get_logger());
    imu_proc_->set_extrinsic(parseQuat(q_xyzw), parseVec3(t_xyz));

    map_ = std::make_unique<LocalMap>(map_voxel, map_max_pts, map_range);

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
    imu_buffer_.push_back(msg);
    tryProcess();
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buf_mutex_);
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
      // --- Stage 1: deskew (motion compensation, see doc/deskew.md) ---
      auto deskewed = imu_proc_->Process(meas);
      if (!deskewed || deskewed->empty()) {
        continue;
      }

      // --- Stage 2: downsample ---
      CloudXYZI::Ptr downsampled(new CloudXYZI());
      voxel_.setInputCloud(deskewed);
      voxel_.filter(*downsampled);

      // --- Stage 3 (Phase 2, TODO): register `downsampled` against map_ ---
      // Registration will set pose_ here. Until it exists pose_ stays identity,
      // so the map accumulates every scan at the origin and SMEARS as soon as the
      // sensor moves. That is expected: closing this gap is exactly what
      // registration does. The map structure itself is pinned by test_local_map.

      // --- Stage 4: fold the aligned scan into the map, in the world frame ---
      CloudXYZI::Ptr scan_world(new CloudXYZI());
      pcl::transformPointCloud(*downsampled, *scan_world, pose_.matrix());
      map_->insert(*scan_world);
      map_->prune(pose_.translation());

      RCLCPP_INFO(
        get_logger(),
        "scan: %zu -> %zu pts (%.0f%% kept) | map: %zu voxels, %zu pts",
        deskewed->size(), downsampled->size(),
        100.0 * static_cast<double>(downsampled->size()) /
        static_cast<double>(deskewed->size()),
        map_->num_voxels(), map_->num_points());

      publish(pub_deskewed_, deskewed, meas.lidar->header);
      publish(pub_downsampled_, downsampled, meas.lidar->header);
      publishMap(meas.lidar->header);
    }
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
  pcl::VoxelGrid<pcl::PointXYZI> voxel_;
  std::unique_ptr<LocalMap> map_;

  /// Sensor pose in the world frame. Identity until registration (Phase 2) sets it.
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_deskewed_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
};

}  // namespace lidar_odom

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_odom::LioNode>());
  rclcpp::shutdown();
  return 0;
}
