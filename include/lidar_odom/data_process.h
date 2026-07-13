#ifndef LIDAR_ODOM_DATA_PROCESS_H
#define LIDAR_ODOM_DATA_PROCESS_H

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sophus/se3.hpp"

#include "lidar_odom/gyr_int.h"
#include "lidar_odom/livox_point.hpp"

namespace lidar_odom
{

/// Raw Livox points, carrying the per-point `timestamp` the deskew needs.
typedef pcl::PointCloud<LivoxPoint> LivoxCloud;
/// Deskewed output. Once motion compensation has consumed the per-point time,
/// timestamp/tag/line are dead weight — dropping to PointXYZI here lets the rest
/// of the pipeline use PCL's precompiled VoxelGrid/GICP instead of custom types.
typedef pcl::PointCloud<pcl::PointXYZI> CloudXYZI;

/// One lidar scan together with the IMU samples covering it (with a bracket
/// sample just before and just after the scan span).
struct MeasureGroup
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};

/// Rotation-only motion compensation (deskew) driven by integrated gyro.
///
/// Per-point time comes from the LivoxPoint `timestamp` field (absolute
/// seconds). Every point is rotated into the scan-end frame using the gyro
/// orientation interpolated at that point's exact acquisition time.
///
/// Translational distortion is intentionally NOT compensated in Phase 1: that
/// needs a velocity estimate, which arrives with the registration loop (Phase 2).
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ImuProcess(rclcpp::Logger logger);

  /// Deskew one measurement group into a motion-compensated cloud (scan-end
  /// frame). Returns nullptr if the group cannot be processed.
  CloudXYZI::Ptr Process(const MeasureGroup & meas);

  /// Extrinsic lidar -> IMU: p_imu = R_il * p_lidar + t_il. Default identity.
  /// `q_il` must be normalized. Only the rotation affects the (rotation-only)
  /// deskew; the translation is stored for translational deskew / tight coupling.
  void set_extrinsic(const Eigen::Quaterniond & q_il, const Eigen::Vector3d & t_il)
  {
    R_il_ = Sophus::SO3d(q_il.normalized());
    t_il_ = t_il;
  }

private:
  Sophus::SO3d R_il_;                                 ///< rotation lidar -> IMU
  Eigen::Vector3d t_il_ = Eigen::Vector3d::Zero();    ///< translation lidar -> IMU (Phase 2/3)
  rclcpp::Logger logger_;
  GyrInt gyr_int_;
};

}  // namespace lidar_odom

#endif  // LIDAR_ODOM_DATA_PROCESS_H
