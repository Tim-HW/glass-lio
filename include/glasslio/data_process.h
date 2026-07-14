#ifndef GLASSLIO_DATA_PROCESS_H
#define GLASSLIO_DATA_PROCESS_H

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sophus/se3.hpp"

#include "glasslio/gyr_int.h"
#include "glasslio/livox_point.hpp"

namespace glasslio
{

/// Raw Livox points, carrying the per-point `timestamp` the deskew needs.
typedef pcl::PointCloud<LivoxPoint> LivoxCloud;
/// Deskewed output. Once motion compensation has consumed the per-point time,
/// timestamp/tag/line are dead weight — dropping to PointXYZI here lets the rest
/// of the pipeline use PCL's precompiled VoxelGrid instead of custom types.
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
/// nanoseconds). Every point is rotated into the scan-end frame using the gyro
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

  /// Extrinsic ROTATION lidar -> IMU. `q_il` must be normalized.
  ///
  /// Only the rotation is taken, because only the rotation is used: deskew is
  /// rotation-only (see doc/3-deskew.md). The extrinsic TRANSLATION is validated at
  /// startup but deliberately not stored -- nothing reads it, and a field that is
  /// written and never read is a lie about what the code does. It comes back in the
  /// commit that actually needs it (translational deskew, or a non-identity extrinsic
  /// under tight coupling).
  void set_extrinsic(const Eigen::Quaterniond & q_il)
  {
    R_il_ = Sophus::SO3d(q_il.normalized());
  }

  /// Gyro bias from ImuInit, subtracted before integration.
  void set_gyro_bias(const Eigen::Vector3d & bias) {gyr_int_.set_bias(bias);}

  /// Rotation of the LIDAR frame across the last processed scan, i.e. the new
  /// scan-end frame expressed in the previous one. Scans are contiguous, so this
  /// is the IMU's prediction of how the sensor turned since the last pose --
  /// exactly the initial guess registration needs.
  const Sophus::SO3d & last_delta_rot() const {return last_delta_rot_;}

  /// Duration (s) of the last processed scan.
  double last_scan_duration() const {return last_dt_;}

  /// Absolute time (s) of the last processed scan's END. Deskew compensates every point
  /// into the scan-END frame, so this is the instant `pose_` actually refers to -- and
  /// therefore the instant an IMU factor between consecutive scans must integrate TO.
  double last_scan_end() const {return last_t1_;}

private:
  Sophus::SO3d last_delta_rot_;
  double last_dt_ = 0.0;
  double last_t1_ = -1.0;
  Sophus::SO3d R_il_;                                 ///< rotation lidar -> IMU
  rclcpp::Logger logger_;
  GyrInt gyr_int_;
};

}  // namespace glasslio

#endif  // GLASSLIO_DATA_PROCESS_H
