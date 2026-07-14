#ifndef GLASSLIO_DESKEW_HPP
#define GLASSLIO_DESKEW_HPP

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sophus/se3.hpp"

#include "glasslio/gyr_int.hpp"
#include "glasslio/types.hpp"

namespace glasslio
{

/// [3] DESKEW -- rotation-only motion compensation, driven by the integrated gyro.
/// See doc/3-deskew.md.
///
/// A scan is not a snapshot: a Livox at 10 Hz spends ~100 ms sweeping, and every point is
/// measured with the sensor at a different orientation. Uncorrected, the error at range r
/// is ~ r * dtheta -- about 2 m at 40 m on this bag.
///
/// Per-point time comes from the LivoxPoint `timestamp` field (absolute NANOSECONDS -- not
/// from `intensity`, which is genuine reflectivity here). Every point is rotated into the
/// SCAN-END frame using the gyro orientation interpolated at its exact acquisition time.
///
/// Translation is deliberately NOT compensated: that needs a trustworthy velocity, which
/// only a tightly-coupled estimator produces (doc/7-tight-coupling.md).
class Deskew
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Deskew(rclcpp::Logger logger);

  /// Deskew one measurement group into a motion-compensated cloud (scan-end
  /// frame). Returns nullptr if the group cannot be processed.
  CloudXYZI::Ptr process(const MeasureGroup & meas);

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

#endif  // GLASSLIO_DESKEW_HPP
