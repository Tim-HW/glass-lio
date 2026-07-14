#ifndef GLASSLIO_LIO_ESTIMATOR_HPP
#define GLASSLIO_LIO_ESTIMATOR_HPP

#include <memory>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/logger.hpp>

#include "glasslio/deskew.hpp"
#include "glasslio/local_map.hpp"
#include "glasslio/nav_state.hpp"
#include "glasslio/preintegration.hpp"
#include "glasslio/registration.hpp"
#include "glasslio/tight_registration.hpp"

namespace glasslio
{

/// Everything the estimator needs to be built. Parsed from ROS parameters by the node --
/// which is the node's ONLY remaining say in how the estimator behaves.
struct EstimatorParams
{
  /// [4] Downsample leaf (m). ICP source only; the map gets the dense cloud.
  double voxel_leaf_size = 0.5;

  /// [6] Local map.
  double map_voxel_size = 1.0;
  int map_max_points_per_voxel = 20;
  double map_max_range = 100.0;
  int map_min_points_for_plane = LocalMap::kDefaultMinPointsForPlane;
  double map_planarity_ratio = LocalMap::kDefaultPlanarityRatio;

  /// [5] Registration.
  RegistrationParams reg;
  /// RMS point-to-plane residual (m) above which we coast on the prediction instead. A
  /// wrong pose poisons the map permanently; a stale one recovers next scan.
  double max_rmse = 0.5;
  bool use_constant_velocity = true;

  /// [7] Tight coupling. `use_tight` is `imu_prior_weight > 0`.
  bool use_tight = false;
  TightParams tight;
  /// Scans to run LOOSE first, so ICP can MEASURE the velocity that IMU init cannot
  /// observe (a static window cannot tell rest from constant velocity).
  int tight_warmup_scans = 10;
  double gyro_noise = 1.7e-3;
  double accel_noise = 2.0e-2;
  double bias_rw_gyro = 1e-4;
  double bias_rw_accel = 1e-3;
  /// INITIAL bias uncertainty -- how wrong the starting bias might be. Distinct from the
  /// random walk, which only says how fast it may drift from wherever it already is.
  double bias_sigma0_gyro = 0.01;
  double bias_sigma0_accel = 0.3;

  /// Multiply raw IMU accel by this to get m/s^2 (9.80665 if the driver reports g, which
  /// Livox does -- a sensor_msgs/Imu spec violation).
  double accel_scale = 9.80665;
  /// Extrinsic ROTATION lidar -> IMU. Identity on the Mid-360.
  Eigen::Quaterniond extrinsic_q_il = Eigen::Quaterniond::Identity();
};

/// What one scan produced. The node turns this into log lines and topics; the estimator
/// does no talking of its own.
struct ScanResult
{
  /// The pipeline ran (a cloud came out). False means the scan was unusable.
  bool ok = false;
  /// Registration produced a pose we TRUST. False means we are coasting on the prediction
  /// and the scan was NOT inserted into the map.
  bool pose_trusted = false;

  double rmse = 0.0;
  int correspondences = 0;

  /// Kept so the node can publish them for RViz. Nothing else reads these.
  CloudXYZI::Ptr deskewed;
  CloudXYZI::Ptr downsampled;
};

/// THE ESTIMATOR -- stages [3] deskew, [4] downsample, [5] register, [6] local map.
///
/// This is the pipeline, and nothing else. It has no subscriptions, no publishers, no
/// parameters and no threads: the node feeds it MeasureGroups and asks what came out.
///
/// That separation is not tidiness. It is what makes the estimator DRIVABLE FROM A TEST --
/// including the tightly-coupled path, whose node-side wiring (preintegration windows, the
/// warm-up, the carried bias covariance) was previously reachable by no test at all, and
/// therefore had no safety net whatsoever.
///
/// THREADING: not thread-safe, and does not try to be. The node owns exactly one, touched
/// by exactly one thread (the worker). See doc/pipeline.md#threading.
class LioEstimator
{
public:
  LioEstimator(const EstimatorParams & params, const rclcpp::Logger & logger);

  /// Seed from IMU init: the gravity-aligned pose, the measured gyro bias, and gravity in
  /// the world frame. Nothing runs before this.
  void initialize(
    const Eigen::Isometry3d & pose,
    const Eigen::Vector3d & gyro_bias,
    const Eigen::Vector3d & gravity);

  /// The world we mapped is gone (the source restarted). Drop the map, the pose, and every
  /// derived quantity.
  void reset();

  /// Run one scan through [3] -> [6].
  ScanResult processScan(const MeasureGroup & meas);

  const Eigen::Isometry3d & pose() const {return pose_;}
  const Eigen::Vector3d & velocity() const {return velocity_;}
  const LocalMap & map() const {return *map_;}

private:
  // --- [4]
  CloudXYZI::Ptr downsample(const CloudXYZI::Ptr & cloud);

  // --- [5]
  bool registerScan(const CloudXYZI::Ptr & scan, const MeasureGroup & meas);
  bool registerScanLoose(const CloudXYZI::Ptr & scan);
  bool registerScanTight(const CloudXYZI::Ptr & scan, const MeasureGroup & meas);
  ImuPreintegration buildPreintegration(
    const MeasureGroup & meas, double t_begin, double t_end) const;
  void seedNavStateFromLoose();
  void commitState(const NavState & x);
  void updatePose(const Eigen::Isometry3d & new_pose, double dt);
  void resetBiasCovariance();
  void resetEstimator();

  // --- [6]
  void insertIntoMap(const CloudXYZI::Ptr & deskewed, bool pose_trusted);

  EstimatorParams p_;
  rclcpp::Logger logger_;

  std::shared_ptr<Deskew> deskew_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_;
  std::unique_ptr<LocalMap> map_;

  /// LIDAR pose in the world frame. What registration produces.
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
  /// World-frame linear velocity. In the loose path this is finite-differenced from the
  /// poses it helps produce -- a feedback loop, and the origin of the constant-velocity
  /// runaway. In the tight path it is a proper state.
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();

  /// Has the map ever supported a successful registration? Until it has, it is too sparse
  /// to align against and we must keep feeding it -- see insertIntoMap().
  bool map_bootstrapped_ = false;

  double last_rmse_ = 0.0;
  int last_corr_ = 0;

  // --- tight coupling
  NavState state_;
  Eigen::Vector3d gravity_{0.0, 0.0, -9.80665};
  Eigen::Matrix<double, 6, 6> bias_cov_ = Eigen::Matrix<double, 6, 6>::Identity();
  int scans_done_ = 0;
  /// Scan-end time of the previous scan: the lower edge of the IMU factor's integration
  /// window. -1 until the first tight scan.
  double prev_scan_end_ = -1.0;
};

}  // namespace glasslio

#endif  // GLASSLIO_LIO_ESTIMATOR_HPP
