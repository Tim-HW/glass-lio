#include "glasslio/lio_estimator.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/common/transforms.h>

#include "glasslio/ros_time.hpp"

namespace glasslio
{

/// An IMU gap longer than this is a hole in the stream, not a sample interval.
static constexpr double kMaxImuGapSec = 0.1;

LioEstimator::LioEstimator(const EstimatorParams & params, const rclcpp::Logger & logger)
: p_(params),
  logger_(logger),
  deskew_(std::make_shared<Deskew>(logger)),
  map_(std::make_unique<LocalMap>(
      params.map_voxel_size, params.map_max_points_per_voxel, params.map_max_range,
      params.map_min_points_for_plane, params.map_planarity_ratio))
{
  deskew_->set_extrinsic(p_.extrinsic_q_il);
  voxel_.setLeafSize(p_.voxel_leaf_size, p_.voxel_leaf_size, p_.voxel_leaf_size);
  resetBiasCovariance();
}

void LioEstimator::initialize(
  const Eigen::Isometry3d & pose,
  const Eigen::Vector3d & gyro_bias,
  const Eigen::Vector3d & gravity)
{
  pose_ = pose;
  deskew_->set_gyro_bias(gyro_bias);

  // Seed the 15-DoF state. Velocity starts at zero and the accel bias at zero -- NEITHER is
  // observable from a static window. Velocity is measured later by the loose warm-up; the
  // accel bias starts UNCERTAIN (resetBiasCovariance) and is discovered from the data.
  gravity_ = gravity;
  state_ = NavState();
  state_.R = Sophus::SO3d(Eigen::Quaterniond(pose.linear()).normalized());
  state_.p = pose.translation();
  state_.bg = gyro_bias;
  resetBiasCovariance();
}

void LioEstimator::reset()
{
  resetPipelineState();
  scans_done_ = 0;
  prev_scan_end_ = -1.0;
  resetBiasCovariance();
}

/// THE PIPELINE, in the order doc/pipeline.md numbers it. Stages [1] IMU init and [2] sync
/// happened before we were called -- a MeasureGroup IS their product.
ScanResult LioEstimator::processScan(const MeasureGroup & meas)
{
  ScanResult r;

  // --- [3] DESKEW: undo the intra-scan rotation (doc/3-deskew.md) ---
  r.deskewed = deskew_->Process(meas);
  if (!r.deskewed || r.deskewed->empty()) {
    return r;   // ok == false
  }

  // --- [4] DOWNSAMPLE: fewer points for ICP to chew (doc/4-downsample.md) ---
  r.downsampled = downsample(r.deskewed);

  // --- [5] REGISTER: align against the local map -> pose_ (doc/5-registration.md) ---
  r.pose_trusted = registerScan(r.downsampled, meas);

  // --- [6] LOCAL MAP: fold the aligned scan back in (doc/6-local-map.md) ---
  insertIntoMap(r.deskewed, r.pose_trusted);

  r.ok = true;
  r.rmse = last_rmse_;
  r.correspondences = last_corr_;
  return r;
}


/// Apply the requested reset. Worker thread only -- this is the thread that
/// owns the estimator, so nothing can be mid-scan against it.
void LioEstimator::resetPipelineState()
{
  map_ = std::make_unique<LocalMap>(
    p_.map_voxel_size, p_.map_max_points_per_voxel, p_.map_max_range, p_.map_min_points_for_plane,
      p_.map_planarity_ratio);
  map_bootstrapped_ = false;
  pose_ = Eigen::Isometry3d::Identity();
  velocity_.setZero();
  state_ = NavState();
  scans_done_ = 0;
  prev_scan_end_ = -1.0;
  resetBiasCovariance();
}


/// Reset the carried bias covariance to genuine ignorance about the accel bias.
void LioEstimator::resetBiasCovariance()
{
  bias_cov_.setZero();
  bias_cov_.block<3, 3>(0, 0) =
    Eigen::Matrix3d::Identity() * (p_.bias_sigma0_gyro * p_.bias_sigma0_gyro);
  bias_cov_.block<3, 3>(3, 3) =
    Eigen::Matrix3d::Identity() * (p_.bias_sigma0_accel * p_.bias_sigma0_accel);
}


/// [4] Voxel-grid downsample. Feeds ICP only -- the MAP gets the dense cloud, see
/// insertIntoMap().
CloudXYZI::Ptr LioEstimator::downsample(const CloudXYZI::Ptr & cloud)
{
  CloudXYZI::Ptr out(new CloudXYZI());
  voxel_.setInputCloud(cloud);
  voxel_.filter(*out);
  return out;
}


/// [5] Register, loose or tight, and hand back whether the pose can be trusted.
///
/// TIGHT COUPLING NEEDS A VELOCITY TO START FROM, and IMU init cannot give it one: a
/// static window cannot tell REST from CONSTANT VELOCITY -- both show zero rotation and
/// zero acceleration variance (doc/1-imu-init.md). On this bag the robot is already
/// cruising at ~1.5 m/s when recording starts, so init reports "static" and seeds v = 0.
///
/// Loose coupling shrugs that off: velocity is only a prior there, and it self-corrects
/// by finite-differencing poses. Tight coupling CANNOT, because it holds x_i FIXED -- a
/// wrong v_i is treated as CERTAIN, so the IMU factor spends every scan insisting the
/// robot is stationary while the LiDAR insists otherwise. They fight, the residual grows,
/// correspondences collapse, and the estimator free-falls on dead reckoning. (Observed:
/// 532/691 scans rejected, pose to +6 km.)
///
/// So run LOOSE for the first few scans, let ICP MEASURE the velocity, and only then hand
/// a correct initial state to the tight solver.
bool LioEstimator::registerScan(const CloudXYZI::Ptr & scan, const MeasureGroup & meas)
{
  const bool warming_up = p_.use_tight && scans_done_ < p_.tight_warmup_scans;

  const bool ok = (p_.use_tight && !warming_up) ?
    registerScanTight(scan, meas) :
    registerScanLoose(scan);

  if (warming_up && scans_done_ + 1 == p_.tight_warmup_scans) {
    seedNavStateFromLoose();
  }
  ++scans_done_;
  return ok;
}


/// [5] LOOSE registration: the IMU only proposes a guess, then ICP solves alone.
/// Used when imu_prior_weight == 0 (the default). Returns false if ICP failed, in which
/// case we keep the IMU prediction and the scan is NOT inserted into the map.
bool LioEstimator::registerScanLoose(const CloudXYZI::Ptr & scan)
{
  // Bootstrap: nothing to align against. The first scan DEFINES the origin --
  // pose_ keeps the gravity-aligned orientation from init, translation zero.
  if (map_->empty()) {
    RCLCPP_INFO(logger_, "first scan: seeding map, origin defined");
    last_rmse_ = 0.0;
    return true;
  }

  // --- Predict. ICP is a LOCAL optimizer: hand it a guess a few degrees off
  // and it will happily lock onto the wrong wall and report a confident fit.
  // Rotation comes from the gyro (accurate); translation from a constant-
  // velocity model (off by default -- see the runaway note above).
  const double dt = deskew_->last_scan_duration();
  Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
  guess.linear() = pose_.linear() * deskew_->last_delta_rot().matrix();
  guess.translation() = p_.use_constant_velocity ?
    (pose_.translation() + velocity_ * dt).eval() :
    pose_.translation();

  const auto r = alignPointToPlane(*scan, *map_, guess, p_.reg);
  last_rmse_ = r.rmse;
  last_corr_ = r.correspondences;

  // Trust `valid` (enough correspondences + finite solve) and the residual --
  // NOT `converged`. Hitting max_iterations is not failure: ICP routinely
  // plateaus above eps while sitting on a perfectly good fit, and rejecting
  // those threw away good poses and froze the estimator.
  if (!r.valid || r.rmse > p_.max_rmse) {
    RCLCPP_WARN(
      logger_,
      "ICP rejected (%s, rmse %.3f, %d corr) -- coasting, scan NOT added to map",
      r.valid ? "residual too large" : "under-constrained",
      r.rmse, r.correspondences);
    // Coast on the prediction. The pose is now a guess, so the scan must NOT
    // go into the map -- see insertIntoMap().
    updatePose(guess, dt);
    return false;
  }

  if (!r.converged) {
    RCLCPP_DEBUG(
      logger_, "ICP hit max_iterations (rmse %.3f) -- accepting anyway", r.rmse);
  }

  updatePose(r.pose, dt);
  return true;
}


/// Preintegrate the IMU samples spanning this scan, at the CURRENT bias estimate.
///
/// The deltas are computed relative to `state_.bg` / `state_.ba`; if the solve then
/// moves the bias, the first-order Jacobians shift the deltas instead of forcing a
/// re-integration. That is the whole point of preintegration.
/// Integrate over EXACTLY (t_begin, t_end] -- the previous scan's end to this one's.
///
/// THE WINDOW IS NOT THE MEASUREGROUP. `meas.imu` deliberately over-covers the scan:
/// it carries a bracket sample before the start and IMU past the end (see
/// doc/2-sync.md), so it spans ~0.12 s. But deskew compensates every point into the
/// scan-END frame, so consecutive POSES are 0.10 s apart.
///
/// Integrate the whole group and the IMU factor asserts "over 0.12 s you moved dp"
/// against a pose delta that covers 0.10 s -- a systematic ~20% over-integration, every
/// scan, compounding. It reads exactly like a gravity error: the estimator accelerates
/// away and Z falls quadratically. (Observed: Z to -1 km.)
///
/// So the interval is clipped at both edges, including partial sample intervals.
ImuPreintegration LioEstimator::buildPreintegration(
  const MeasureGroup & meas, double t_begin, double t_end) const
{
  ImuPreintegration pre(state_.bg, state_.ba, p_.gyro_noise, p_.accel_noise);

  for (std::size_t i = 0; i + 1 < meas.imu.size(); ++i) {
    const double ta = stamp_sec(meas.imu[i]);
    const double tb = stamp_sec(meas.imu[i + 1]);

    // Clip this sample's interval into the window.
    const double lo = std::max(ta, t_begin);
    const double hi = std::min(tb, t_end);
    const double dt = hi - lo;
    if (dt <= 0.0 || dt > kMaxImuGapSec) {
      continue;   // outside the window, duplicate, out of order, or a hole
    }

    const auto & m = *meas.imu[i];   // zero-order hold over the interval
    const Eigen::Vector3d w(
      m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
    // SI, always. The Livox reports accel in g (see doc/1-imu-init.md); feeding raw
    // g into an integrator that also adds gravity in m/s^2 would be incoherent.
    const Eigen::Vector3d a = Eigen::Vector3d(
      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z) *
      p_.accel_scale;
    pre.integrate(w, a, dt);
  }
  return pre;
}


/// TIGHTLY-COUPLED registration: one joint solve over the 15-DoF nav state, with the
/// LiDAR residuals and the preintegrated IMU factor in the SAME normal equations.
/// Updates `state_` (and `pose_`/`velocity_`, which are just views of it).
bool LioEstimator::registerScanTight(const CloudXYZI::Ptr & scan, const MeasureGroup & meas)
{
  // The pose refers to the scan-END instant (deskew compensates there), so the IMU
  // factor must span previous-scan-end -> this-scan-end. Nothing else is coherent.
  const double t_end = deskew_->last_scan_end();
  const double t_begin = (prev_scan_end_ > 0.0) ?
    prev_scan_end_ :
    t_end - deskew_->last_scan_duration();

  const ImuPreintegration pre = buildPreintegration(meas, t_begin, t_end);
  if (pre.dt() <= 0.0) {
    return false;   // no usable IMU span; nothing to predict from
  }
  prev_scan_end_ = t_end;

  // RANDOM WALK: the bias may have drifted since the last scan, so we are now slightly
  // LESS certain of it than we were. Covariance grows with time.
  const double dt_s = pre.dt();
  bias_cov_.block<3, 3>(0, 0) +=
    Eigen::Matrix3d::Identity() * (p_.bias_rw_gyro * p_.bias_rw_gyro * dt_s);
  bias_cov_.block<3, 3>(3, 3) +=
    Eigen::Matrix3d::Identity() * (p_.bias_rw_accel * p_.bias_rw_accel * dt_s);

  // PREDICT. This replaces the constant-velocity guess outright: it integrates the
  // accelerometer rather than assuming the acceleration was zero.
  const NavState guess = predictState(state_, pre, gravity_);

  // Bootstrap: the first scan defines the origin. Keep the gravity-aligned attitude
  // from init, zero translation -- there is nothing to align against yet.
  if (map_->empty()) {
    RCLCPP_INFO(logger_, "first scan: seeding map, origin defined");
    last_rmse_ = 0.0;
    last_corr_ = 0;
    return true;
  }

  const TightResult r = alignTightlyCoupled(
    *scan, *map_, state_, pre, gravity_, guess, bias_cov_.inverse(), p_.tight);
  last_rmse_ = r.rmse;
  last_corr_ = r.correspondences;

  if (!r.valid || r.rmse > p_.max_rmse) {
    RCLCPP_WARN(
      logger_,
      "tight solve rejected (%s, rmse %.3f, %d corr) -- coasting on IMU, scan NOT added",
      r.valid ? "residual too large" : "under-constrained", r.rmse, r.correspondences);
    // Coast on the IMU prediction. Note this is a REAL dead-reckon now (velocity and
    // bias are states, driven by the accelerometer), not a constant-velocity guess.
    commitState(guess);
    return false;
  }

  // POSTERIOR: fold in what this solve actually learned about the biases.
  //
  //     P_posterior = (P_prior^-1 + H_bias)^-1
  //
  // THIS is the difference between a filter and a one-shot factor. Without it the
  // estimator can never become more certain of a bias than it was at startup, so a
  // quantity only the data can reveal -- the accel bias -- stays frozen at its initial
  // guess forever, and every error it causes is permanent.
  const Eigen::Matrix<double, 6, 6> posterior_info =
    bias_cov_.inverse() + r.H.block<6, 6>(kIdxBg, kIdxBg);
  const Eigen::Matrix<double, 6, 6> posterior_cov = posterior_info.inverse();
  if (posterior_cov.allFinite()) {
    bias_cov_ = posterior_cov;
  }

  commitState(r.state);
  return true;
}


/// Hand the loose path's answer to the tight solver: the pose it converged on, and --
/// the part that actually matters -- the VELOCITY it measured by finite-differencing.
/// That is the number IMU init could not observe.
void LioEstimator::seedNavStateFromLoose()
{
  state_ = NavState();
  state_.R = Sophus::SO3d(Eigen::Quaterniond(pose_.linear()).normalized());
  state_.p = pose_.translation();
  state_.v = velocity_;
  // gyro bias is already in state_.bg (set by initialize())
  // Accel bias starts at zero -- and, crucially, starts UNCERTAIN. A static window
  // cannot observe it, so we say so, and let the data reveal it.
  resetBiasCovariance();

  RCLCPP_INFO(
    logger_,
    "tight coupling engaged after %d warm-up scans. seeded v = [%+.2f %+.2f %+.2f] m/s "
    "(|v| = %.2f) -- IMU init could not observe this.",
    p_.tight_warmup_scans, state_.v.x(), state_.v.y(), state_.v.z(), state_.v.norm());
}


/// Commit a nav state and mirror it into pose_/velocity_, which the map insert, the
/// odometry publisher and the TF broadcaster all read.
void LioEstimator::commitState(const NavState & x)
{
  state_ = x;
  pose_.linear() = x.R.matrix();
  pose_.translation() = x.p;
  velocity_ = x.v;
}


/// Commit a new pose and refresh the constant-velocity estimate.
void LioEstimator::updatePose(const Eigen::Isometry3d & new_pose, double dt)
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


/// [6] Fold the aligned scan into the map, in the world frame.
///
/// Insert ONLY if we trust the pose. A scan placed at a known-wrong pose POISONS the map
/// -- and the map is what the NEXT scan registers against, so the error compounds into a
/// death spiral: the pose freezes, the robot keeps moving, each frozen-pose scan corrupts
/// the map further, correspondences collapse, and it never recovers.
///
/// EXCEPT while bootstrapping. Until the map has supported one successful registration it
/// is too sparse to register against (a voxel needs >= 5 points before PCA yields a
/// plane), so refusing to insert would deadlock: sparse map -> ICP under-constrained ->
/// no insert -> map stays sparse, forever. During bootstrap we dead-reckon on the IMU
/// prior and keep feeding the map anyway.
void LioEstimator::insertIntoMap(const CloudXYZI::Ptr & deskewed, bool pose_trusted)
{
  if (pose_trusted) {
    map_bootstrapped_ = true;
  }
  if (!pose_trusted && map_bootstrapped_) {
    return;
  }

  // Feed the map the DENSE cloud, not the downsampled one. Downsampling is an ICP
  // *source* optimisation; the map is the plane-fitting *target* and needs the density.
  // At a 0.5 m leaf the downsampled cloud gives ~1 point per map voxel -- never enough
  // for a plane.
  CloudXYZI::Ptr scan_world(new CloudXYZI());
  pcl::transformPointCloud(*deskewed, *scan_world, pose_.matrix());
  map_->insert(*scan_world);
  map_->prune(pose_.translation());
}

}  // namespace glasslio
