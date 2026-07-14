#ifndef GLASSLIO_TIGHT_REGISTRATION_HPP
#define GLASSLIO_TIGHT_REGISTRATION_HPP

#include <Eigen/Core>

#include "glasslio/data_process.h"
#include "glasslio/local_map.hpp"
#include "glasslio/nav_state.hpp"
#include "glasslio/preintegration.hpp"

namespace glasslio
{

struct TightParams
{
  // --- LiDAR side: identical semantics to the loose path, so the tuning carries over.
  double max_correspondence_distance = 1.0;
  double huber_delta = 0.2;
  int max_iterations = 30;
  int min_correspondences = 50;
  double eps_translation = 1e-3;
  double eps_rotation = 1e-4;

  /// Standard deviation of a point-to-plane measurement (m). THE MOST IMPORTANT NUMBER
  /// IN THIS STRUCT, and the easiest to forget.
  ///
  /// In the loose path this could be omitted: with only one sensor, a global scale on
  /// every residual cancels out of `H xi = b` and changes nothing. The moment a SECOND
  /// sensor enters the same normal equations, that scale stops being arbitrary -- it is
  /// what decides which sensor is believed.
  ///
  /// Leave it out (i.e. weight 1.0) and you are implicitly declaring each laser return
  /// accurate to ONE METRE, while the IMU's covariance says millimetres. The IMU then
  /// wins every disagreement, including the ones it should lose, and thousands of
  /// point-to-plane constraints are silently worth less than a single 9-vector.
  double lidar_sigma = 0.05;

  /// How much to trust the IMU factor. Scales the preintegration information matrix.
  ///
  /// 1.0 = trust it exactly as far as its own covariance says. Lower it if the IMU is
  /// fighting good geometry; raise it if the pose slides in degenerate scenes.
  ///
  /// NOTE: 0 does NOT belong here. With zero IMU information, velocity and the biases
  /// have NO constraint at all (the LiDAR Jacobian's columns for them are structurally
  /// zero -- a laser return knows nothing about velocity), so H would be rank-deficient
  /// and the solve meaningless. The node handles `imu_prior_weight == 0` by running the
  /// LOOSE path instead, which is the honest interpretation: with no IMU information
  /// those states are unobservable, so we do not pretend to estimate them.
  double imu_prior_weight = 1.0;

};

struct TightResult
{
  NavState state;
  /// The total information matrix at the solution, H = sum(J^T Omega J).
  ///
  /// Handed back so the caller can carry a POSTERIOR covariance forward:
  ///     P_posterior = (P_prior^-1 + H_data)^-1
  /// which is the whole difference between a filter and a one-shot factor. Without it,
  /// the estimator can never become MORE certain about a bias than it started, and a
  /// quantity like the accel bias -- which only the data can reveal -- stays frozen.
  Eigen::Matrix<double, kNavDim, kNavDim> H =
    Eigen::Matrix<double, kNavDim, kNavDim>::Zero();
  bool valid = false;
  bool converged = false;
  int iterations = 0;
  int correspondences = 0;
  double rmse = 0.0;    ///< RMS point-to-plane residual (m), LiDAR only -- comparable
                        ///< with the loose path's rmse.
};

/// Tightly-coupled scan registration: ONE Gauss-Newton solve over the 15-DoF nav state,
/// stacking the LiDAR point-to-plane residuals and the preintegrated IMU factor into the
/// SAME normal equations.
///
///     H = sum_lidar J^T J  +  w * J_imu^T Sigma^-1 J_imu  +  bias prior
///
/// That sum is the entire fusion. Where the geometry is well-conditioned, the LiDAR term
/// dominates and ICP effectively wins. Where it is degenerate -- a corridor, where the
/// along-axis direction is a NULL SPACE of the LiDAR term -- the IMU information is the
/// only thing there, and it takes over. No mode switch, no heuristic: the information
/// matrices arbitrate, per-direction, per-iteration.
///
/// `xi` is the previous scan's state, held FIXED (this is odometry, not a sliding
/// window). `guess` seeds the solve -- normally the IMU's own prediction from xi.
///
/// ASSUMES the IMU and LiDAR frames are aligned (R_il = identity). True for the Mid-360.
/// A non-identity extrinsic would have to rotate the preintegrated deltas into the lidar
/// frame first.
/// `bias_information` (6x6, [gyro; accel]) is the CALLER'S current certainty about the
/// biases -- i.e. the inverse of a covariance it carries and updates. It is not derived
/// from a random-walk constant here, because that would pin the bias to its initial value
/// forever: a random-walk prior only says how fast a bias may DRIFT, never how wrong it
/// might have been to begin with.
TightResult alignTightlyCoupled(
  const CloudXYZI & source,
  const LocalMap & map,
  const NavState & xi,
  const ImuPreintegration & pre,
  const Eigen::Vector3d & gravity,
  const NavState & guess,
  const Eigen::Matrix<double, 6, 6> & bias_information,
  const TightParams & params);

/// The IMU's own prediction of the next state: xj = xi (+) preintegrated delta.
///
/// This REPLACES the constant-velocity guess. It is strictly better information: the
/// constant-velocity model assumes the acceleration was zero, while this one uses the
/// accelerometer that actually measured it.
NavState predictState(
  const NavState & xi, const ImuPreintegration & pre, const Eigen::Vector3d & gravity);

}  // namespace glasslio

#endif  // GLASSLIO_TIGHT_REGISTRATION_HPP
