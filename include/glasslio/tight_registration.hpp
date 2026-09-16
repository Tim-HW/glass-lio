#ifndef GLASSLIO_TIGHT_REGISTRATION_HPP
#define GLASSLIO_TIGHT_REGISTRATION_HPP

#include <Eigen/Core>

#include "glasslio/types.hpp"   // CloudXYZI, MeasureGroup
#include "glasslio/local_map.hpp"
#include "glass_core/nav_residual.hpp"   // predictState (and the IMU factor itself)
#include "glass_core/nav_state.hpp"
#include "glass_core/preintegration.hpp"

namespace glasslio
{

// The estimation math lives in glass_core; pull its names in (see lio_estimator.hpp).
using namespace glass_core;  // NOLINT(build/namespaces)

/// Gravity, estimated as a free 3-DoF world-frame vector, augments the 15-DoF nav state to
/// 18 (roadmap Phase 1). The tight solve is over [dx_nav (15) ; dg (3)]. Gravity is ONE
/// global vector, not a per-state field, so it lives at the tight-solve level, appended
/// after the nav state -- not inside NavState.
inline constexpr int kIdxGrav = kNavDim;       ///< gravity block offset in the augmented state
inline constexpr int kTightDim = kNavDim + 3;  ///< 18 = nav (15) + gravity (3)

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
  double lidar_sigma = 0.02;

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

  /// DEGENERACY GATE for the deskew bias resync (see LioEstimator::registerScanTight).
  /// Not a scan-rejection threshold -- the pose solve itself is unaffected, since the
  /// IMU information already arbitrates degenerate directions there (see the class
  /// comment above). This ONLY decides whether THIS scan's refined bias is trusted
  /// enough to feed back into deskew's intra-scan motion compensation. Below this
  /// ratio, `TightResult::rotation_eigenvalue_ratio` says the LiDAR alone barely
  /// constrains rotation, so the bias correction the joint solve just produced is
  /// mostly an IMU-side dead-reckoning guess, not a LiDAR-verified refinement --
  /// feeding it into deskew would let a single degenerate scan corrupt every
  /// subsequent scan's geometry, compounding rather than correcting. Same calibrated
  /// starting value as the loose path's analogous translation gate (registration.hpp).
  double min_rotation_eigenvalue_ratio = 0.05;
};

struct TightResult
{
  NavState state;
  /// The solved gravity (world frame). Carried as the next initial guess; the prior
  /// remains anchored to the fixed initialization value.
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  /// The total information matrix at the last linearization, H = sum(J^T Omega J), over the FULL
  /// 18-DoF augmented state [nav (15) ; gravity (3)].
  ///
  /// Includes the bias and gravity priors already. Marginal covariances are blocks of
  /// H^-1, NOT inverses of H's diagonal blocks. This is a local Gaussian approximation
  /// with the last robust weights; it does not make the estimator a complete filter.
  Eigen::Matrix<double, kTightDim, kTightDim> H =
    Eigen::Matrix<double, kTightDim, kTightDim>::Zero();
  /// Solve H P = I. On non-finite or non-positive-definite H, leave covariance unchanged.
  bool posteriorCovariance(Eigen::Matrix<double, kTightDim, kTightDim> & covariance) const;
  bool valid = false;
  bool converged = false;
  int iterations = 0;
  int correspondences = 0;
  double rmse = 0.0;    ///< RMS point-to-plane residual (m), LiDAR only -- comparable
                        ///< with the loose path's rmse.

  /// DIAGNOSTIC (not a gate -- nothing rejects or reweights on this yet). Smallest-
  /// eigenvalue/trace ratio of the LiDAR-ONLY rotation block (kIdxPhi, 3x3) of H,
  /// captured before the IMU/bias/gravity blocks are added each iteration -- i.e. what
  /// the LiDAR ALONE can see about rotation, independent of how much the IMU factor is
  /// backfilling. The loose path has an analogous check on its TRANSLATION block (see
  /// RegistrationParams::min_translation_eigenvalue_ratio) that actually gates; this is
  /// the rotational counterpart, logged only, to find out whether the tight path's
  /// oscillating-APE symptom on open outdoor stretches lines up with yaw becoming
  /// LiDAR-degenerate there. 0 if never computed (e.g. rejected on correspondence count).
  double rotation_eigenvalue_ratio = 0.0;
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
  const Eigen::Vector3d & gravity_prior,
  const NavState & guess,
  const Eigen::Matrix<double, 6, 6> & bias_information,
  const Eigen::Matrix3d & gravity_information,
  const Eigen::Matrix<double, kNavDim, kNavDim> & xi_cov,
  const TightParams & params);

// predictState() now lives in glass_core/nav_residual.hpp, beside the imuResidual it is
// the forward dual of -- it is engine, not LiDAR, and glassvio needs it too. The
// using-directive above re-exports it into glasslio, so call sites are unchanged.

}  // namespace glasslio

#endif  // GLASSLIO_TIGHT_REGISTRATION_HPP
