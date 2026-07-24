#ifndef GLASS_CORE_NAV_RESIDUAL_HPP
#define GLASS_CORE_NAV_RESIDUAL_HPP

#include <Eigen/Core>

#include "glass_core/nav_state.hpp"
#include "glass_core/preintegration.hpp"
#include "glass_core/so3_jacobian.hpp"

namespace glass_core
{

/// The residuals a tightly-coupled solve stacks into one linear system.
///
/// All Jacobians here are d r / d dx under the RIGHT-perturbation retraction defined in
/// nav_state.hpp (R <- R Exp(dphi); everything else additive). They are NOT the same as
/// the left-perturbation Jacobians used by optimizeSE3 -- see pointToPlaneJacobianNav
/// below, which is the same residual as the loose path but a DIFFERENT derivative.

// =================================================================================
// 1. THE IMU FACTOR. 9 residuals: [r_dR ; r_dv ; r_dp].
// =================================================================================

using ImuResidual = Eigen::Matrix<double, 9, 1>;
using ImuJacobian = Eigen::Matrix<double, 9, kNavDim>;

/// Residual of the preintegrated IMU delta between state `xi` (previous scan, held
/// fixed) and `xj` (current scan, being solved for).
///
///     r_dR = Log( dR_hat^T * R_i^T R_j )
///     r_dv = R_i^T (v_j - v_i - g dt)                 - dv_hat
///     r_dp = R_i^T (p_j - p_i - v_i dt - 1/2 g dt^2)  - dp_hat
///
/// Read each as "what the STATE says happened" minus "what the IMU says happened".
/// Zero when they agree.
///
/// The `_hat` deltas are the preintegrated measurements shifted to xj's CURRENT bias
/// estimate via the first-order correction -- that is what lets the solver move the
/// bias without re-integrating hundreds of samples every iteration.
///
/// GRAVITY ENTERS HERE, not in the preintegration. It was deliberately left out there
/// because folding it in would have re-introduced a dependence on R_i, which is exactly
/// what preintegration exists to remove. Here R_i is in hand, so we add it back.
inline ImuResidual imuResidual(
  const NavState & xi, const NavState & xj,
  const ImuPreintegration & pre, const Eigen::Vector3d & gravity)
{
  const double dt = pre.dt();

  // Bias offset from whatever bias the deltas were integrated with.
  const Eigen::Vector3d dbg = xj.bg - pre.bias_gyro();
  const Eigen::Vector3d dba = xj.ba - pre.bias_accel();

  const Sophus::SO3d dR_hat = pre.dR_corrected(dbg);
  const Eigen::Vector3d dv_hat = pre.dv_corrected(dbg, dba);
  const Eigen::Vector3d dp_hat = pre.dp_corrected(dbg, dba);

  const Eigen::Matrix3d Ri_T = xi.R.matrix().transpose();

  ImuResidual r;
  // Rotations cannot be subtracted. dR_hat^T * (R_i^T R_j) is the RELATIVE rotation
  // between prediction and state -- identity if they agree -- and Log maps that
  // discrepancy into R^3 so least squares can square it.
  r.segment<3>(0) = (dR_hat.inverse() * xi.R.inverse() * xj.R).log();
  r.segment<3>(3) = Ri_T * (xj.v - xi.v - gravity * dt) - dv_hat;
  r.segment<3>(6) =
    Ri_T * (xj.p - xi.p - xi.v * dt - 0.5 * gravity * dt * dt) - dp_hat;
  return r;
}

/// d(imuResidual) / d(dx_j). Analytic; verified against finite differences.
inline ImuJacobian imuJacobian(
  const NavState & xi, const NavState & xj,
  const ImuPreintegration & pre)
{
  const Eigen::Vector3d dbg = xj.bg - pre.bias_gyro();
  const Eigen::Matrix3d Ri_T = xi.R.matrix().transpose();

  const Sophus::SO3d dR_hat = pre.dR_corrected(dbg);
  const Eigen::Vector3d r_dR = (dR_hat.inverse() * xi.R.inverse() * xj.R).log();

  ImuJacobian J = ImuJacobian::Zero();

  // --- r_dR ------------------------------------------------------------------
  // r_dR is a Log, and dphi_j sits INSIDE it. The identity
  //     Log(Exp(phi) Exp(d)) ~= phi + Jr^-1(phi) d
  // says the perturbation does NOT pass through unchanged: the manifold's curvature
  // stretches it by Jr^-1. THIS is what rightJacobianInverse was built for.
  //
  // Replace it with the identity matrix and nothing crashes -- the solver merely
  // mis-weights the rotation residual, worst during aggressive rotation, i.e. exactly
  // when the IMU matters most. The single most-cited bug in preintegration code.
  const Eigen::Matrix3d Jr_inv = rightJacobianInverse(r_dR);
  J.block<3, 3>(0, kIdxPhi) = Jr_inv;

  // d r_dR / d bg: the bias shifts dR_hat, which sits inside the Log too. Chain rule
  // through the first-order bias correction.
  J.block<3, 3>(0, kIdxBg) =
    -Jr_inv * (Sophus::SO3d::exp(r_dR).inverse()).matrix() *
    rightJacobian(pre.dR_dbg() * dbg) * pre.dR_dbg();

  // --- r_dv ------------------------------------------------------------------
  // Note: r_dv does not mention R_j or p_j, so those blocks stay zero. Sparse by
  // construction, not by approximation.
  J.block<3, 3>(3, kIdxVel) = Ri_T;
  J.block<3, 3>(3, kIdxBg) = -pre.dv_dbg();
  J.block<3, 3>(3, kIdxBa) = -pre.dv_dba();

  // --- r_dp ------------------------------------------------------------------
  J.block<3, 3>(6, kIdxPos) = Ri_T;
  J.block<3, 3>(6, kIdxBg) = -pre.dp_dbg();
  J.block<3, 3>(6, kIdxBa) = -pre.dp_dba();

  return J;
}

/// The IMU's own prediction of the next state: xj = xi (+) preintegrated delta.
///
/// The FORWARD DUAL of imuResidual: the same three lines run forwards instead of
/// compared. imuResidual asks "do xi and xj agree with the IMU?"; this asks "given
/// xi, where does the IMU say xj is?". Keeping the two adjacent is deliberate -- edit
/// one without the other and the residual stops being the error of this prediction.
///
/// As a guess it REPLACES a constant-velocity model, and is strictly better
/// information: constant-velocity assumes the acceleration was zero, while this uses
/// the accelerometer that actually measured it.
///
/// GRAVITY ENTERS HERE, exactly as in imuResidual and for the same reason: it was kept
/// out of the preintegration so the deltas would not depend on xi.
inline NavState predictState(
  const NavState & xi, const ImuPreintegration & pre, const Eigen::Vector3d & gravity)
{
  const double dt = pre.dt();

  NavState xj;
  xj.R = xi.R * pre.dR();
  xj.v = xi.v + gravity * dt + xi.R * pre.dv();
  xj.p = xi.p + xi.v * dt + 0.5 * gravity * dt * dt + xi.R * pre.dp();
  // Biases are modelled as constant over one interval; the solve refines them.
  xj.bg = xi.bg;
  xj.ba = xi.ba;
  return xj;
}

/// d(imuResidual) / d(dx_I) -- the OTHER half, wrt the state held fixed.
///
/// WHY IT WAS MISSING. imuJacobian differentiates wrt x_j only, because both front-ends solve
/// for x_j with x_i a constant. That is not merely an omission: passing x_i as a number
/// ASSERTS IT IS PERFECT. Over a short interval the IMU's own information is enormous -- and
/// correctly so -- so an infinitely-certain x_i lets the factor overrule every other
/// measurement, and the solve collapses into dead reckoning. Measured on EuRoC V1_01: with
/// this missing, reprojection error grew monotonically to 643 px while 135 landmarks sat in
/// view, unrejected and simply outvoted. glasslio's README records the same divergence from
/// the LiDAR side ("x_i is held infinitely certain -- a factor, not a filter").
///
/// WHAT IT BUYS. The residual's true covariance is not the delta's alone:
///
///     Sigma_eff = Sigma_pre + J_i P_i J_i^T
///
/// i.e. inflate the IMU's information by whatever x_i was already unsure of. That is what
/// lets a caller carry a posterior forward (NormalEquationsN::H() exists for exactly this)
/// without moving to a two-state solve. It is also precisely the block a two-state window
/// would need, so it is not throwaway.
///
/// NO BIAS COLUMNS: imuResidual corrects the deltas using xj.bg / xj.ba, so the bias belongs
/// to j. Those blocks are structurally absent, not merely zero.
///
/// Analytic; verified against finite differences in test_nav_residual.cpp.
inline ImuJacobian imuJacobianI(
  const NavState & xi, const NavState & xj,
  const ImuPreintegration & pre, const Eigen::Vector3d & gravity)
{
  const double dt = pre.dt();
  const Eigen::Vector3d dbg = xj.bg - pre.bias_gyro();
  const Eigen::Matrix3d Ri_T = xi.R.matrix().transpose();

  const Sophus::SO3d dR_hat = pre.dR_corrected(dbg);
  const Eigen::Vector3d r_dR = (dR_hat.inverse() * xi.R.inverse() * xj.R).log();

  ImuJacobian J = ImuJacobian::Zero();

  // --- r_dR ------------------------------------------------------------------
  // Perturbing R_i on the RIGHT puts Exp(-dphi_i) inside the Log, on the far side of dR_hat.
  // Pushing it out costs the adjoint (R_j^T R_i) and the curvature term Jr^-1 -- the same
  // Jr^-1 that imuJacobian's r_dR block needs, and the same one that is silently survivable
  // at small angles.
  J.block<3, 3>(0, kIdxPhi) =
    -rightJacobianInverse(r_dR) * xj.R.matrix().transpose() * xi.R.matrix();

  // --- r_dv ------------------------------------------------------------------
  // r_dv = R_i^T (v_j - v_i - g dt) - dv_hat. Under R_i <- R_i Exp(dphi),
  // R_i^T -> (I - dphi^) R_i^T, so the term picks up -dphi^ a = +hat(a) dphi.
  const Eigen::Vector3d a_v = Ri_T * (xj.v - xi.v - gravity * dt);
  J.block<3, 3>(3, kIdxPhi) = Sophus::SO3d::hat(a_v);
  J.block<3, 3>(3, kIdxVel) = -Ri_T;

  // --- r_dp ------------------------------------------------------------------
  const Eigen::Vector3d a_p =
    Ri_T * (xj.p - xi.p - xi.v * dt - 0.5 * gravity * dt * dt);
  J.block<3, 3>(6, kIdxPhi) = Sophus::SO3d::hat(a_p);
  J.block<3, 3>(6, kIdxPos) = -Ri_T;
  J.block<3, 3>(6, kIdxVel) = -Ri_T * dt;

  return J;
}

// =================================================================================
// 2. THE LIDAR FACTOR, re-derived for the RIGHT perturbation.
// =================================================================================

/// Point-to-plane residual for a point in the SENSOR frame:
///
///     r = n^T ( R p_sensor + p - c )
///
/// Identical geometry to the loose path -- the same plane, the same signed distance.
inline double pointToPlaneResidualNav(
  const NavState & x, const Eigen::Vector3d & p_sensor,
  const Eigen::Vector3d & centroid, const Eigen::Vector3d & normal)
{
  return normal.dot(x.R * p_sensor + x.p - centroid);
}

/// d r / d dx under the RIGHT perturbation.
///
/// >> THIS IS NOT THE SAME JACOBIAN AS pointToPlaneJacobian() IN registration.hpp. <<
///
/// Same residual, different retraction, therefore a different derivative. The loose path
/// perturbs on the LEFT (world frame) and gets [n^T, (q x n)^T]. Here the rotation is
/// perturbed on the RIGHT (body frame):
///
///     R Exp(dphi) p  ~=  R (p + dphi x p)  =  R p - R p^ dphi
///     => d r / d dphi  =  -n^T R p^                  (p^ = hat(p_sensor))
///     => d r / d dp    =   n^T                       (world-frame translation)
///
/// Copy the left-perturbation Jacobian into this solver and it still runs, still
/// converges, and is wrong. Pinned by finite differences.
inline Eigen::Matrix<double, 1, kNavDim> pointToPlaneJacobianNav(
  const NavState & x, const Eigen::Vector3d & p_sensor, const Eigen::Vector3d & normal)
{
  Eigen::Matrix<double, 1, kNavDim> J = Eigen::Matrix<double, 1, kNavDim>::Zero();
  J.block<1, 3>(0, kIdxPhi) =
    -normal.transpose() * x.R.matrix() * Sophus::SO3d::hat(p_sensor);
  J.block<1, 3>(0, kIdxPos) = normal.transpose();
  // The LiDAR says nothing about velocity or biases -- those columns are structurally
  // zero. The IMU is the only thing constraining them, which is precisely why loose
  // coupling could never estimate them.
  return J;
}

// =================================================================================
// 3. THE BIAS RANDOM-WALK PRIOR. 6 residuals.
// =================================================================================

/// Biases are UNOBSERVABLE when the vehicle is not excited: with no rotation and no
/// acceleration, a gyro bias and a genuine slow turn look identical, and nothing in the
/// data can separate them. Left free, the biases would wander into whatever value makes
/// the residuals incrementally smaller -- absorbing real motion into themselves.
///
/// This anchors them to the previous estimate, weighted by how fast a bias is physically
/// allowed to drift. It is the numerical statement of "biases change SLOWLY".
///
///     r = [ b_g,j - b_g,i ; b_a,j - b_a,i ]
inline Eigen::Matrix<double, 6, 1> biasResidual(const NavState & xi, const NavState & xj)
{
  Eigen::Matrix<double, 6, 1> r;
  r.segment<3>(0) = xj.bg - xi.bg;
  r.segment<3>(3) = xj.ba - xi.ba;
  return r;
}

inline Eigen::Matrix<double, 6, kNavDim> biasJacobian()
{
  Eigen::Matrix<double, 6, kNavDim> J = Eigen::Matrix<double, 6, kNavDim>::Zero();
  J.block<3, 3>(0, kIdxBg) = Eigen::Matrix3d::Identity();
  J.block<3, 3>(3, kIdxBa) = Eigen::Matrix3d::Identity();
  return J;
}

// =================================================================================
// 4. THE STATE PRIOR. 15 residuals -- the bias prior, widened to the whole state.
// =================================================================================

using PriorResidual = Eigen::Matrix<double, kNavDim, 1>;
using PriorJacobian = Eigen::Matrix<double, kNavDim, kNavDim>;

/// "x should be near `ref`, with information Omega."  r = x [-] ref.
///
/// WHY THIS EXISTS. The IMU factor takes TWO states and only ever gets a Jacobian for the
/// second: imuJacobian is d r / d dx_j, and x_i enters as a CONSTANT. That silently asserts
/// the previous state was perfect. It is not, and the assertion is expensive -- over a short
/// interval the IMU's own information is enormous and correctly so, so an infinitely-certain
/// x_i lets the factor overrule every other measurement and the solve degenerates into dead
/// reckoning. Measured on EuRoC: reprojection error grew monotonically to 643 px while 135
/// landmarks sat in view, unrejected and unheeded.
///
/// A prior is what gives the previous state a FINITE certainty. `ref` is the prediction
/// carried forward, and Omega is the inverse of the covariance carried with it -- which is
/// what NormalEquationsN::H() has always been for ("callers that carry a POSTERIOR covariance
/// forward: P_posterior = (P_prior^-1 + H)^-1").
///
/// This is the same shape as the bias prior above, and for the same reason. That one anchors
/// 6 DoF because biases are unobservable when the vehicle is not excited; this anchors 15
/// because the previous state is uncertain rather than unknown.
inline PriorResidual priorResidual(const NavState & x, const NavState & ref)
{
  return boxminus(x, ref);
}

/// d(priorResidual) / d(dx) under the RIGHT perturbation.
///
/// NOT THE IDENTITY, and that is the whole subtlety. The vector blocks are identity, but the
/// rotation block is not: r_phi = Log(R_ref^-1 R Exp(dphi)), and dphi sits INSIDE the Log.
/// The identity
///     Log(Exp(phi) Exp(d)) ~= phi + Jr^-1(phi) d
/// says the perturbation is stretched by the manifold's curvature on its way out.
///
/// Replace it with I and nothing crashes -- the prior is merely mis-weighted in rotation,
/// worst when the state has moved far from `ref`, i.e. exactly when the prior is doing work.
/// The same term, and the same trap, as imuJacobian's r_dR block. Jr^-1 -> I as phi -> 0,
/// which is precisely why it survives testing.
inline PriorJacobian priorJacobian(const NavState & x, const NavState & ref)
{
  PriorJacobian J = PriorJacobian::Identity();
  J.block<3, 3>(kIdxPhi, kIdxPhi) = rightJacobianInverse((ref.R.inverse() * x.R).log());
  return J;
}

}  // namespace glass_core

#endif  // GLASS_CORE_NAV_RESIDUAL_HPP
