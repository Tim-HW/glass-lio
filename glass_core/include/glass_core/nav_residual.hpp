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

}  // namespace glass_core

#endif  // GLASS_CORE_NAV_RESIDUAL_HPP
