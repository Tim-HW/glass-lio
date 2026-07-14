#include "glasslio/preintegration.hpp"

#include "glasslio/so3_jacobian.hpp"

namespace glasslio
{

using Sophus::SO3d;

ImuPreintegration::ImuPreintegration(
  const Eigen::Vector3d & bias_gyro,
  const Eigen::Vector3d & bias_accel,
  double gyro_noise,
  double accel_noise)
: bias_gyro_(bias_gyro),
  bias_accel_(bias_accel),
  gyro_noise_(gyro_noise),
  accel_noise_(accel_noise)
{
}

void ImuPreintegration::integrate(
  const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt)
{
  if (dt <= 0.0) {
    return;
  }

  // Bias-corrected measurements. Everything below is relative to THIS bias; the
  // Jacobians accumulated alongside are what let us move off it later without
  // re-integrating.
  const Eigen::Vector3d w = gyro - bias_gyro_;
  const Eigen::Vector3d a = accel - bias_accel_;

  const Eigen::Matrix3d dR_mat = dR_.matrix();       // state BEFORE this step
  const Eigen::Matrix3d a_hat = SO3d::hat(a);
  const SO3d dRk = SO3d::exp(w * dt);                // this step's rotation increment
  const Eigen::Matrix3d Jr = rightJacobian(w * dt);

  // ---------------------------------------------------------------------------
  // 1. COVARIANCE, propagated in the TANGENT space. Must come first: it linearises
  //    about the CURRENT state, so it needs dR before we advance it.
  //
  //    err_{k+1} = A err_k + B noise      =>   cov = A cov A^T + B Q B^T
  //
  //    State error order: [dphi; dv; dp].
  // ---------------------------------------------------------------------------
  Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
  A.block<3, 3>(0, 0) = dRk.matrix().transpose();
  A.block<3, 3>(3, 0) = -dR_mat * a_hat * dt;
  A.block<3, 3>(6, 0) = -0.5 * dR_mat * a_hat * dt * dt;
  A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
  B.block<3, 3>(0, 0) = Jr * dt;
  B.block<3, 3>(3, 3) = dR_mat * dt;
  B.block<3, 3>(6, 3) = 0.5 * dR_mat * dt * dt;

  // Discrete-time noise: a continuous noise DENSITY sigma becomes sigma^2/dt when
  // held over a step. (Halve dt and you take twice as many steps, each noisier --
  // the random walk grows as sqrt(t), not t.)
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.block<3, 3>(0, 0) =
    Eigen::Matrix3d::Identity() * (gyro_noise_ * gyro_noise_ / dt);
  Q.block<3, 3>(3, 3) =
    Eigen::Matrix3d::Identity() * (accel_noise_ * accel_noise_ / dt);

  cov_ = A * cov_ * A.transpose() + B * Q * B.transpose();

  // ---------------------------------------------------------------------------
  // 2. BIAS JACOBIANS (Forster eq. 59-61). Also before the state update, and the
  //    ORDER WITHIN this block matters: dp/dv use the OLD dR_dbg, so dR_dbg must be
  //    advanced LAST. Advance it early and the position Jacobian silently uses a
  //    Jacobian from the wrong step.
  // ---------------------------------------------------------------------------
  dp_dba_ += dv_dba_ * dt - 0.5 * dR_mat * dt * dt;
  dp_dbg_ += dv_dbg_ * dt - 0.5 * dR_mat * a_hat * dR_dbg_ * dt * dt;

  dv_dba_ += -dR_mat * dt;
  dv_dbg_ += -dR_mat * a_hat * dR_dbg_ * dt;

  dR_dbg_ = dRk.matrix().transpose() * dR_dbg_ - Jr * dt;

  // ---------------------------------------------------------------------------
  // 3. THE STATE. Same ordering trap: dp uses the old dv, and both use the old dR.
  //    Update dR first and you have integrated position with the wrong orientation.
  // ---------------------------------------------------------------------------
  dp_ += dv_ * dt + 0.5 * dR_mat * a * dt * dt;
  dv_ += dR_mat * a * dt;
  dR_ = dR_ * dRk;      // compose ON the manifold, on the RIGHT (body-frame increment)

  dt_ += dt;
}

// -----------------------------------------------------------------------------
// First-order bias correction. `dbg`/`dba` are the OFFSET from the bias these deltas
// were integrated with -- not the new bias itself.
//
// Exact to first order in the offset, which is the whole bargain: we trade a little
// accuracy (for a bias that has barely moved) against re-integrating hundreds of
// samples inside every solver iteration. If the bias ever drifts far enough that this
// stops holding, the answer is to re-integrate, not to add higher-order terms.
// -----------------------------------------------------------------------------
Sophus::SO3d ImuPreintegration::dR_corrected(const Eigen::Vector3d & dbg) const
{
  // The correction lives in the tangent space at dR_, so it composes on the RIGHT.
  return dR_ * SO3d::exp(dR_dbg_ * dbg);
}

Eigen::Vector3d ImuPreintegration::dv_corrected(
  const Eigen::Vector3d & dbg, const Eigen::Vector3d & dba) const
{
  return dv_ + dv_dbg_ * dbg + dv_dba_ * dba;   // dv lives in R^3: plain addition
}

Eigen::Vector3d ImuPreintegration::dp_corrected(
  const Eigen::Vector3d & dbg, const Eigen::Vector3d & dba) const
{
  return dp_ + dp_dbg_ * dbg + dp_dba_ * dba;
}

}  // namespace glasslio
