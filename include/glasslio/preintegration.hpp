#ifndef GLASSLIO_PREINTEGRATION_HPP
#define GLASSLIO_PREINTEGRATION_HPP

#include <Eigen/Core>

#include "sophus/so3.hpp"

namespace glasslio
{

/// IMU PREINTEGRATION (Forster et al., "On-Manifold Preintegration", TRO 2017).
///
/// THE PROBLEM IT SOLVES. To use the IMU as a constraint between two poses you must
/// integrate it between them. But naive integration is expressed in the WORLD frame:
/// it needs the starting orientation R_i to rotate each acceleration sample into the
/// world. So the moment the optimizer changes its estimate of R_i, every one of those
/// hundreds of samples must be re-integrated. Inside an iterative solve that is
/// ruinous.
///
/// THE TRICK. Integrate in the frame of the FIRST sample instead. Then the result --
/// the "delta" below -- depends only on the IMU measurements and the bias, and is
/// completely INDEPENDENT of R_i, p_i, v_i. The optimizer can move the poses all it
/// likes; the preintegrated quantity never has to be recomputed.
///
///     dR_ij = prod Exp( (w_k - b_g) dt )
///     dv_ij = sum  dR_ik (a_k - b_a) dt
///     dp_ij = sum  [ dv_ik dt + 1/2 dR_ik (a_k - b_a) dt^2 ]
///
/// Gravity is deliberately NOT integrated here. It is a known constant in the world
/// frame, so it is added analytically at residual time (see the residual doc). Folding
/// it in would re-introduce the dependence on R_i we just worked to remove.
///
/// THE BIAS CAVEAT. The deltas DO depend on the bias, and the bias is a state we are
/// estimating. Re-integrating every time the bias nudges would defeat the purpose, so
/// we instead carry the Jacobians d(delta)/d(bias) and apply a FIRST-ORDER correction
/// when the bias changes. That is exact to first order, and it is why `dR_dbg` and
/// friends exist below.
class ImuPreintegration
{
public:
  /// Noise density of the gyro / accelerometer (continuous time):
  ///   gyro_noise  : rad/s/sqrt(Hz)
  ///   accel_noise : m/s^2/sqrt(Hz)
  /// These are datasheet numbers, and then tuned. They set how much the IMU is
  /// TRUSTED relative to the LiDAR -- see covariance().
  ImuPreintegration(
    const Eigen::Vector3d & bias_gyro,
    const Eigen::Vector3d & bias_accel,
    double gyro_noise,
    double accel_noise);

  /// Fold in one IMU sample held constant over `dt` seconds.
  /// `gyro` in rad/s, `accel` in m/s^2 (SI -- scale it before you get here).
  void integrate(
    const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt);

  /// --- The preintegrated delta, relative to the bias it was integrated with.
  const Sophus::SO3d & dR() const {return dR_;}
  const Eigen::Vector3d & dv() const {return dv_;}
  const Eigen::Vector3d & dp() const {return dp_;}
  double dt() const {return dt_;}

  /// --- First-order bias correction. Rather than re-integrating when the bias
  /// estimate moves by `dbg`/`dba`, shift the delta along these Jacobians.
  Sophus::SO3d dR_corrected(const Eigen::Vector3d & dbg) const;
  Eigen::Vector3d dv_corrected(
    const Eigen::Vector3d & dbg, const Eigen::Vector3d & dba) const;
  Eigen::Vector3d dp_corrected(
    const Eigen::Vector3d & dbg, const Eigen::Vector3d & dba) const;

  /// 9x9 covariance of the preintegrated delta, ordered [dphi; dv; dp].
  ///
  /// This is the INFORMATION the IMU carries, and it is what lets the solver weigh
  /// IMU against LiDAR automatically: a long gap between scans means a big covariance
  /// means the IMU is trusted less. It grows without bound the longer you integrate --
  /// which is the honest statement that dead reckoning is not a substitute for a
  /// measurement.
  const Eigen::Matrix<double, 9, 9> & covariance() const {return cov_;}

  /// The bias these deltas were integrated with.
  const Eigen::Vector3d & bias_gyro() const {return bias_gyro_;}
  const Eigen::Vector3d & bias_accel() const {return bias_accel_;}

  /// --- Bias Jacobians (exposed for the residual's Jacobians, and for tests).
  const Eigen::Matrix3d & dR_dbg() const {return dR_dbg_;}
  const Eigen::Matrix3d & dv_dbg() const {return dv_dbg_;}
  const Eigen::Matrix3d & dv_dba() const {return dv_dba_;}
  const Eigen::Matrix3d & dp_dbg() const {return dp_dbg_;}
  const Eigen::Matrix3d & dp_dba() const {return dp_dba_;}

private:
  Eigen::Vector3d bias_gyro_;
  Eigen::Vector3d bias_accel_;
  double gyro_noise_;
  double accel_noise_;

  // The preintegrated delta. Identity/zero at construction: the interval starts empty.
  Sophus::SO3d dR_;
  Eigen::Vector3d dv_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dp_ = Eigen::Vector3d::Zero();
  double dt_ = 0.0;

  Eigen::Matrix<double, 9, 9> cov_ = Eigen::Matrix<double, 9, 9>::Zero();

  Eigen::Matrix3d dR_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_dba_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_dba_ = Eigen::Matrix3d::Zero();
};

}  // namespace glasslio

#endif  // GLASSLIO_PREINTEGRATION_HPP
