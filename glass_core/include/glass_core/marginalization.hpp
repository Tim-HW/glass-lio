#ifndef GLASS_CORE_MARGINALIZATION_HPP
#define GLASS_CORE_MARGINALIZATION_HPP

#include <Eigen/Dense>

namespace glass_core
{

/// Schur-complement marginalization of a linear-Gaussian information form.
///
/// Given the joint normal equations  H dx = b  over a state partitioned as [m ; k] -- the
/// block to MARGINALIZE out first, then the block to KEEP -- eliminate m and return the
/// reduced system on k:
///
///     H_marg = H_kk - H_km H_mm^-1 H_mk
///     b_marg = b_k  - H_km H_mm^-1 b_m
///
/// (H_km = H_mk^T, because an information matrix is symmetric.)
///
/// This is the EXACT summary of every factor that touched the marginalized variable -- the
/// Gaussian prior a fixed-lag smoother carries forward when it drops the oldest state. The
/// alternative -- holding that state FIXED and infinitely certain, which is what glasslio's
/// one-shot tight solve does to x_i -- is precisely the bug this exists to fix: a wrong
/// value for the dropped state is then treated as ground truth, and the estimator freezes
/// against it. See doc/7-tight-coupling.md.
///
/// THE CONTRACT is exactness: solving  H_marg dx_k = b_marg  yields the SAME dx_k as solving
/// the full  H dx = b  and reading off the kept block. That is the definition of
/// marginalization for a linear-Gaussian system, and it is what test_marginalization.cpp
/// pins -- against BOTH the full solve (must match) and the naive "hold m fixed" block-
/// diagonal solve (must differ, or the cross-coupling H_mk was doing nothing).
inline void schurMarginalize(
  const Eigen::MatrixXd & H, const Eigen::VectorXd & b, int m,
  Eigen::MatrixXd & H_marg, Eigen::VectorXd & b_marg)
{
  const int k = static_cast<int>(H.rows()) - m;

  const Eigen::MatrixXd Hmm = H.topLeftCorner(m, m);
  const Eigen::MatrixXd Hmk = H.topRightCorner(m, k);
  const Eigen::MatrixXd Hkk = H.bottomRightCorner(k, k);
  const Eigen::VectorXd bm = b.head(m);
  const Eigen::VectorXd bk = b.tail(k);

  // H_mm is symmetric positive definite when the marginalized block is constrained, so
  // SOLVE against it rather than forming an explicit inverse (more stable, and it is what
  // production code should do).
  const Eigen::LDLT<Eigen::MatrixXd> Hmm_ldlt(Hmm);
  const Eigen::MatrixXd Hmm_inv_Hmk = Hmm_ldlt.solve(Hmk);   // H_mm^-1 H_mk
  const Eigen::VectorXd Hmm_inv_bm = Hmm_ldlt.solve(bm);     // H_mm^-1 b_m

  H_marg = Hkk - Hmk.transpose() * Hmm_inv_Hmk;              // H_kk - H_km H_mm^-1 H_mk
  b_marg = bk - Hmk.transpose() * Hmm_inv_bm;                // b_k  - H_km H_mm^-1 b_m

  // Re-symmetrize: the subtraction above can leave tiny asymmetries from rounding, and a
  // prior information matrix must stay symmetric or later solves drift.
  H_marg = (0.5 * (H_marg + H_marg.transpose())).eval();
}

}  // namespace glass_core

#endif  // GLASS_CORE_MARGINALIZATION_HPP
