#ifndef GLASS_CORE_GAUSS_NEWTON_HPP
#define GLASS_CORE_GAUSS_NEWTON_HPP

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "sophus/se3.hpp"

namespace glass_core
{

/// Iterative least squares on the SE(3) manifold. Knows nothing about LiDAR,
/// point clouds, or planes -- it only ever sees a residual and its Jacobian.
///
/// THE PERTURBATION CONVENTION. This solver linearises about a LEFT perturbation:
///
///     T <- Exp(xi) * T ,    xi = [rho; phi] in se(3)   (translation; rotation)
///
/// i.e. the correction is expressed in the WORLD frame and therefore composes on
/// the LEFT. That is not a free choice: it is fixed by the frame the correction
/// lives in. (Contrast GyrInt, where the gyro's delta-theta is measured in the
/// BODY frame and so composes on the RIGHT: R <- R * Exp(delta_theta). Same
/// manifold, same Exp, opposite side. Swapping them silently produces an
/// estimator that still runs, still converges, and is wrong.)
///
/// Every Jacobian handed to this solver must therefore be dr/dxi under the LEFT
/// perturbation. test_jacobian.cpp pins that against finite differences.

/// Huber weight: 1 near zero, delta/|r| in the tail. Squared error grows
/// quadratically, so without this a handful of gross residuals (a moving car, a
/// wall seen for the first time) would dominate a solve of thousands of good ones.
inline double huberWeight(double r, double delta)
{
  const double abs_r = std::abs(r);
  return abs_r <= delta ? 1.0 : delta / abs_r;
}

/// The normal equations H xi = b, accumulated one residual at a time, for a state of
/// dimension N.
///
/// This is the whole of Gauss-Newton's bookkeeping: each residual contributes to H and
/// b, and solving the stacked system is what makes it a least-squares step rather than
/// a gradient step.
///
/// TWO WAYS IN, because sensors contribute differently:
///
///   addScalar  -- ONE scalar residual, robustly weighted. LiDAR: thousands of these,
///                 each a point-to-plane distance, each individually suspect (a moving
///                 car, a mis-association), hence Huber.
///
///   addBlock   -- ONE vector residual with a full information matrix. The IMU factor
///                 is 9 correlated numbers (dphi, dv, dp) whose uncertainties are
///                 coupled -- you cannot weight them one at a time, you need Sigma^-1.
///                 No Huber: a preintegrated IMU delta is not an outlier candidate the
///                 way a single laser return is.
///
/// BOTH FOLD INTO THE SAME H AND b. That sum IS the sensor fusion. There is no other
/// fusion step, no filter, no blending coefficient -- just two sets of Jacobians
/// stacked into one linear system, each weighted by how much it actually knows.
template<int N>
class NormalEquationsN
{
public:
  using Vec = Eigen::Matrix<double, N, 1>;
  using Mat = Eigen::Matrix<double, N, N>;
  using RowJ = Eigen::Matrix<double, 1, N>;

  /// One scalar residual `r` with its 1xN Jacobian, Huber-weighted.
  void addScalar(double r, const RowJ & J, double huber_delta)
  {
    const double w = huberWeight(r, huber_delta);
    H_.noalias() += w * J.transpose() * J;
    b_.noalias() -= w * J.transpose() * r;
    sq_err_ += w * r * r;
    ++n_;
  }

  /// One M-dimensional residual with its MxN Jacobian and MxM information matrix
  /// (Omega = Sigma^-1). The information is what makes this self-weighting: a delta
  /// integrated over a long gap has a big Sigma, hence a small Omega, and the solver
  /// leans on it less. No heuristic decides that -- the covariance does.
  template<int M>
  void addBlock(
    const Eigen::Matrix<double, M, 1> & r,
    const Eigen::Matrix<double, M, N> & J,
    const Eigen::Matrix<double, M, M> & information)
  {
    H_.noalias() += J.transpose() * information * J;
    b_.noalias() -= J.transpose() * information * r;
    sq_err_ += r.transpose() * information * r;
    n_ += M;
  }

  /// Backwards-compatible alias: the SE(3)-only path calls this.
  void add(double r, const RowJ & J, double huber_delta) {addScalar(r, J, huber_delta);}

  int count() const {return n_;}

  double rmse() const
  {
    return n_ > 0 ? std::sqrt(sq_err_ / static_cast<double>(n_)) : 0.0;
  }

  /// The accumulated information matrix. Needed by callers that carry a POSTERIOR
  /// covariance forward: P_posterior = (P_prior^-1 + H)^-1.
  const Mat & H() const {return H_;}

  /// xi = H^-1 b. LDLT is the right factorisation here: H = sum(J^T Omega J) is
  /// symmetric positive semi-definite by construction, never indefinite.
  Vec solve() const {return H_.ldlt().solve(b_);}

private:
  Mat H_ = Mat::Zero();
  Vec b_ = Vec::Zero();
  double sq_err_ = 0.0;
  int n_ = 0;
};

/// The SE(3)-only case: LiDAR alone, no IMU in the state. This is what
/// registration.cpp uses today.
using NormalEquations = NormalEquationsN<6>;

struct GaussNewtonParams
{
  int max_iterations = 30;
  /// Converged once BOTH increments fall below these.
  double eps_translation = 1e-3;   ///< m
  double eps_rotation = 1e-4;      ///< rad
  double huber_delta = 0.2;
  /// Below this many residuals the 6-DoF problem is under-constrained; refuse.
  int min_residuals = 50;
};

struct GaussNewtonResult
{
  /// Enough residuals and a finite solve: the state is usable. This -- NOT
  /// `converged` -- is the trust signal.
  bool valid = false;
  /// The increment fell below eps before max_iterations. Not a trust signal:
  /// the solver routinely plateaus above eps while sitting on a good fit, and
  /// treating "hit max_iterations" as failure throws away good states.
  bool converged = false;
  int iterations = 0;
  int residuals = 0;
  double rmse = 0.0;
};

/// Gauss-Newton on SE(3). `accumulate(T, eq)` is the ONLY problem-specific part:
/// it is called once per iteration with the current estimate, and must push each
/// residual and its Jacobian into `eq`. Re-running it every iteration is what
/// makes this iterative -- for ICP, that re-association is the outer loop.
///
/// `T` is updated in place. Returns why it stopped.
template<typename Accumulate>
GaussNewtonResult optimizeSE3(
  Sophus::SE3d & T, const GaussNewtonParams & params, Accumulate && accumulate)
{
  GaussNewtonResult result;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    NormalEquations eq;
    accumulate(T, eq);

    result.residuals = eq.count();
    if (eq.count() < params.min_residuals) {
      result.valid = false;   // under-constrained: refuse rather than invent a state
      return result;
    }

    const Eigen::Matrix<double, 6, 1> xi = eq.solve();
    if (!xi.allFinite()) {
      result.valid = false;   // degenerate geometry; do NOT hand back a state
      return result;
    }

    // RETRACT. The increment xi lives in the tangent space se(3); Exp maps it back
    // onto the manifold, and we compose on the LEFT (see the convention note above).
    // Never add xi to the state -- SE(3) is not a vector space.
    T = Sophus::SE3d::exp(xi) * T;

    result.iterations = iter + 1;
    result.rmse = eq.rmse();
    result.valid = true;

    if (xi.head<3>().norm() < params.eps_translation &&
      xi.tail<3>().norm() < params.eps_rotation)
    {
      result.converged = true;
      break;
    }
  }

  return result;
}

}  // namespace glass_core

#endif  // GLASS_CORE_GAUSS_NEWTON_HPP
