#ifndef GLASSLIO_GAUSS_NEWTON_HPP
#define GLASSLIO_GAUSS_NEWTON_HPP

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "sophus/se3.hpp"

namespace glasslio
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

/// The normal equations H xi = b, accumulated one residual at a time.
///
/// This is the whole of Gauss-Newton's bookkeeping: each residual contributes a
/// rank-1 term to H and a gradient term to b. Solving the stacked system is what
/// makes it a least-squares step rather than a gradient step.
class NormalEquations
{
public:
  /// Fold in one residual `r` with its 1x6 Jacobian `J = dr/dxi` (left perturbation).
  void add(double r, const Eigen::Matrix<double, 1, 6> & J, double huber_delta)
  {
    const double w = huberWeight(r, huber_delta);
    H_.noalias() += w * J.transpose() * J;
    b_.noalias() -= w * J.transpose() * r;
    sq_err_ += w * r * r;
    ++n_;
  }

  int count() const {return n_;}

  double rmse() const
  {
    return n_ > 0 ? std::sqrt(sq_err_ / static_cast<double>(n_)) : 0.0;
  }

  /// xi = H^-1 b. LDLT is the right factorisation here: H = sum(w J^T J) is
  /// symmetric positive semi-definite by construction, never indefinite.
  Eigen::Matrix<double, 6, 1> solve() const {return H_.ldlt().solve(b_);}

private:
  Eigen::Matrix<double, 6, 6> H_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> b_ = Eigen::Matrix<double, 6, 1>::Zero();
  double sq_err_ = 0.0;
  int n_ = 0;
};

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

}  // namespace glasslio

#endif  // GLASSLIO_GAUSS_NEWTON_HPP
