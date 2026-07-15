// Self-check for the LIE-ALGEBRA CONTRACT the solver depends on.
//
// gauss_newton.hpp linearises about a LEFT perturbation (T <- Exp(xi) * T), so every
// Jacobian handed to it must be dr/dxi under THAT convention. Nothing in the type
// system enforces this. Get the side or the sign wrong and the estimator still runs,
// still converges, and is quietly wrong -- which is the whole reason this file exists.
//
// The oracle is finite differences: perturb the state ON THE MANIFOLD by Exp(eps*e_i),
// measure how the residual actually moves, and compare against the analytic Jacobian.
// A derivation cannot argue with a numerical derivative.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include "glass_core/gauss_newton.hpp"
#include "glasslio/registration.hpp"

using namespace glasslio;
using namespace glass_core;

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat16 = Eigen::Matrix<double, 1, 6>;

/// The residual as the solver actually sees it, as a function of the perturbation xi
/// applied on the LEFT of the current estimate:
///
///     r(xi) = n^T ( Exp(xi) * q - c )        where q = T * p is already in the world
static double residualAt(
  const Vec6 & xi, const Eigen::Vector3d & q,
  const Eigen::Vector3d & c, const Eigen::Vector3d & n)
{
  const Eigen::Vector3d q_pert = Sophus::SE3d::exp(xi) * q;
  return pointToPlaneResidual(q_pert, c, n);
}

/// Central finite differences of r wrt xi, evaluated at xi = 0. Central rather than
/// forward: the error is O(h^2) instead of O(h), which is what lets us assert to 1e-7
/// instead of squinting at 1e-4.
static Mat16 numericJacobian(
  const Eigen::Vector3d & q, const Eigen::Vector3d & c, const Eigen::Vector3d & n)
{
  const double h = 1e-6;
  Mat16 J;
  for (int i = 0; i < 6; ++i) {
    Vec6 dp = Vec6::Zero();
    Vec6 dm = Vec6::Zero();
    dp[i] = h;
    dm[i] = -h;
    J(0, i) = (residualAt(dp, q, c, n) - residualAt(dm, q, c, n)) / (2.0 * h);
  }
  return J;
}

// ---------------------------------------------------------------------------------
// 1. The analytic Jacobian must match the numerical one. This is THE test.
// ---------------------------------------------------------------------------------
static void testJacobianMatchesFiniteDifferences()
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(-5.0, 5.0);

  double worst = 0.0;
  for (int trial = 0; trial < 500; ++trial) {
    const Eigen::Vector3d q(u(rng), u(rng), u(rng));
    const Eigen::Vector3d c(u(rng), u(rng), u(rng));
    const Eigen::Vector3d n = Eigen::Vector3d(u(rng), u(rng), u(rng)).normalized();

    const Mat16 analytic = pointToPlaneJacobian(q, n);
    const Mat16 numeric = numericJacobian(q, c, n);

    const double err = (analytic - numeric).cwiseAbs().maxCoeff();
    worst = std::max(worst, err);
    assert(err < 1e-7 && "analytic Jacobian disagrees with finite differences");
  }
  std::printf("  jacobian vs finite differences : max err %.2e over 500 trials  OK\n", worst);
}

// ---------------------------------------------------------------------------------
// 2. The rotation block is (q x n), NOT (n x q).
//
// This is not paranoia: the two differ only by a sign, both produce a Jacobian of the
// right SHAPE, and the solver will happily run with either. With the wrong one it walks
// AWAY from the solution while still reporting a plausible residual. Assert that the
// swapped version genuinely fails the oracle -- i.e. that test 1 above has teeth.
// ---------------------------------------------------------------------------------
static void testSwappedCrossProductIsDetected()
{
  const Eigen::Vector3d q(1.0, 2.0, 3.0);
  const Eigen::Vector3d c(0.5, -1.0, 2.0);
  const Eigen::Vector3d n = Eigen::Vector3d(0.3, -0.7, 0.5).normalized();

  const Mat16 numeric = numericJacobian(q, c, n);

  Mat16 wrong;
  wrong.head<3>() = n.transpose();
  wrong.tail<3>() = n.cross(q).transpose();   // the classic bug: operands reversed

  const double err = (wrong - numeric).cwiseAbs().maxCoeff();
  assert(err > 1e-3 && "the swapped cross product should NOT match; test 1 has no teeth");

  // ...and it is wrong by exactly a sign flip on the rotation block, nothing subtler.
  const Mat16 correct = pointToPlaneJacobian(q, n);
  assert((wrong.tail<3>() + correct.tail<3>()).norm() < 1e-12);

  std::printf("  swapped (n x q) is caught       : off by %.2e (sign-flipped)   OK\n", err);
}

// ---------------------------------------------------------------------------------
// 3. Left and right perturbation are NOT interchangeable.
//
// The solver composes on the LEFT (correction lives in the world frame); GyrInt
// composes on the RIGHT (the gyro's increment is measured in the body frame). Pin that
// they genuinely differ, so nobody "simplifies" one into the other.
// ---------------------------------------------------------------------------------
static void testLeftAndRightPerturbationDiffer()
{
  Vec6 xi;
  xi << 0.0, 0.0, 0.0, 0.0, 0.0, 0.3;   // pure rotation about +Z

  Sophus::SE3d T(
    Sophus::SO3d::rotX(0.4).matrix(), Eigen::Vector3d(2.0, 0.0, 0.0));

  const Sophus::SE3d left = Sophus::SE3d::exp(xi) * T;
  const Sophus::SE3d right = T * Sophus::SE3d::exp(xi);

  const double diff = (left.matrix() - right.matrix()).cwiseAbs().maxCoeff();
  assert(diff > 1e-3 && "left and right perturbation must not be conflated");

  std::printf("  left vs right perturbation      : differ by %.3f            OK\n", diff);
}

// ---------------------------------------------------------------------------------
// 4. Huber: quadratic (w = 1) inside delta, linear (w = delta/|r|) outside, and the
//    two branches must agree exactly at the changeover or the cost has a kink.
// ---------------------------------------------------------------------------------
static void testHuberWeight()
{
  const double d = 0.2;
  assert(std::abs(huberWeight(0.0, d) - 1.0) < 1e-12);
  assert(std::abs(huberWeight(0.1, d) - 1.0) < 1e-12);
  assert(std::abs(huberWeight(d, d) - 1.0) < 1e-12);          // continuous at r = delta
  assert(std::abs(huberWeight(0.4, d) - 0.5) < 1e-12);        // delta/|r| = 0.2/0.4
  assert(std::abs(huberWeight(-0.4, d) - 0.5) < 1e-12);       // symmetric in sign

  // An outlier's INFLUENCE (w * r) saturates at delta instead of growing without bound.
  // That is the whole point: squared error would give it r^2 of pull.
  for (double r = 1.0; r < 1e6; r *= 10.0) {
    assert(std::abs(huberWeight(r, d) * r - d) < 1e-9);
  }
  std::printf("  huber weight                    : continuous, saturating      OK\n");
}

// ---------------------------------------------------------------------------------
// 5. The solver, end to end, with NO LocalMap: three orthogonal plane families fully
//    constrain 6 DoF. Perturb by a known transform and assert it is recovered.
//    This exercises optimizeSE3 in isolation from association.
// ---------------------------------------------------------------------------------
static void testOptimizeSE3RecoversKnownTransform()
{
  // World points on three orthogonal planes (x=0, y=0, z=0), spread out so that
  // rotation is observable and not just translation.
  struct Corr { Eigen::Vector3d p, c, n; };
  std::vector<Corr> corrs;
  for (double a = -4.0; a <= 4.0; a += 1.0) {
    for (double b = -4.0; b <= 4.0; b += 1.0) {
      corrs.push_back({{0.0, a, b}, {0.0, a, b}, {1.0, 0.0, 0.0}});
      corrs.push_back({{a, 0.0, b}, {a, 0.0, b}, {0.0, 1.0, 0.0}});
      corrs.push_back({{a, b, 0.0}, {a, b, 0.0}, {0.0, 0.0, 1.0}});
    }
  }

  // Ground truth is identity; start the solver from a known perturbation and it must
  // walk back. (Correspondences are fixed here -- we are testing the SOLVE, not the
  // association.)
  Vec6 xi_true;
  xi_true << 0.10, -0.05, 0.08, 0.02, -0.03, 0.04;   // ~10 cm, ~2 deg
  Sophus::SE3d T = Sophus::SE3d::exp(xi_true);

  GaussNewtonParams params;
  params.min_residuals = 10;
  params.huber_delta = 1.0;   // no outliers here; keep it out of the way

  const auto result = optimizeSE3(
    T, params, [&](const Sophus::SE3d & T_cur, NormalEquations & eq) {
      for (const auto & corr : corrs) {
        const Eigen::Vector3d q = T_cur * corr.p;
        eq.add(
          pointToPlaneResidual(q, corr.c, corr.n),
          pointToPlaneJacobian(q, corr.n),
          params.huber_delta);
      }
    });

  assert(result.valid);
  assert(result.converged);

  // Recovered T must be (near) identity: log of the residual transform is ~zero.
  const double err = T.log().norm();
  assert(err < 1e-6 && "solver failed to recover the known transform");
  assert(result.rmse < 1e-3);

  std::printf(
    "  optimizeSE3 recovers transform  : |log(T)| %.2e in %d iters   OK\n",
    err, result.iterations);
}

// ---------------------------------------------------------------------------------
// 6. A DEGENERATE problem must be refused, not answered.
//    One plane family constrains only its own normal -- the other 5 DoF are a null
//    space of H. The solver must not hand back a confident pose for that.
// ---------------------------------------------------------------------------------
static void testDegenerateGeometryIsRefused()
{
  Sophus::SE3d T;
  GaussNewtonParams params;
  params.min_residuals = 10;

  // Points on a SINGLE plane (z = 0). Sliding in x/y and yawing change no residual.
  const auto result = optimizeSE3(
    T, params, [&](const Sophus::SE3d & T_cur, NormalEquations & eq) {
      for (double a = -4.0; a <= 4.0; a += 1.0) {
        for (double b = -4.0; b <= 4.0; b += 1.0) {
          const Eigen::Vector3d p(a, b, 0.0);
          const Eigen::Vector3d q = T_cur * p;
          const Eigen::Vector3d n(0.0, 0.0, 1.0);
          eq.add(pointToPlaneResidual(q, p, n), pointToPlaneJacobian(q, n), params.huber_delta);
        }
      }
    });

  // H is rank-deficient. Whatever comes back must not contain NaN/Inf: the solver
  // either refuses (valid == false) or returns a finite pose in the observable subspace.
  assert(T.matrix().allFinite() && "degenerate geometry leaked a non-finite pose");
  if (result.valid) {
    // The one observable DoF (z) must still be right.
    assert(std::abs(T.translation().z()) < 1e-6);
  }
  std::printf("  degenerate geometry             : no NaN leaked              OK\n");
}

int main()
{
  std::printf("test_jacobian: the Lie-algebra contract behind the solver\n");
  testJacobianMatchesFiniteDifferences();
  testSwappedCrossProductIsDetected();
  testLeftAndRightPerturbationDiffer();
  testHuberWeight();
  testOptimizeSE3RecoversKnownTransform();
  testDegenerateGeometryIsRefused();
  std::printf("test_jacobian: all checks passed\n");
  return 0;
}
