// Self-check for the tightly-coupled residuals and their Jacobians.
//
// Every Jacobian here is hand-derived, and every one of them fails SILENTLY if wrong:
// no NaN, no divergence, just a confident and slowly-wrong trajectory. So each is
// checked against finite differences taken ON the 15-DoF manifold -- perturb via
// boxplus (which uses Exp for the rotation block), never by adding to the state.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "glass_core/nav_residual.hpp"
#include "glass_core/nav_state.hpp"
#include "glass_core/preintegration.hpp"

using namespace glass_core;
using Sophus::SO3d;

static const Eigen::Vector3d kG(0.0, 0.0, -9.80665);

/// A non-trivial pair of states + a real preintegrated interval. Nothing here may be
/// zero or identity: a zero block hides a wrong Jacobian (anything times zero is zero).
struct Fixture
{
  NavState xi, xj;
  ImuPreintegration pre{
    Eigen::Vector3d(0.01, -0.02, 0.005),    // the bias the deltas were integrated with
    Eigen::Vector3d(0.1, 0.05, -0.08),
    1e-3, 1e-2};

  Fixture()
  {
    xi.R = SO3d::exp(Eigen::Vector3d(0.2, -0.4, 0.7));
    xi.p = Eigen::Vector3d(3.0, -1.0, 0.5);
    xi.v = Eigen::Vector3d(1.2, 0.4, -0.1);
    xi.bg = Eigen::Vector3d(0.01, -0.02, 0.005);
    xi.ba = Eigen::Vector3d(0.1, 0.05, -0.08);

    // Feed a spinning, accelerating IMU stream so every Jacobian block is exercised.
    const double dt = 0.005;
    for (int k = 0; k < 100; ++k) {
      const double t = k * dt;
      pre.integrate(
        Eigen::Vector3d(0.3 * std::sin(t), 0.2, 0.4) + xi.bg,
        Eigen::Vector3d(0.5, -0.3 * std::cos(t), 9.7) + xi.ba,
        dt);
    }

    // xj: deliberately NOT the exact IMU prediction -- we want a non-zero residual, or
    // we would be linearising at the solution and half the terms would vanish.
    xj.R = xi.R * pre.dR() * SO3d::exp(Eigen::Vector3d(0.03, -0.02, 0.05));
    xj.p = xi.p + xi.v * pre.dt() + 0.5 * kG * pre.dt() * pre.dt() +
      xi.R * pre.dp() + Eigen::Vector3d(0.05, -0.03, 0.02);
    xj.v = xi.v + kG * pre.dt() + xi.R * pre.dv() + Eigen::Vector3d(0.02, 0.01, -0.03);
    xj.bg = xi.bg + Eigen::Vector3d(1e-3, -2e-3, 5e-4);   // off the integration bias,
    xj.ba = xi.ba + Eigen::Vector3d(-3e-3, 1e-3, 2e-3);   // so the bias columns matter
  }
};

/// Central finite differences of any residual wrt the 15-DoF error state, taken through
/// boxplus. This is the oracle: it shares none of the derivation's assumptions.
template<int M, typename ResidualFn>
static Eigen::Matrix<double, M, kNavDim> numericJacobian(
  const NavState & x, ResidualFn && r_of)
{
  const double h = 1e-7;
  Eigen::Matrix<double, M, kNavDim> J;
  for (int i = 0; i < kNavDim; ++i) {
    NavVec dp = NavVec::Zero(), dm = NavVec::Zero();
    dp[i] = h;
    dm[i] = -h;
    const Eigen::Matrix<double, M, 1> rp = r_of(boxplus(x, dp));
    const Eigen::Matrix<double, M, 1> rm = r_of(boxplus(x, dm));
    J.col(i) = (rp - rm) / (2.0 * h);
  }
  return J;
}

// =================================================================================
// 1. The IMU factor's 9x15 Jacobian. The big one.
// =================================================================================
static void testImuJacobian()
{
  Fixture f;

  // Sanity: the residual must be NON-ZERO, or this test proves nothing.
  const ImuResidual r0 = imuResidual(f.xi, f.xj, f.pre, kG);
  assert(r0.norm() > 1e-3 && "fixture is degenerate: residual is ~0, Jacobians untested");

  const ImuJacobian analytic = imuJacobian(f.xi, f.xj, f.pre);
  const ImuJacobian numeric = numericJacobian<9>(
    f.xj, [&](const NavState & x) {return imuResidual(f.xi, x, f.pre, kG);});

  const double err = (analytic - numeric).cwiseAbs().maxCoeff();
  assert(err < 1e-5 && "IMU Jacobian disagrees with finite differences");

  std::printf("  IMU factor  d(9)/d(15)      : max err %.2e  (|r| = %.3f)   OK\n",
    err, r0.norm());
}

// =================================================================================
// 2. Jr^-1 in the rotation block is LOad-BEARING.
//
//    Replace it with the identity -- the classic shortcut -- and assert the Jacobian
//    becomes measurably wrong. Without this, test 1 cannot tell us whether the Jr^-1
//    term is actually doing anything, or whether we'd pass with it stubbed out.
// =================================================================================
static void testJrInverseIsLoadBearing()
{
  Fixture f;

  const ImuJacobian numeric = numericJacobian<9>(
    f.xj, [&](const NavState & x) {return imuResidual(f.xi, x, f.pre, kG);});

  ImuJacobian naive = imuJacobian(f.xi, f.xj, f.pre);
  naive.block<3, 3>(0, kIdxPhi) = Eigen::Matrix3d::Identity();   // the shortcut

  const double err_naive =
    (naive.block<3, 3>(0, kIdxPhi) - numeric.block<3, 3>(0, kIdxPhi)).cwiseAbs().maxCoeff();
  const double err_true =
    (imuJacobian(f.xi, f.xj, f.pre).block<3, 3>(0, kIdxPhi) -
    numeric.block<3, 3>(0, kIdxPhi)).cwiseAbs().maxCoeff();

  assert(err_true < 1e-6);
  assert(err_naive > 1e-3 && "Jr^-1 made no difference -- is the fixture rotating at all?");

  std::printf(
    "  Jr^-1 is load-bearing       : with %.1e, stubbed to I %.1e   OK\n",
    err_true, err_naive);
}

// =================================================================================
// 3. The LiDAR Jacobian under the RIGHT perturbation -- a DIFFERENT derivative from
//    the left-perturbation one in registration.hpp, for the very same residual.
// =================================================================================
static void testLidarJacobianNav()
{
  Fixture f;
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-5.0, 5.0);

  double worst = 0.0;
  for (int t = 0; t < 200; ++t) {
    const Eigen::Vector3d p_sensor(u(rng), u(rng), u(rng));
    const Eigen::Vector3d c(u(rng), u(rng), u(rng));
    const Eigen::Vector3d n = Eigen::Vector3d(u(rng), u(rng), u(rng)).normalized();

    const auto analytic = pointToPlaneJacobianNav(f.xj, p_sensor, n);
    const auto numeric = numericJacobian<1>(
      f.xj, [&](const NavState & x) {
        Eigen::Matrix<double, 1, 1> r;
        r(0) = pointToPlaneResidualNav(x, p_sensor, c, n);
        return r;
      });

    const double err = (analytic - numeric).cwiseAbs().maxCoeff();
    worst = std::max(worst, err);
    assert(err < 1e-6 && "LiDAR nav Jacobian disagrees with finite differences");
  }

  // The LiDAR must say NOTHING about velocity or the biases -- structurally zero.
  // If these are ever non-zero, a residual is leaking into states it cannot observe.
  const auto J = pointToPlaneJacobianNav(f.xj, Eigen::Vector3d(1, 2, 3),
      Eigen::Vector3d::UnitZ());
  // Extra parens: the commas inside block<1, 3> would otherwise look like macro args.
  assert((J.block<1, 3>(0, kIdxVel).isZero()));
  assert((J.block<1, 3>(0, kIdxBg).isZero()));
  assert((J.block<1, 3>(0, kIdxBa).isZero()));

  std::printf("  LiDAR (right perturbation)  : max err %.2e, vel/bias cols zero   OK\n",
    worst);
}

// =================================================================================
// 4. The two LiDAR Jacobians are NOT interchangeable.
//
//    Same residual, same point, same plane -- but the left-perturbation Jacobian from
//    the loose path is a different derivative, and using it here would be a silent bug.
//    Assert they genuinely differ, so nobody "unifies" them later.
// =================================================================================
static void testLeftAndRightLidarJacobiansDiffer()
{
  Fixture f;
  const Eigen::Vector3d p_sensor(2.0, -1.0, 0.5);
  const Eigen::Vector3d n = Eigen::Vector3d(0.3, 0.6, -0.7).normalized();

  // The world point, as the LEFT-perturbation convention sees it.
  const Eigen::Vector3d q = f.xj.R * p_sensor + f.xj.p;

  const auto right = pointToPlaneJacobianNav(f.xj, p_sensor, n);
  const Eigen::Matrix<double, 1, 3> left_rot = q.cross(n).transpose();  // the loose one

  const double diff =
    (right.block<1, 3>(0, kIdxPhi) - left_rot).cwiseAbs().maxCoeff();
  assert(diff > 1e-2 && "left and right LiDAR Jacobians must not be conflated");

  std::printf("  left vs right LiDAR J       : differ by %.3f                  OK\n", diff);
}

// =================================================================================
// 5. The bias prior: trivial Jacobian, but assert it anyway -- it is the block that
//    keeps the biases from absorbing real motion when they are unobservable.
// =================================================================================
static void testBiasJacobian()
{
  Fixture f;
  const auto analytic = biasJacobian();
  const auto numeric = numericJacobian<6>(
    f.xj, [&](const NavState & x) {return biasResidual(f.xi, x);});

  const double err = (analytic - numeric).cwiseAbs().maxCoeff();
  assert(err < 1e-8);
  std::printf("  bias random-walk prior      : max err %.2e                  OK\n", err);
}

// =================================================================================
// 6. CONSISTENCY: if xj is EXACTLY the IMU's prediction from xi, the residual must be
//    zero. This is the end-to-end check that the residual's frame bookkeeping and the
//    preintegration agree -- gravity, R_i^T, the lot.
// =================================================================================
static void testResidualZeroAtPrediction()
{
  Fixture f;
  const double dt = f.pre.dt();

  // Predict xj from xi, at the SAME bias the deltas were integrated with (so the
  // first-order correction is exactly zero and cannot mask an error).
  NavState pred;
  pred.R = f.xi.R * f.pre.dR();
  pred.v = f.xi.v + kG * dt + f.xi.R * f.pre.dv();
  pred.p = f.xi.p + f.xi.v * dt + 0.5 * kG * dt * dt + f.xi.R * f.pre.dp();
  pred.bg = f.pre.bias_gyro();
  pred.ba = f.pre.bias_accel();

  const ImuResidual r = imuResidual(f.xi, pred, f.pre, kG);
  assert(r.norm() < 1e-12 && "residual must vanish at the IMU's own prediction");

  std::printf("  residual == 0 at prediction : |r| = %.2e                    OK\n", r.norm());
}

int main()
{
  std::printf("test_nav_residual: tightly-coupled residuals and Jacobians\n");
  testImuJacobian();
  testJrInverseIsLoadBearing();
  testLidarJacobianNav();
  testLeftAndRightLidarJacobiansDiffer();
  testBiasJacobian();
  testResidualZeroAtPrediction();
  std::printf("test_nav_residual: all checks passed\n");
  return 0;
}
