// Self-check for IMU preintegration and the SO(3) right Jacobian.
//
// Everything here has the same failure signature: get it wrong and you do not get a
// crash, you get an estimator that runs and is quietly biased. So each property is
// checked against an INDEPENDENT oracle -- finite differences, or brute-force
// integration -- rather than against the derivation that produced it.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "glass_core/preintegration.hpp"
#include "glass_core/so3_jacobian.hpp"

using namespace glass_core;
using Sophus::SO3d;

static const Eigen::Vector3d kGravityVec(0.0, 0.0, -9.80665);

// =================================================================================
// 1. The right Jacobian, against its DEFINING property:
//
//        Exp(phi + delta) ~= Exp(phi) * Exp( Jr(phi) * delta )
//
//    This is not a restatement of the formula -- it is the thing the formula is
//    supposed to mean. If our closed form is wrong, this fails.
// =================================================================================
static void testRightJacobianDefinition()
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-2.0, 2.0);

  double worst = 0.0;
  for (int t = 0; t < 200; ++t) {
    const Eigen::Vector3d phi(u(rng), u(rng), u(rng));
    const Eigen::Vector3d delta = Eigen::Vector3d(u(rng), u(rng), u(rng)) * 1e-6;

    const SO3d lhs = SO3d::exp(phi + delta);
    const SO3d rhs = SO3d::exp(phi) * SO3d::exp(rightJacobian(phi) * delta);

    // Discrepancy as a rotation angle -- the only frame-independent way to compare.
    const double err = (lhs.inverse() * rhs).log().norm();
    worst = std::max(worst, err);
    assert(err < 1e-10 && "rightJacobian does not satisfy its defining property");
  }
  std::printf("  Jr definition Exp(p+d)=Exp(p)Exp(Jr d) : max err %.2e   OK\n", worst);
}

// =================================================================================
// 2. Jr^-1 really is the inverse of Jr, and the LOG property it exists for:
//
//        Log( Exp(phi) * Exp(delta) ) ~= phi + Jr^-1(phi) * delta
// =================================================================================
static void testRightJacobianInverse()
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  std::uniform_real_distribution<double> ang(1e-3, 3.0);   // strictly inside (0, pi)

  double worst_inv = 0.0, worst_log = 0.0;
  for (int t = 0; t < 200; ++t) {
    // DOMAIN MATTERS. Sample |phi| < pi deliberately: beyond a half-turn, Log wraps to
    // the equivalent short-way rotation and no longer returns phi, so comparing tangent
    // VECTORS is meaningless there -- and Jr^-1 is singular at pi besides. (Test 1
    // compares rotations, not vectors, so it is immune to this and samples freely.)
    const Eigen::Vector3d axis = Eigen::Vector3d(u(rng), u(rng), u(rng)).normalized();
    const Eigen::Vector3d phi = axis * ang(rng);

    const Eigen::Matrix3d prod = rightJacobian(phi) * rightJacobianInverse(phi);
    const double err_inv = (prod - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
    worst_inv = std::max(worst_inv, err_inv);
    assert(err_inv < 1e-9 && "Jr * Jr^-1 != I");

    const Eigen::Vector3d delta = Eigen::Vector3d(u(rng), u(rng), u(rng)) * 1e-6;
    const Eigen::Vector3d lhs = (SO3d::exp(phi) * SO3d::exp(delta)).log();
    const Eigen::Vector3d rhs = phi + rightJacobianInverse(phi) * delta;
    const double err_log = (lhs - rhs).norm();
    worst_log = std::max(worst_log, err_log);
    assert(err_log < 1e-9 && "Jr^-1 does not linearise Log");
  }
  std::printf("  Jr^-1: inverse %.2e, Log-linearisation %.2e   OK\n", worst_inv, worst_log);

  // Pin the wrap-around itself, so the domain limit above is documented as BEHAVIOUR
  // and not just a comment: past pi, Log does not return what you put in.
  const Eigen::Vector3d big = Eigen::Vector3d::UnitZ() * (2.0 * M_PI - 0.5);
  const Eigen::Vector3d wrapped = SO3d::exp(big).log();
  assert((wrapped - big).norm() > 1.0 && "expected Log to wrap past pi");
  assert(std::abs(wrapped.norm() - 0.5) < 1e-9);   // it came back the short way
  std::printf("  Log wraps past pi (|phi|=%.2f -> %.2f)                OK\n",
    big.norm(), wrapped.norm());
}

// =================================================================================
// 3. The small-angle branch. theta = 0 is the MOST COMMON input (a gyro at rest over
//    a 5 ms step), and the closed form divides by theta^3 there. Assert no NaN, and
//    assert the branch is CONTINUOUS with the closed form across the threshold --
//    a discontinuity here would inject a tiny jolt every time the robot slowed down.
// =================================================================================
static void testSmallAngleBranch()
{
  const Eigen::Matrix3d Jr0 = rightJacobian(Eigen::Vector3d::Zero());
  assert(Jr0.allFinite() && "rightJacobian(0) produced NaN -- the theta^3 division");
  assert((Jr0 - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-12);

  const Eigen::Matrix3d Ji0 = rightJacobianInverse(Eigen::Vector3d::Zero());
  assert(Ji0.allFinite() && "rightJacobianInverse(0) produced NaN");

  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();

  // CONTINUITY ACROSS THE BRANCH: the two forms must agree where they MEET -- compared
  // at the SAME angle, just above the threshold, where the closed form is what runs.
  // (Comparing two different angles would only measure that Jr varies with theta.)
  const Eigen::Vector3d phi = axis * (kSmallAngle * 1.01);
  const Eigen::Matrix3d W = SO3d::hat(phi);
  const Eigen::Matrix3d closed_form = rightJacobian(phi);      // closed-form branch
  const Eigen::Matrix3d taylor =
    Eigen::Matrix3d::Identity() - 0.5 * W + W * W / 6.0;       // small-angle branch
  const double jump = (closed_form - taylor).cwiseAbs().maxCoeff();
  assert(jump < 1e-10 && "small-angle branch is discontinuous with the closed form");

  // THE ONE THAT ACTUALLY BITES. Assert the DEFINING property still holds at angles far
  // below the threshold. This is what catches a badly-placed cutoff: with the threshold
  // set too low, the closed form runs at th ~ 1e-8, `1 - cos(th)` cancels to zero in
  // double precision, and Jr silently loses its first-order term. No NaN, no warning --
  // just a wrong Jacobian, exactly where a gyro at rest lives.
  double worst = 0.0;
  for (double th = 1e-1; th > 1e-12; th *= 0.1) {
    const Eigen::Vector3d p = axis * th;
    assert(rightJacobian(p).allFinite());
    assert(rightJacobianInverse(p).allFinite());

    const Eigen::Vector3d d = Eigen::Vector3d(0.7, -0.2, 0.4) * 1e-9;
    const SO3d lhs = SO3d::exp(p + d);
    const SO3d rhs = SO3d::exp(p) * SO3d::exp(rightJacobian(p) * d);
    const double err = (lhs.inverse() * rhs).log().norm();
    worst = std::max(worst, err);
    assert(err < 1e-12 && "Jr wrong at small angle -- cancellation in the closed form?");
  }
  std::printf(
    "  small-angle: continuous (%.1e), Jr exact down to 1e-12 (%.1e)   OK\n", jump, worst);
}

// =================================================================================
// 4. THE BIG ONE. Preintegration must reproduce brute-force world-frame integration.
//
//    Preintegration's whole claim is that you can integrate in the first sample's
//    frame, independently of the starting pose, and recover the same answer via:
//
//        R_j = R_i * dR
//        v_j = v_i + g*dt + R_i*dv
//        p_j = p_i + v_i*dt + 1/2 g*dt^2 + R_i*dp
//
//    Brute force is the oracle. If the frame bookkeeping or the gravity handling is
//    wrong, these diverge.
// =================================================================================
static void testMatchesBruteForceIntegration()
{
  // A synthetic IMU stream: spinning and accelerating, so no term is trivially zero.
  const double dt = 0.005;                   // 200 Hz
  const int n = 200;                         // 1 s
  const Eigen::Vector3d bg(0.01, -0.02, 0.005);
  const Eigen::Vector3d ba(0.1, 0.05, -0.08);

  std::vector<Eigen::Vector3d> gyros, accels;
  for (int k = 0; k < n; ++k) {
    const double t = k * dt;
    gyros.push_back(Eigen::Vector3d(0.3 * std::sin(t), 0.2 * std::cos(t), 0.5) + bg);
    accels.push_back(Eigen::Vector3d(0.5 * std::cos(t), -0.4, 9.81 * 0.1 * t) + ba);
  }

  // A non-trivial starting state -- if the deltas secretly depended on it, this is
  // what would expose it.
  const SO3d R_i = SO3d::exp(Eigen::Vector3d(0.3, -0.5, 0.9));
  const Eigen::Vector3d v_i(1.5, -0.3, 0.2);
  const Eigen::Vector3d p_i(10.0, -4.0, 2.0);

  // --- Oracle: integrate in the WORLD frame, gravity included, step by step.
  SO3d R = R_i;
  Eigen::Vector3d v = v_i, p = p_i;
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector3d a_world = R * (accels[k] - ba) + kGravityVec;
    p += v * dt + 0.5 * a_world * dt * dt;    // same ordering as the preintegrator
    v += a_world * dt;
    R = R * SO3d::exp((gyros[k] - bg) * dt);
  }

  // --- Preintegration: no knowledge of R_i, v_i, p_i at all.
  ImuPreintegration pre(bg, ba, 1e-4, 1e-3);
  for (int k = 0; k < n; ++k) {
    pre.integrate(gyros[k], accels[k], dt);
  }

  const double T = pre.dt();
  const SO3d R_j = R_i * pre.dR();
  const Eigen::Vector3d v_j = v_i + kGravityVec * T + R_i * pre.dv();
  const Eigen::Vector3d p_j =
    p_i + v_i * T + 0.5 * kGravityVec * T * T + R_i * pre.dp();

  const double err_R = (R.inverse() * R_j).log().norm();
  const double err_v = (v - v_j).norm();
  const double err_p = (p - p_j).norm();

  assert(std::abs(T - n * dt) < 1e-12);
  assert(err_R < 1e-12 && "preintegrated rotation != brute force");
  assert(err_v < 1e-10 && "preintegrated velocity != brute force");
  assert(err_p < 1e-10 && "preintegrated position != brute force");

  std::printf(
    "  vs brute-force world integration : dR %.1e  dv %.1e  dp %.1e   OK\n",
    err_R, err_v, err_p);
}

// =================================================================================
// 5. The first-order bias correction. This is the OTHER half of preintegration's
//    claim: when the bias estimate moves, you may shift the delta along its
//    Jacobians instead of re-integrating hundreds of samples.
//
//    Oracle: actually re-integrate with the perturbed bias. The corrected delta must
//    match it to first order -- i.e. the residual error must be O(db^2), which we
//    verify by HALVING db and watching the error fall ~4x.
// =================================================================================
static void testBiasFirstOrderCorrection()
{
  const double dt = 0.005;
  const int n = 200;
  const Eigen::Vector3d bg(0.01, -0.02, 0.005);
  const Eigen::Vector3d ba(0.1, 0.05, -0.08);

  std::vector<Eigen::Vector3d> gyros, accels;
  for (int k = 0; k < n; ++k) {
    const double t = k * dt;
    gyros.push_back(Eigen::Vector3d(0.3 * std::sin(t), 0.2 * std::cos(t), 0.5) + bg);
    accels.push_back(Eigen::Vector3d(0.5 * std::cos(t), -0.4, 0.981 * t) + ba);
  }

  ImuPreintegration base(bg, ba, 1e-4, 1e-3);
  for (int k = 0; k < n; ++k) {
    base.integrate(gyros[k], accels[k], dt);
  }

  auto error_for = [&](double scale) {
      const Eigen::Vector3d dbg = Eigen::Vector3d(1.0, -0.5, 0.7) * 1e-3 * scale;
      const Eigen::Vector3d dba = Eigen::Vector3d(-0.6, 0.2, 0.9) * 1e-3 * scale;

      // Oracle: re-integrate from scratch at the shifted bias.
      ImuPreintegration exact(bg + dbg, ba + dba, 1e-4, 1e-3);
      for (int k = 0; k < n; ++k) {
        exact.integrate(gyros[k], accels[k], dt);
      }

      // Cheap path: shift the ORIGINAL delta along its bias Jacobians.
      const double eR = (exact.dR().inverse() * base.dR_corrected(dbg)).log().norm();
      const double eV = (exact.dv() - base.dv_corrected(dbg, dba)).norm();
      const double eP = (exact.dp() - base.dp_corrected(dbg, dba)).norm();
      return std::max({eR, eV, eP});
    };

  const double e1 = error_for(1.0);
  const double e_half = error_for(0.5);

  // First-order accurate => error is O(db^2) => halving db should quarter the error.
  const double ratio = e1 / e_half;
  assert(e1 < 1e-5 && "bias correction is not tracking re-integration");
  assert(ratio > 3.0 && ratio < 5.0 && "bias-correction error is not second-order");

  std::printf(
    "  bias first-order correction      : err %.2e, halving db -> %.1fx smaller   OK\n",
    e1, ratio);
}

// =================================================================================
// 6. Covariance must be symmetric, PSD, and GROW. The last one is the honest
//    statement that dead reckoning is not a measurement: integrate for longer and
//    you know LESS, and the solver must be told so or it will over-trust the IMU.
// =================================================================================
static void testCovarianceGrows()
{
  ImuPreintegration pre(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1e-3, 1e-2);

  const double dt = 0.005;
  double prev_trace = 0.0;
  for (int k = 0; k < 200; ++k) {
    pre.integrate(
      Eigen::Vector3d(0.1, 0.0, 0.2), Eigen::Vector3d(0.0, 0.0, 9.81), dt);

    const Eigen::Matrix<double, 9, 9> & C = pre.covariance();
    const double asym = (C - C.transpose()).cwiseAbs().maxCoeff();
    assert(asym < 1e-12 && "covariance is not symmetric");

    const double trace = C.trace();
    assert(trace >= prev_trace - 1e-15 && "covariance shrank -- the IMU cannot gain information");
    prev_trace = trace;
  }

  // PSD: the smallest eigenvalue must not be meaningfully negative.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(pre.covariance());
  assert(es.eigenvalues().minCoeff() > -1e-12 && "covariance is not PSD");

  std::printf(
    "  covariance: symmetric, PSD, grows  (trace %.3e after 1 s)   OK\n", prev_trace);
}

int main()
{
  std::printf("test_preintegration: on-manifold IMU preintegration\n");
  testRightJacobianDefinition();
  testRightJacobianInverse();
  testSmallAngleBranch();
  testMatchesBruteForceIntegration();
  testBiasFirstOrderCorrection();
  testCovarianceGrows();
  std::printf("test_preintegration: all checks passed\n");
  return 0;
}
