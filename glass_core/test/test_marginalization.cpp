// Self-check for Schur-complement marginalization -- the engine kernel of a fixed-lag
// smoother (roadmap Phase 2). The oracle is the DEFINITION: marginalizing a variable and
// solving must give the same answer on the kept variables as solving the full system.
//
// It also pins the thing marginalization is FOR: the naive alternative -- holding the
// dropped variable fixed and independent (a block-diagonal solve) -- must give a DIFFERENT
// answer, or the cross-coupling that carries information forward was doing nothing. That
// naive path is exactly what glasslio's one-shot tight solve does to x_i, and why it
// freezes. See doc/7-tight-coupling.md.
#include <cassert>
#include <cstdio>
#include <random>

#include <Eigen/Dense>

#include "glass_core/marginalization.hpp"

using glass_core::schurMarginalize;

int main()
{
  std::printf("test_marginalization: Schur complement of a linear-Gaussian system\n");

  std::mt19937 rng(7);
  std::normal_distribution<double> nd(0.0, 1.0);

  const int n = 12;   // total state
  const int m = 5;    // marginalize the first 5, keep the last 7

  // A dense, well-conditioned SPD information matrix H = A^T A + ridge, and a random b.
  // Dense on purpose: if the marginalized and kept blocks were uncoupled (H_mk = 0),
  // marginalization would be trivial and the negative check below could not fire.
  Eigen::MatrixXd A(n, n);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; ++i) {
    b(i) = nd(rng);
    for (int j = 0; j < n; ++j) {
      A(i, j) = nd(rng);
    }
  }
  const Eigen::MatrixXd H = A.transpose() * A + Eigen::MatrixXd::Identity(n, n) * n;

  // --- The full solve, then read off the kept block.
  const Eigen::VectorXd x_full = H.ldlt().solve(b);
  const Eigen::VectorXd xk_full = x_full.tail(n - m);

  // --- Marginalize m out, then solve the reduced system.
  Eigen::MatrixXd H_marg;
  Eigen::VectorXd b_marg;
  schurMarginalize(H, b, m, H_marg, b_marg);
  const Eigen::VectorXd xk_marg = H_marg.ldlt().solve(b_marg);

  const double err = (xk_full - xk_marg).cwiseAbs().maxCoeff();
  assert(err < 1e-9 && "marginalized solve disagrees with the full solve on the kept block");

  // --- The reduced matrix must stay symmetric (a prior that isn't makes later solves rot).
  const double asym = (H_marg - H_marg.transpose()).cwiseAbs().maxCoeff();
  assert(asym < 1e-12 && "marginal information matrix is not symmetric");

  // --- NEGATIVE / the whole point: "hold m fixed and independent" = ignore the coupling =
  //     solve H_kk x_k = b_k. This is the freeze. It must DISAGREE with the truth, or the
  //     information the dropped block carried forward was zero and there was nothing to fix.
  const Eigen::MatrixXd Hkk = H.bottomRightCorner(n - m, n - m);
  const Eigen::VectorXd bk = b.tail(n - m);
  const Eigen::VectorXd xk_held_fixed = Hkk.ldlt().solve(bk);
  const double err_naive = (xk_full - xk_held_fixed).cwiseAbs().maxCoeff();
  assert(err_naive > 1e-2 && "holding the dropped block fixed made no difference -- weak fixture");

  std::printf(
    "  Schur == full solve       : max err %.1e   OK\n"
    "  hold-fixed (the freeze)   : off by %.2e   OK (marginalization is load-bearing)\n"
    "  marginal H symmetric      : asym %.1e   OK\n",
    err, err_naive, asym);
  std::printf("test_marginalization: all checks passed\n");
  return 0;
}
