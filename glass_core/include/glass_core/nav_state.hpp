#ifndef GLASS_CORE_NAV_STATE_HPP
#define GLASS_CORE_NAV_STATE_HPP

#include <Eigen/Core>

#include "sophus/so3.hpp"

namespace glass_core
{

/// The 15-DoF navigation state that a tightly-coupled estimator solves for.
///
///     x = ( R, p, v, b_g, b_a )
///
/// WHY EACH ONE HAS TO BE HERE. A quantity belongs in the state if a residual mentions
/// it and we do not already know it:
///
///   R, p   -- the pose. What we actually want. The LiDAR constrains these.
///   v      -- VELOCITY. The IMU's dv residual relates v_j to v_i, so the solver needs
///             a variable to move. Loose coupling had no such variable and instead
///             finite-differenced velocity from the very poses it was helping produce
///             -- a feedback loop, and the origin of the constant-velocity runaway.
///   b_g,   -- BIASES. The preintegrated deltas were computed at some assumed bias. If
///   b_a       the truth differs, the deltas are wrong. Estimate them and we can shift
///             the deltas along their Jacobians instead of re-integrating.
///
/// Gravity is NOT a state here: it is fixed from the init window and held. (Estimating
/// it online is the natural next extension, and would make this 18-DoF.)
struct NavState
{
  Sophus::SO3d R;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
};

/// Dimension of the error state, and the index of each block within it.
///
/// THE ORDERING IS A CONTRACT. Every Jacobian in the system writes its columns at these
/// offsets. Get one wrong and the solver applies the rotation correction to the
/// accelerometer bias -- which will not crash, will not NaN, and will simply produce a
/// slowly, confidently wrong trajectory. Named constants, never bare numbers.
enum : int
{
  kIdxPhi = 0,    ///< rotation      (3)
  kIdxPos = 3,    ///< position      (3)
  kIdxVel = 6,    ///< velocity      (3)
  kIdxBg = 9,     ///< gyro bias     (3)
  kIdxBa = 12,    ///< accel bias    (3)
  kNavDim = 15
};

using NavVec = Eigen::Matrix<double, kNavDim, 1>;

/// The RETRACTION (boxplus): x <- x [+] dx.
///
/// Only the rotation is curved. Everything else is R^3 and adds normally -- which is
/// the whole reason this state is manageable: a 15-DoF manifold with exactly ONE
/// non-trivial block.
///
/// The rotation composes on the RIGHT (body frame):  R <- R * Exp(dphi).
///
/// >> This is the opposite side from optimizeSE3 (which perturbs on the LEFT, because
/// >> its correction lives in the world frame). Both are correct; they are different
/// >> conventions, and the Jacobians are NOT interchangeable between them. We adopt the
/// >> right convention here because the preintegration Jacobians are derived for it.
/// >> Every Jacobian handed to the tightly-coupled solver must therefore be d r / d dx
/// >> under THIS retraction. Pinned by finite differences in test_nav_residual.cpp.
inline NavState boxplus(const NavState & x, const NavVec & dx)
{
  NavState out;
  out.R = x.R * Sophus::SO3d::exp(dx.segment<3>(kIdxPhi));   // manifold: compose
  out.p = x.p + dx.segment<3>(kIdxPos);                      // vector space: add
  out.v = x.v + dx.segment<3>(kIdxVel);
  out.bg = x.bg + dx.segment<3>(kIdxBg);
  out.ba = x.ba + dx.segment<3>(kIdxBa);
  return out;
}

/// The INVERSE of boxplus (boxminus):  x [-] y  =  the dx for which  y [+] dx  ==  x.
///
/// The difference between two states, expressed in the tangent space at `y`. It exists for
/// one reason: a PRIOR on the state. "x should be near y, with covariance P" is not
/// expressible without it, because the rotation block cannot be subtracted -- Log of the
/// RELATIVE rotation is what a difference means on SO(3), exactly as in imuResidual.
///
/// THE SIDE MATTERS, and it is fixed by boxplus. Since boxplus composes on the RIGHT
/// (R <- R Exp(dphi)), inverting it gives Log(y.R^-1 x.R), NOT Log(x.R y.R^-1). Swap them and
/// the residual is still small, still shrinks, and pulls the state to the wrong place.
///
/// Exactness is the contract: boxminus(boxplus(x, dx), x) == dx for any dx in the domain of
/// Log. Pinned in test_nav_residual.cpp.
inline NavVec boxminus(const NavState & x, const NavState & y)
{
  NavVec dx;
  dx.segment<3>(kIdxPhi) = (y.R.inverse() * x.R).log();   // manifold: Log of the relative
  dx.segment<3>(kIdxPos) = x.p - y.p;                     // vector space: subtract
  dx.segment<3>(kIdxVel) = x.v - y.v;
  dx.segment<3>(kIdxBg) = x.bg - y.bg;
  dx.segment<3>(kIdxBa) = x.ba - y.ba;
  return dx;
}

}  // namespace glass_core

#endif  // GLASS_CORE_NAV_STATE_HPP
