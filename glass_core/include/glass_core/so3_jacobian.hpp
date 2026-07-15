#ifndef GLASS_CORE_SO3_JACOBIAN_HPP
#define GLASS_CORE_SO3_JACOBIAN_HPP

#include <cmath>

#include <Eigen/Core>

#include "sophus/so3.hpp"

namespace glass_core
{

/// The RIGHT JACOBIAN of SO(3), and its inverse.
///
/// Sophus gives us Exp, Log, hat and vee. It does NOT give us this -- and it is the
/// piece every nontrivial derivative on SO(3) needs, so we write it out.
///
/// WHAT IT IS. On a vector space, d/dx of (x + delta) is trivially the identity. On a
/// manifold it is not, because you cannot add. The right Jacobian is the correction
/// factor that makes the chain rule work:
///
///     Exp(phi + delta)  ~=  Exp(phi) * Exp( Jr(phi) * delta )        (delta small)
///
/// Read it as: "perturbing the tangent vector by delta does NOT rotate the result by
/// delta -- it rotates it by Jr(phi)*delta." The manifold's curvature stretches and
/// twists the perturbation on its way through Exp, and Jr is exactly that distortion.
///
/// Its inverse runs the other way, and is what you need to differentiate a Log:
///
///     Log( Exp(phi) * Exp(delta) )  ~=  phi + Jr^-1(phi) * delta
///
/// This is the term people silently drop -- it looks like a second-order detail, and
/// omitting it leaves an estimator that still converges but is systematically
/// overconfident, precisely in the high-rotation moments where it matters most.
///
/// Note Jr(phi) -> I as phi -> 0: with no rotation there is no curvature to correct
/// for, and the manifold looks locally like the vector space you wish it were.

/// Below this angle, use the Taylor expansion instead of the closed form.
///
/// THE NUMERICAL CLIFF, and why the threshold is 1e-4 and not 1e-12.
///
/// The closed forms below divide by theta^2 and theta^3, so theta = 0 is 0/0 -- and
/// theta = 0 is not an exotic input, it is the single MOST COMMON one (a gyro at rest,
/// integrated over a 5 ms step). That much is obvious, and it tempts you to set the
/// threshold as small as possible "so the accurate closed form runs almost always".
///
/// That is exactly backwards. The closed form goes numerically WRONG long before it
/// goes undefined. Consider `(1 - cos th) / th^2` at th = 1e-8: mathematically it is
/// ~0.5, but in double precision cos(1e-8) rounds to exactly 1.0, so the numerator
/// evaluates to ZERO and the whole term vanishes. Catastrophic cancellation -- the
/// relative error is ~eps/th^2, which reaches 100% around th ~ 1e-8.
///
/// So the branch is not protecting against a division by zero. It is protecting
/// against subtracting two nearly-equal floats. Cross over while the closed form is
/// still ACCURATE (th ~ 1e-4, where eps/th^2 ~ 1e-8) and carry a second-order Taylor
/// term so the approximation is good there too. Both branches are then correct in the
/// overlap, which is the property the tests actually assert.
inline constexpr double kSmallAngle = 1e-4;

/// Jr(phi): Exp(phi + delta) ~= Exp(phi) * Exp(Jr(phi) delta)
///
///   Jr = I - ((1 - cos th) / th^2) * phi^  +  ((th - sin th) / th^3) * (phi^)^2
inline Eigen::Matrix3d rightJacobian(const Eigen::Vector3d & phi)
{
  const double th2 = phi.squaredNorm();
  const Eigen::Matrix3d W = Sophus::SO3d::hat(phi);

  if (th2 < kSmallAngle * kSmallAngle) {
    // Taylor, to second order: the coefficients above tend to 1/2 and 1/6.
    //   Jr = I - phi^/2 + (phi^)^2/6 + O(th^3)
    // Error at the threshold is ~th^3 ~ 1e-12 -- far better than the closed form
    // manages down here, and finite at phi = 0.
    return Eigen::Matrix3d::Identity() - 0.5 * W + W * W / 6.0;
  }

  const double th = std::sqrt(th2);
  const double a = (1.0 - std::cos(th)) / th2;
  const double b = (th - std::sin(th)) / (th2 * th);
  return Eigen::Matrix3d::Identity() - a * W + b * W * W;
}

/// Jr^-1(phi): Log(Exp(phi) * Exp(delta)) ~= phi + Jr^-1(phi) delta
///
///   Jr^-1 = I + phi^/2 + (1/th^2 - (1 + cos th) / (2 th sin th)) * (phi^)^2
///
/// DOMAIN: |phi| < pi. Note the `sin th` in the denominator -- Jr^-1 is genuinely
/// SINGULAR at th = pi, and that is not an artefact of this formula. At a half-turn,
/// Log stops being a function: Exp(phi) and Exp(-phi) are the same rotation, so there
/// is no unique tangent vector to differentiate, and the derivative blows up.
///
/// In practice this is a non-issue for us: we apply it to rotation INCREMENTS (one IMU
/// step, or the error between two nearby estimates), which are tiny. A single 5 ms gyro
/// step would need >600 rad/s to reach pi. But it is a real cliff, and a preintegration
/// interval long enough to accumulate a half-turn of ERROR is one you should not be
/// linearising about anyway.
inline Eigen::Matrix3d rightJacobianInverse(const Eigen::Vector3d & phi)
{
  const double th2 = phi.squaredNorm();
  const Eigen::Matrix3d W = Sophus::SO3d::hat(phi);

  if (th2 < kSmallAngle * kSmallAngle) {
    // Taylor: the (phi^)^2 coefficient tends to 1/12 (not 1/6 -- it is NOT simply the
    // sign-flipped Jr). Same cancellation argument as above.
    return Eigen::Matrix3d::Identity() + 0.5 * W + W * W / 12.0;
  }

  const double th = std::sqrt(th2);
  const double c = 1.0 / th2 - (1.0 + std::cos(th)) / (2.0 * th * std::sin(th));
  return Eigen::Matrix3d::Identity() + 0.5 * W + c * W * W;
}

}  // namespace glass_core

#endif  // GLASS_CORE_SO3_JACOBIAN_HPP
