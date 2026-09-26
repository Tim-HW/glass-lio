#ifndef GLASSLIO_REGISTRATION_HPP
#define GLASSLIO_REGISTRATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glasslio/types.hpp"   // CloudXYZI
#include "glasslio/local_map.hpp"

namespace glasslio
{

struct RegistrationParams
{
  double max_correspondence_distance = 1.0;
  int max_iterations = 30;
  /// Converged once the incremental update is smaller than these.
  double eps_translation = 1e-3;   ///< m
  double eps_rotation = 1e-4;      ///< rad
  /// Huber threshold (m). Residuals beyond this get down-weighted linearly
  /// instead of quadratically, so a few bad correspondences cannot dominate.
  double huber_delta = 0.2;
  /// Below this many correspondences the problem is under-constrained; refuse.
  int min_correspondences = 50;
  /// DEGENERACY GATE. H_t = sum(w_i * normal_i * normal_i^T), the translation block of
  /// H, built purely from correspondence normals. Its smallest eigenvalue divided by
  /// its trace is the RATIO checked here -- NOT the raw eigenvalue, which scales with
  /// correspondence count and so cannot tell "under-constrained" from "just fewer
  /// points". The ratio is count-invariant: a healthy scene's normals span all three
  /// directions roughly evenly (ratio ~0.1-0.35 either way); a scene dominated by
  /// near-parallel planes (an open outdoor stretch: mostly ground plus one or two
  /// walls) leaves one direction with near-zero normal support relative to the others.
  /// Calibrated empirically -- see docs/implementation/5-registration.md -- against the indoor test
  /// bag (ratio never below 0.117 at the 5th percentile) and the Outdoor01 divergence
  /// (ratio never above 0.034): the two do not overlap.
  double min_translation_eigenvalue_ratio = 0.05;
};

struct RegistrationResult
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  /// The solve produced a usable pose: enough correspondences, finite solution.
  /// This -- not `converged` -- is what decides whether the pose is trustworthy.
  bool valid = false;

  /// The update fell below eps before max_iterations. NOT a trust signal:
  /// ICP routinely plateaus above eps while sitting on a perfectly good fit, and
  /// treating "hit max_iterations" as failure throws away good poses.
  bool converged = false;

  int iterations = 0;
  int correspondences = 0;
  double rmse = 0.0;     ///< RMS point-to-plane residual (m) at the solution
  /// Smallest-eigenvalue/trace ratio of the translation block of H at the solution --
  /// see RegistrationParams::min_translation_eigenvalue_ratio. 0 if never computed
  /// (e.g. the solve was already rejected on correspondence count).
  double translation_eigenvalue_ratio = 0.0;
};

/// The point-to-plane residual: signed distance from the transformed source point
/// `q = T * p` to the plane `(centroid, normal)`.
///
///     r(T) = n^T (T p - c)
///
/// Point-to-plane, not point-to-point: a LiDAR never re-samples the same physical
/// points, it hits the same *surface* in different spots. Penalising only the
/// distance ALONG the normal lets points slide freely ACROSS the surface, which is
/// exactly what a wall does and does not constrain.
inline double pointToPlaneResidual(
  const Eigen::Vector3d & q, const Eigen::Vector3d & centroid, const Eigen::Vector3d & normal)
{
  return normal.dot(q - centroid);
}

/// dr/dxi for the residual above, under the LEFT perturbation T <- Exp(xi) * T
/// used by optimizeSE3 (see gauss_newton.hpp).
///
/// Linearise Exp(xi) acting on q, to first order:
///     Exp(xi) q  ~=  q + rho + phi x q
///     r(xi)      ~=  r + n^T rho + n^T (phi x q)
///                 =  r + n^T rho + (q x n)^T phi      [scalar triple product]
/// so, with xi = [rho; phi]:
///     J = [ n^T , (q x n)^T ]
///
/// The rotation block is (q x n), NOT (n x q): swapping them flips the sign of
/// every rotational update, and the solver will then walk away from the solution
/// while still reporting a plausible residual. Pinned by test_jacobian.cpp.
inline Eigen::Matrix<double, 1, 6> pointToPlaneJacobian(
  const Eigen::Vector3d & q, const Eigen::Vector3d & normal)
{
  Eigen::Matrix<double, 1, 6> J;
  J.head<3>() = normal.transpose();
  J.tail<3>() = q.cross(normal).transpose();
  return J;
}

/// Point-to-plane ICP of `source` (sensor frame) onto `map` (world frame),
/// started from `guess`. Solved by Gauss-Newton on the SE(3) manifold.
///
/// This function owns only the ICP-specific half of the problem: ASSOCIATION
/// (which plane does each point belong to) and the residual above. The solve
/// itself -- normal equations, robust weighting, retraction -- lives in
/// gauss_newton.hpp and knows nothing about LiDAR.
RegistrationResult alignPointToPlane(
  const CloudXYZI & source,
  const LocalMap & map,
  const Eigen::Isometry3d & guess,
  const RegistrationParams & params);

}  // namespace glasslio

#endif  // GLASSLIO_REGISTRATION_HPP
