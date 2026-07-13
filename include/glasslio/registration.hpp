#ifndef GLASSLIO_REGISTRATION_HPP
#define GLASSLIO_REGISTRATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glasslio/data_process.h"
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
};

/// Point-to-plane ICP of `source` (sensor frame) onto `map` (world frame),
/// started from `guess`. Solved by Gauss-Newton on the SE(3) manifold.
///
/// Point-to-plane, not point-to-point: a LiDAR never re-samples the same physical
/// points, it hits the same *surface* in different spots. Penalising only the
/// distance ALONG the surface normal lets points slide freely across the surface,
/// which is exactly what the geometry does or does not constrain.
RegistrationResult alignPointToPlane(
  const CloudXYZI & source,
  const LocalMap & map,
  const Eigen::Isometry3d & guess,
  const RegistrationParams & params);

}  // namespace glasslio

#endif  // GLASSLIO_REGISTRATION_HPP
