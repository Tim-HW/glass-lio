#include "glasslio/registration.hpp"

#include <cmath>

#include "glass_core/gauss_newton.hpp"
#include "sophus/se3.hpp"

namespace glasslio
{

using namespace glass_core;  // the Gauss-Newton solver (NOLINT: build/namespaces)

RegistrationResult alignPointToPlane(
  const CloudXYZI & source,
  const LocalMap & map,
  const Eigen::Isometry3d & guess,
  const RegistrationParams & params)
{
  RegistrationResult result;
  result.pose = guess;

  if (map.empty() || source.empty()) {
    return result;
  }

  Sophus::SE3d T(guess.linear(), guess.translation());

  GaussNewtonParams gn_params;
  gn_params.max_iterations = params.max_iterations;
  gn_params.eps_translation = params.eps_translation;
  gn_params.eps_rotation = params.eps_rotation;
  gn_params.huber_delta = params.huber_delta;
  gn_params.min_residuals = params.min_correspondences;

  // ICP alternates: you need the pose to find the correspondences, and the
  // correspondences to find the pose. This lambda is the ASSOCIATE half, and the
  // solver re-runs it every iteration -- that re-association IS the ICP loop.
  // Everything else (H, b, Huber, LDLT, retraction) is generic least squares and
  // lives in gauss_newton.hpp, which has never heard of a point cloud.
  const auto associate = [&](const Sophus::SE3d & T_cur, NormalEquations & eq) {
      for (const auto & pt : source.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
          continue;
        }
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        const Eigen::Vector3d q = T_cur * p;   // source point in the world frame

        // Correspondence is a HASH LOOKUP, not a KD-tree query: the map caches one
        // plane per voxel, so we hash q and scan the 27-cell neighbourhood.
        Plane plane;
        if (!map.closestPlane(q, params.max_correspondence_distance, plane)) {
          continue;
        }

        eq.add(
          pointToPlaneResidual(q, plane.centroid, plane.normal),
          pointToPlaneJacobian(q, plane.normal),
          params.huber_delta);
      }
    };

  const GaussNewtonResult gn = optimizeSE3(T, gn_params, associate);

  result.valid = gn.valid;
  result.converged = gn.converged;
  result.iterations = gn.iterations;
  result.correspondences = gn.residuals;
  result.rmse = gn.rmse;

  result.pose.linear() = T.rotationMatrix();
  result.pose.translation() = T.translation();
  return result;
}

}  // namespace glasslio
