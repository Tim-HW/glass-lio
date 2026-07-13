#include "glasslio/registration.hpp"

#include <cmath>

#include "sophus/se3.hpp"

namespace glasslio
{

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

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    // Gauss-Newton normal equations for a 6-DoF increment xi = [rho; phi]
    // (translation; rotation), applied as a LEFT perturbation: T <- Exp(xi) * T.
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

    double sq_err = 0.0;
    int n = 0;

    for (const auto & pt : source.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const Eigen::Vector3d p(pt.x, pt.y, pt.z);
      const Eigen::Vector3d q = T * p;               // source point in world

      // ASSOCIATE: nearest cached plane in the 27-cell neighbourhood.
      Plane plane;
      if (!map.closestPlane(q, params.max_correspondence_distance, plane)) {
        continue;
      }

      // Point-to-plane residual: signed distance along the surface normal.
      const double r = plane.normal.dot(q - plane.centroid);

      // Jacobian of r wrt the left perturbation xi.
      //   Exp(xi) * q  ~=  q + rho + phi x q
      //   r(xi) ~= r + n^T rho + n^T (phi x q)
      //          = r + n^T rho + (q x n)^T phi
      Eigen::Matrix<double, 1, 6> J;
      J.head<3>() = plane.normal.transpose();
      J.tail<3>() = q.cross(plane.normal).transpose();

      // Huber: quadratic near zero, linear in the tail, so a handful of bad
      // correspondences (moving objects, new geometry) cannot dominate the solve.
      const double abs_r = std::abs(r);
      const double w = (abs_r <= params.huber_delta)
        ? 1.0
        : params.huber_delta / abs_r;

      H.noalias() += w * J.transpose() * J;
      b.noalias() -= w * J.transpose() * r;

      sq_err += w * r * r;
      ++n;
    }

    result.correspondences = n;
    if (n < params.min_correspondences) {
      result.valid = false;
      return result;   // under-constrained: refuse rather than invent a pose
    }

    // Solve H xi = b. LDLT is fine: H is symmetric positive semi-definite.
    const Eigen::Matrix<double, 6, 1> xi = H.ldlt().solve(b);
    if (!xi.allFinite()) {
      result.valid = false;   // degenerate geometry; do NOT hand back a pose
      return result;
    }

    // Update on the manifold. Left perturbation, so compose on the left.
    T = Sophus::SE3d::exp(xi) * T;

    result.iterations = iter + 1;
    result.rmse = std::sqrt(sq_err / static_cast<double>(n));
    // We have enough correspondences and a finite solve: the pose is usable,
    // whether or not the update has settled below eps yet.
    result.valid = true;

    const double d_rho = xi.head<3>().norm();
    const double d_phi = xi.tail<3>().norm();
    if (d_rho < params.eps_translation && d_phi < params.eps_rotation) {
      result.converged = true;
      break;
    }
  }

  result.pose.linear() = T.rotationMatrix();
  result.pose.translation() = T.translation();
  return result;
}

}  // namespace glasslio
