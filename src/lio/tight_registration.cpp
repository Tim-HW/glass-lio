#include "glasslio/tight_registration.hpp"

#include <cmath>

#include "glass_core/gauss_newton.hpp"
#include "glass_core/nav_residual.hpp"

namespace glasslio
{

using NavEquations = NormalEquationsN<kNavDim>;

// predictState() moved to glass_core/nav_residual.hpp, beside the imuResidual it is the
// forward dual of. It is engine, not LiDAR, and glassvio needs it too.

TightResult alignTightlyCoupled(
  const CloudXYZI & source,
  const LocalMap & map,
  const NavState & xi,
  const ImuPreintegration & pre,
  const Eigen::Vector3d & gravity,
  const NavState & guess,
  const Eigen::Matrix<double, 6, 6> & bias_information,
  const TightParams & params)
{
  TightResult result;
  result.state = guess;

  if (map.empty() || source.empty()) {
    return result;
  }

  // --- Information matrices, built once. -------------------------------------
  //
  // The IMU factor is weighted by the INVERSE of the covariance preintegration
  // accumulated. This is what makes the fusion self-tuning: a longer gap between scans
  // means a bigger Sigma, hence a smaller Sigma^-1, hence less pull from the IMU. No
  // heuristic decides that -- the propagated uncertainty does.
  Eigen::Matrix<double, 9, 9> imu_information =
    pre.covariance().inverse() * params.imu_prior_weight;
  if (!imu_information.allFinite()) {
    imu_information.setZero();   // a singular covariance means we learned nothing
  }

  // The bias prior comes from the CALLER, which carries a covariance and updates it with
  // what each solve learns. It deliberately does NOT come from a random-walk constant:
  // a random walk says how fast a bias may DRIFT, never how wrong it might have been to
  // begin with -- and an accel bias is exactly a quantity you start out wrong about.

  NavState x = guess;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    NavEquations eq;

    // --- 1. LiDAR: one scalar residual per correspondence, Huber-weighted.
    //
    // WHITENED by lidar_sigma. Dividing the residual and its Jacobian by sigma gives
    // each point an information of 1/sigma^2 -- which is what puts it on a common
    // footing with the IMU's Sigma^-1. Without this the two sensors are being compared
    // in different units and the weighting is meaningless. (Huber's threshold is
    // whitened too, so `huber_delta` keeps its natural units of metres.)
    const double inv_sigma = 1.0 / params.lidar_sigma;
    const double huber_whitened = params.huber_delta * inv_sigma;

    double lidar_sq_err = 0.0;
    int n_corr = 0;
    for (const auto & pt : source.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const Eigen::Vector3d p_sensor(pt.x, pt.y, pt.z);
      const Eigen::Vector3d q = x.R * p_sensor + x.p;   // into the world

      Plane plane;
      if (!map.closestPlane(q, params.max_correspondence_distance, plane)) {
        continue;
      }

      const double r = pointToPlaneResidualNav(x, p_sensor, plane.centroid, plane.normal);
      eq.addScalar(
        r * inv_sigma,
        pointToPlaneJacobianNav(x, p_sensor, plane.normal) * inv_sigma,
        huber_whitened);

      lidar_sq_err += r * r;   // raw, so the reported rmse stays in metres
      ++n_corr;
    }

    result.correspondences = n_corr;
    if (n_corr < params.min_correspondences) {
      result.valid = false;   // under-constrained: refuse rather than invent a pose
      return result;
    }

    // --- 2. The IMU factor: one 9-vector, weighted by its own information.
    eq.addBlock<9>(
      imuResidual(xi, x, pre, gravity), imuJacobian(xi, x, pre), imu_information);

    // --- 3. The bias random-walk prior: keeps the biases from absorbing real motion
    //        in the (common) case where they are unobservable.
    eq.addBlock<6>(biasResidual(xi, x), biasJacobian(), bias_information);

    // --- Solve and retract. Identical to the SE(3) case, just wider.
    const NavVec dx = eq.solve();
    if (!dx.allFinite()) {
      result.valid = false;
      return result;
    }

    x = boxplus(x, dx);   // RIGHT perturbation on R; additive elsewhere
    result.H = eq.H();    // hand the information back so the caller can shrink its P

    result.iterations = iter + 1;
    result.valid = true;
    // Report the LiDAR-only RMSE, so it is directly comparable with the loose path's.
    result.rmse = std::sqrt(lidar_sq_err / static_cast<double>(n_corr));

    if (dx.segment<3>(kIdxPos).norm() < params.eps_translation &&
      dx.segment<3>(kIdxPhi).norm() < params.eps_rotation)
    {
      result.converged = true;
      break;
    }
  }

  result.state = x;
  return result;
}

}  // namespace glasslio
